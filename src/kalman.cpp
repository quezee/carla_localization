#include <cmath>
#include "kalman.h"
using namespace Eigen;


KalmanFilter::KalmanFilter(const po::variables_map& vm, const Pose& poseRef)
    : poseRef(poseRef)
    , lastUpdateTime(system_clock::now())
    , var_jerk(vm["kalman.var_jerk"].as<double>())
    , var_yaw_a(vm["kalman.var_yaw_a"].as<double>())
{
    x.setZero();
    Xsig.setZero();

    P.setIdentity();
    P.diagonal() *= vm["kalman.P_init"].as<double>();
    P_a.setZero();

	R_gnss.setZero();
	R_gnss.diagonal() << vm["gnss.var_lat"].as<double>(),
                         vm["gnss.var_lon"].as<double>();

	R_lidar.setZero();
	R_lidar.diagonal() << vm["lidar.var_x"].as<double>(),
                          vm["lidar.var_y"].as<double>(),
						  vm["lidar.var_yaw"].as<double>();

    CalculateSigmaPoints();

    weights.fill(1 / (2*(lambda + n_a)));
    weights(0) = lambda / (lambda + n_a);
}

void KalmanFilter::CalculateSigmaPoints() {
    // set augmented state
    x_a.setZero();
    x_a.topRows(n_x) = x;

    // set augmented state covariance
    P_a.setZero();
    P_a.topLeftCorner(n_x, n_x) = P;
    P_a(n_x, n_x)     = var_jerk;
    P_a(n_x+1, n_x+1) = var_yaw_a;

    // calculate square root of P_a
    Matrix7d L = P_a.llt().matrixL();

    // calculate sigma points
    Xsig.col(0) = x_a;
    for (int i = 0; i < n_a; ++i) {
        Xsig.col(i+1)     = x_a + sqrt(lambda+n_a) * L.col(i);
        Xsig.col(i+1+n_a) = x_a - sqrt(lambda+n_a) * L.col(i);
    }
}

void KalmanFilter::TruncateYaw(double& yaw) {
    if (yaw > M_PI)
        yaw -= 2*M_PI;
    else if (yaw < -M_PI)
        yaw += 2*M_PI;
}

Vector5d KalmanFilter::ApplyMotion(const Vector7d& state, const IMUMeasurement& meas, double dt)
{
    Vector2d pos = state.segment<2>(0);
    Vector2d vel = state.segment<2>(3);
    double yaw = state(2);
    double jerk = state(5);
    double yaw_a = state(6);
    
    Matrix2d rot = Rotation2D<double>(yaw).toRotationMatrix();
    Vector2d accel {meas.ax, meas.ay};

    // add IMU inputs
    pos += dt * vel + pow(dt, 2) / 2 * (rot*accel); // TODO double calc
    vel += dt * rot*accel;
    yaw += dt * meas.yaw_rate;
    // add process noise
    pos.array() += jerk * pow(dt, 3) / 6;
    vel.array() += jerk * pow(dt, 2) / 2;
    yaw += yaw_a * pow(dt, 2) / 2;
    TruncateYaw(yaw);

    Vector5d state_pred; // TODO make inplace
    state_pred << pos, yaw, vel;
    return state_pred;
}

double KalmanFilter::GetTimedelta() {
    auto dt = duration_cast<milliseconds>(
        system_clock::now() - lastUpdateTime
    );
    return dt.count() / 1000.;
}

void KalmanFilter::Predict(const IMUMeasurement& meas) {
    CalculateSigmaPoints();

    double dt = GetTimedelta();
    lastUpdateTime = system_clock::now();

    // apply motion to sigma points
    for (int i = 0; i < Xsig.cols(); i++)
        Xsig.col(i).head<5>() = ApplyMotion(Xsig.col(i), meas, dt);
    
    // calculate predicted state
    x = Xsig.topRows(n_x) * weights;
    // calculate predicted covariance matrix
    P.setZero();
    VectorXd x_diff (n_x);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig.col(i).head<5>() - x;
        TruncateYaw(x_diff(2));
        P += weights(i) * x_diff * x_diff.transpose();
    }
}

void KalmanFilter::CorrectInner(const MatrixXd& Zsig, const VectorXd& z_pred,
                                const MatrixXd& R, const VectorXd& z)
{
    // calculate predicted measurements covar.
    size_t n_z = z.size();
    MatrixXd S = R;
    VectorXd z_diff (n_z);
    for (int i = 0; i < weights.rows(); ++i) {
        z_diff = Zsig.col(i) - z_pred; // TODO Zsig_normed
        TruncateYaw(z_diff(2));
        S += weights(i) * z_diff * z_diff.transpose();
    }
    // calculate cross-corr. between sigmas in state vs measurement spaces
    MatrixXd T (n_x, n_z);
    T.setZero();
    VectorXd x_diff (n_x);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig.col(i).head<5>() - x;
        z_diff = Zsig.col(i) - z_pred;
        TruncateYaw(x_diff(2));
        if (n_z > 2) TruncateYaw(z_diff(2));
        T += weights(i) * x_diff * z_diff.transpose();
    }
    // calculate Kalman gain
    MatrixXd K = T * S.inverse();
    // update state
    VectorXd dx = K * (z - z_pred);
    x += dx;
    TruncateYaw(x(2));
    // update estimation error covar.
    P -= K * S * K.transpose();
    // cout << "P\n" << P.norm() << endl;
    lastUpdateTime = system_clock::now();    
}

void KalmanFilter::Correct(const LidarMeasurement& meas) {
    // calculate predicted measurements
    auto Zsig = Xsig.topRows(meas.dim);
    auto z_pred = x.topRows(meas.dim);
    // compose input measurement
    VectorXd z (meas.dim);
    z << meas.x, meas.y, meas.yaw;
    // commit state correction
    CorrectInner(Zsig, z_pred, R_lidar, z);
}

void KalmanFilter::Correct(const GNSSMeasurement& meas) {
    // calculate predicted measurements
    auto Zsig = Xsig.topRows(meas.dim);
    auto z_pred = x.topRows(meas.dim);
    // compose input measurement
    VectorXd z (meas.dim);
    cg::Location meas_loc = gnss2location({meas.lat, meas.lon, 0});
    meas_loc.x -= poseRef.position.x;
    meas_loc.y -= poseRef.position.y;
    z << meas_loc.x, meas_loc.y;
    // commit state correction
    CorrectInner(Zsig, z_pred, R_gnss, z);
}

Pose KalmanFilter::getPose() const {
    return {Point(x(0), x(1), 0),
            Rotate(x(2), 0, 0)};
}
