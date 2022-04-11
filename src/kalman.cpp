#include <cmath>
#include "kalman.h"
using namespace Eigen;


KalmanFilter::KalmanFilter(const Matrix2d& R_gnss, const Matrix3d& R_lidar, const Matrix5d& Q,
                           double P_init, const Pose& poseRef)
    : R_gnss(R_gnss), R_lidar(R_lidar), Q(Q)
    , poseRef(poseRef), lastUpdateTime(system_clock::now())
{
    x.setZero();

    P.setIdentity();
    P *= P_init;

    CalculateSigmaPoints();

    weights.setZero(n_sigmas, 1);
    weights.fill(1 / (2*(lambda + n_x)));
    weights(0) = lambda / (lambda + n_x);
}

void KalmanFilter::CalculateSigmaPoints() {
    // calculate square root of P_aug
    Matrix5d L = P.llt().matrixL();

    // calculate sigma points
    Xsig.col(0) = x;
    for (int i = 0; i < n_x; ++i) {
        Xsig.col(i+1)     = x + sqrt(lambda+n_x) * L.col(i);
        Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * L.col(i);
    }
}

void KalmanFilter::TruncateYaw(double& yaw) {
    if (yaw > M_PI)
        yaw -= 2*M_PI;
    else if (yaw < -M_PI)
        yaw += 2*M_PI;
}

Vector5d KalmanFilter::ApplyMotion(const Vector5d& state, const IMUMeasurement& meas, double dt)
{
    Vector2d pos = state.segment<2>(0);
    Vector2d vel = state.segment<2>(3);

    double yaw = state(2);
    Matrix2d rot = Rotation2D<double>(yaw).toRotationMatrix().transpose();
    Vector2d accel {meas.ax, meas.ay};

    pos += dt * vel + pow(dt, 2) / 2 * (rot*accel); // TODO double calc
    vel += dt * rot*accel;
    yaw += dt * meas.yaw_rate;
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

    // apply motion to sigma points
    double dt = GetTimedelta();
    for (int i = 0; i < Xsig.cols(); i++)
        Xsig.col(i) = ApplyMotion(Xsig.col(i), meas, dt);
    
    // calculate predicted state
    x = Xsig * weights;
    // calculate predicted covariance matrix
    P = Q;
    VectorXd x_diff (n_x);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig.col(i) - x;
        P += weights(i) * x_diff * x_diff.transpose();
    }
    lastUpdateTime = system_clock::now();
}

void KalmanFilter::Correct(const LidarMeasurement& meas) {
    // calculate predicted measurements
    auto Zsig = Xsig.topRows(meas.dim);
    auto z_pred = x.topRows(meas.dim);

    // calculate predicted measurements covar.
    auto S = R_lidar;
    VectorXd z_diff (meas.dim);
    for (int i = 0; i < weights.rows(); ++i) {
        z_diff = Zsig.col(i) - z_pred; // TODO Zsig_normed
        S += weights(i) * z_diff * z_diff.transpose();
    }

    // calculate cross-corr. between sigmas in state vs measurement spaces
    MatrixXd T (n_x, meas.dim);
    T.setZero();
    VectorXd x_diff (n_x);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig.col(i) - x;
        z_diff = Zsig.col(i) - z_pred;
        T += weights(i) * x_diff * z_diff.transpose();
    }
    
    // calculate Kalman gain
    auto K = T * S.inverse();
    // update state
    VectorXd z (meas.dim);
    z << meas.x, meas.y, meas.yaw;
    VectorXd dx = K * (z - z_pred);
    x += dx;
    TruncateYaw(x(2));
    // update estimation error covar.
    P -= K * S * K.transpose();

    lastUpdateTime = system_clock::now();    
}

void KalmanFilter::Correct(const GNSSMeasurement& meas) {
    // calculate predicted measurements
    auto Zsig = Xsig.topRows(meas.dim);
    auto z_pred = x.topRows(meas.dim);

    // calculate predicted measurements covar.
    auto S = R_gnss;
    VectorXd z_diff (meas.dim);
    for (int i = 0; i < weights.rows(); ++i) {
        z_diff = Zsig.col(i) - z_pred;
        S += weights(i) * z_diff * z_diff.transpose();
    }

    // calculate cross-corr. between sigmas in state vs measurement spaces
    MatrixXd T (n_x, meas.dim);
    T.setZero();
    VectorXd x_diff (n_x);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig.col(i) - x;
        z_diff = Zsig.col(i) - z_pred;
        T += weights(i) * x_diff * z_diff.transpose();
    }

    // calculate Kalman gain
    auto K = T * S.inverse();
    // update state
    VectorXd z (meas.dim);
    cg::Location meas_loc = gnss2location({meas.lat, meas.lon, 0});
    meas_loc.x -= poseRef.position.x;
    meas_loc.y -= poseRef.position.y;
    z << meas_loc.x, meas_loc.y;
    VectorXd dx = K * (z - z_pred);
    x += dx;
    // update estimation error covar.
    P -= K * S * K.transpose();

    lastUpdateTime = system_clock::now();    
}

Pose KalmanFilter::getPose() const {
    return {Point(x(0), x(1), 0),
            Rotate(x(2), 0, 0)};
}
