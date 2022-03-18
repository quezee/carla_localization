#include <cmath>
#include "kalman.h"
using namespace Eigen;

KalmanFilter::KalmanFilter(const VectorXd& meas_noise, double std_j, double std_wd,
                           size_t n_x, size_t n_aug, size_t n_z)
    : n_x(n_x), n_aug(n_aug), n_z(n_z), n_sigmas(1 + 2*n_aug), lambda(3 - n_aug)
    , var_j(pow(std_j, 2)), var_wd(pow(std_wd, 2))
{
    x.setZero(n_x, 1);
    x_aug.setZero(n_aug, 1);
    P.setIdentity(n_x, n_x);
    P_aug.setZero(n_aug, n_aug);
    L.setZero(n_aug, n_aug);
    Xsig_aug.setZero(n_aug, n_sigmas);
    Xsig_pred.setZero(n_x, n_sigmas);
    Zsig_pred.setZero(n_z, n_sigmas);
    weights.setZero(n_sigmas, 1);
    z.setZero(n_z, 1);
    z_pred.setZero(n_z, 1);
    S.setZero(n_z, n_z);
    T.setZero(n_x, n_z);
    K.setZero(n_x, n_z);
    I.setIdentity(n_x, n_x);

    H.setZero(n_z, n_x);
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 3) = 1;
    H(3, 4) = 1;
    H(4, 5) = 1;

    R.setZero(n_z, n_z);
    R.diagonal() << meas_noise;
}

void KalmanFilter::CalculateSigmaPoints() {
    // set augmented mean vector
    x_aug.setZero();
    x_aug.topRows(n_x) = x;

    // set augmented state covariance
    P_aug.topLeftCorner(n_x, n_x) = P;
    P_aug(n_x, n_x)     = var_j;
    P_aug(n_x+1, n_x+1) = var_wd;

    // calculate square root of P_aug
    L = P_aug.llt().matrixL();

    // calculate augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug; ++i) {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
        Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
    }  
}

void KalmanFilter::PredictSigmaPoints(double dt) {
    for (int i = 0; i < Xsig_pred.cols(); i++) {
        double px  = Xsig_aug(0, i);
        double py  = Xsig_aug(1, i);
        double v   = Xsig_aug(2, i);
        double a   = Xsig_aug(3, i);
        double yaw = Xsig_aug(4, i);
        double w   = Xsig_aug(5, i);
        double d_px, d_py;
        if (!w) {
            d_px = ((a*dt/2) + v) * cos(yaw) * dt;
            d_py = ((a*dt/2) + v) * sin(yaw) * dt;
        } else {
            d_px = ((v + a*dt) * sin(yaw + w*dt) - v*sin(yaw)) / w;
            d_px += a * (cos(yaw + w*dt) - cos(yaw)) / pow(w, 2);
            d_py = ((v + a*dt) * -cos(yaw + w*dt) + v*cos(yaw)) / w;
            d_py += a * (sin(yaw + w*dt) - sin(yaw)) / pow(w, 2);
        }
        VectorXd p(n_x);
        p << d_px, d_py, a*dt, 0, w*dt, 0;

        double q_j  = Xsig_aug(6, i);
        double q_wd = Xsig_aug(7, i);
        VectorXd q(n_x);
        q << cos(yaw) * q_j * pow(dt, 3) / 6,
             sin(yaw) * q_j * pow(dt, 3) / 6,
             q_j * pow(dt, 2) / 2,
             q_j * dt,
             q_wd * pow(dt, 2) / 2,
             q_wd * dt;
        Xsig_pred.col(i) = Xsig_aug.col(i).topRows(n_x) + p + q;
    }
}

void KalmanFilter::PredictMeanAndCovariance() {
    // set weights vector
    weights.fill(1 / (2*(lambda + n_aug)));
    weights(0) = lambda / (lambda + n_aug);

    // calculate predicted state
    x = Xsig_pred * weights;

    // calculate predicted covariance matrix
    P.setZero();
    VectorXd x_diff (n_x);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig_pred.col(i) - x;
        P += weights(i) * x_diff * x_diff.transpose();
    }
}

void KalmanFilter::PredictMeasurement() {
    // calculate predicted measurements
    Zsig_pred = H * Xsig_pred;
    z_pred = Zsig_pred * weights;

    // calculate predicted measurements covar.
    S = R;
    VectorXd z_diff (n_z);
    for (int i = 0; i < weights.rows(); ++i) {
        z_diff = Zsig_pred.col(i) - z_pred;
        S += weights(i) * z_diff * z_diff.transpose();
    }
}

// void KalmanFilter::RegisterMeasurement(const Measurement& meas) {
//     z << meas.x, meas.y, meas.a, meas.yaw, meas.w;
// }

void KalmanFilter::Update(const Measurement& meas, double dt) {
    CalculateSigmaPoints();
    PredictSigmaPoints(dt);
    PredictMeanAndCovariance();
    PredictMeasurement();

    // calculate cross-corr. between sigmas in state vs measurement spaces
    T.setZero();
    VectorXd x_diff (n_x);
    VectorXd z_diff (n_z);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig_pred.col(i) - x;
        z_diff = Zsig_pred.col(i) - z_pred;
        T += weights(i) * x_diff * z_diff.transpose();
    }
    
    K = T * S.inverse();
    z << meas.x, meas.y, meas.a, meas.yaw, meas.w;
    x += K * (z - z_pred);
    x(4) = fmod(x(4), 2*M_PI);
    P -= K * S * K.transpose();    
}

Pose KalmanFilter::getPose() const {
    return {Point(x(0), x(1), 0),
            Rotate(x(4), 0, 0)};
}
