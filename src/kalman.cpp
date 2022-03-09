#include <cmath>
#include "kalman.h"
using namespace Eigen;

KalmanFilter::KalmanFilter(double var_x, double var_y, double var_yaw, double var_w,
                           double std_a, double std_wd)
    : var_x(var_x), var_y(var_y), var_yaw(var_yaw), var_w(var_w)
    , var_a(pow(std_a, 2)), var_wd(pow(std_wd, 2))
{
    x.setZero();

    P.setIdentity();
    P *= 0.01;
    // P(2, 2) = 1000;

    H.setZero();
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 3) = 1;
    H(3, 4) = 1;

    R.setZero();
    R.diagonal() << var_x, var_y, var_yaw, var_w;

    I.setIdentity();
}

void KalmanFilter::CalculateSigmaPoints() {
    // set augmented mean vector
    x_aug.setZero();
    x_aug.topRows(5) = x;

    // set augmented state covariance
    P_aug.setZero();
    P_aug.topLeftCorner(5, 5) = P;
    P_aug(5, 5) = var_a;
    P_aug(6, 6) = var_wd;

    // calculate square root of P_aug
    L = P_aug.llt().matrixL();

    // calculate augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug; ++i) {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
        Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
    }  
}

void KalmanFilter::PredictSigmaPoints(double delta_t) {
    for (int i = 0; i < Xsig_pred.cols(); i++) {
        const VectorXd& xi = Xsig_aug.col(i);  
        VectorXd p(5);
        if (!xi(4))
            p << xi(2) * cos(xi(3)) * delta_t,
                 xi(2) * sin(xi(3)) * delta_t,
                 0, 0, 0;
        else
            p << (xi(2)/xi(4)) * (sin(xi(3) + xi(4)*delta_t) - sin(xi(3))),
                 (xi(2)/xi(4)) * (-cos(xi(3) + xi(4)*delta_t) + cos(xi(3))),
                 0,
                 xi(4) * delta_t,
                 0;
        double ql = Xsig_aug(5, i);
        double qy = Xsig_aug(6, i);
        VectorXd q(5);
        q << (pow(delta_t, 2)/2) * cos(xi(3)) * ql,
             (pow(delta_t, 2)/2) * sin(xi(3)) * ql,
             delta_t * ql,
             qy * pow(delta_t, 2) / 2,
             qy * delta_t;
        Xsig_pred.col(i) = xi.topRows(5) + p + q;
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
        z_diff = Zsig_pred.col(i) - z;
        S += weights(i) * z_diff * z_diff.transpose();
    }
}

void KalmanFilter::Update(const Measurement& meas, double delta_t) {
    CalculateSigmaPoints();
    PredictSigmaPoints(delta_t);
    PredictMeanAndCovariance();
    PredictMeasurement();

    // calculate cross-corr. between sigmas in state vs measurement spaces
    T.setZero();
    VectorXd x_diff (n_x);
    VectorXd z_diff (n_z);
    for (int i = 0; i < weights.rows(); ++i) {
        x_diff = Xsig_pred.col(i) - x;
        z_diff = Zsig_pred.col(i) - z;
        T += weights(i) * x_diff * z_diff.transpose();
    }
    
    K = T * S.inverse();
    z << meas.x, meas.y, meas.yaw, meas.w;
    x += K * (z - z_pred);
    x(3) = fmod(x(3), 2*M_PI);
    P -= K * S * K.transpose();    
}


Pose KalmanFilter::getPose() const {
    return {Point(x(0), x(1), 0),
            Rotate(x(3), 0, 0)};
}
