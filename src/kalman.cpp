#include <cmath>
#include "kalman.h"
using namespace Eigen;

KalmanFilter::KalmanFilter(double var_x, double var_y, double var_yaw,
                           double std_vdd, double std_ydd)
    : var_x(var_x), var_y(var_y), var_yaw(var_yaw), std_vdd(std_vdd), std_ydd(std_ydd)
{
    H.setZero();
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 3) = 1;

    R.setZero();
    R.diagonal() << var_x, var_y, var_yaw;

    I.setIdentity();
}

void KalmanFilter::CalculateSigmaPoints() {
    // set augmented mean vector
    x_aug.setZero();
    x_aug.topRows(5) = x;

    // set augmented state covariance
    P_aug.setZero();
    P_aug.topLeftCorner(5, 5) = P;
    P_aug(5, 5) = pow(std_vdd, 2);
    P_aug(6, 6) = pow(std_ydd, 2);

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
        const VectorXd& x = Xsig_aug.col(i);  
        VectorXd p(5);
        if (!x(4))
            p << x(2) * cos(x(3)) * delta_t,
                 x(2) * sin(x(3)) * delta_t,
                 0, 0, 0;
        else
            p << (x(2)/x(4)) * (sin(x(3) + x(4)*delta_t) - sin(x(3))),
                 (x(2)/x(4)) * (-cos(x(3) + x(4)*delta_t) + cos(x(3))),
                 0,
                 x(4) * delta_t,
                 0;
        double qv = Xsig_aug(5, i);
        double qy = Xsig_aug(6, i);
        VectorXd q(5);
        q << (pow(delta_t, 2)/2) * cos(x(3)) * qv,
             (pow(delta_t, 2)/2) * sin(x(3)) * qv,
             delta_t * qv,
             qy * pow(delta_t, 2) / 2,
             qy * delta_t;
        Xsig_pred.col(i) = x.topRows(5) + p + q;
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
    VectorXd x_i (n_x);
    for (int i = 0; i < weights.rows(); ++i) {
        x_i = Xsig_pred.col(i) - x;
        P += weights(i) * x_i * x_i.transpose();
    }
}

void KalmanFilter::StateToMeasurement() {
    z_pred = H * x;
    S = H * P * H.transpose() + R;
}

void KalmanFilter::Update(const Measurement& meas, double delta_t) {
    CalculateSigmaPoints();
    PredictSigmaPoints(delta_t);
    PredictMeanAndCovariance();
    StateToMeasurement();
    z << meas.x, meas.y, meas.yaw;
    K = P * H.transpose() * S.inverse();
    x += K * (z - z_pred);
    x(3) = fmod(x(3), 2*M_PI);
    P *= (I - K * H);    
}


Pose KalmanFilter::getPose() const {
    return {Point(x(0), x(1), 0),
            Rotate(x(3), 0, 0)};
}
