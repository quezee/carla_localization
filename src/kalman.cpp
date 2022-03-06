#include <cmath>
#include "kalman.h"
using namespace Eigen;

KalmanFilter::KalmanFilter(double q)
    : q(q)
{
    state << MatrixXd::Zero(8, 1);

    P << MatrixXd::Identity(8, 8);
    P *= 100;

    R << MatrixXd::Zero(6, 6);
    R(0, 0) = 0.1;
    R(2, 2) = 0.1;
    R(4, 4) = 0.1;

    H << MatrixXd::Zero(6, 8);
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 3) = 1;
    H(3, 5) = 1;
    H(4, 6) = 1;
    H(5, 7) = 1;

    I << MatrixXd::Identity(8, 8);
}

void KalmanFilter::setF(double dt) {
    F << MatrixXd::Identity(8, 8);
    F(0, 1) = dt;
    F(0, 2) = pow(dt, 2) / 2;
    F(1, 2) = dt;
    F(3, 4) = dt;
    F(3, 5) = pow(dt, 2) / 2;
    F(4, 5) = dt;
    F(6, 7) = dt;
}

void KalmanFilter::setQ(double dt) {
    Q << MatrixXd::Zero(8, 8);
}

void KalmanFilter::Predict(double dt) {
    setF(dt);
    setQ(dt);
    state = F * state;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::Update(const Measurement& meas, double dt) {
    Predict(dt);
    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    z << meas.x, meas.ax, meas.y, meas.ay, meas.yaw, meas.v_yaw;
    cout << z << endl << endl;
    state += K * (z - H * state);
    state(6) = fmod(state(6), 2*M_PI);
    P *= (I - K * H);
}

Pose KalmanFilter::getPose() const {
    return {Point(state(0), state(3), 0),
            Rotate(state(6), 0, 0)};
}
