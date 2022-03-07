#include <cmath>
#include "kalman.h"
using namespace Eigen;

KalmanFilter::KalmanFilter(double qx, double qy, double qz, double qh)
    : qx(qx), qy(qy), qz(qz), qh(qh)
{
    state.setZero();

    P.setIdentity();
    P(1, 1) = 1000;
    P(3, 3) = 1000;
    P(5, 5) = 1000;
    P(7, 7) = 1000;

    R.setIdentity();
    // R *= 0.1;

    H.setZero();
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 4) = 1;
    H(3, 6) = 1;

    I.setIdentity();
}

void KalmanFilter::setF(double dt) {
    F.setIdentity();
    F(0, 1) = dt;
    F(2, 3) = dt;
    F(4, 5) = dt;
    F(6, 7) = dt;
}

void KalmanFilter::setQ(double dt) {
    Q.setZero();
    Q(0, 0) = qx * pow(dt, 3) / 3;
    Q(0, 1) = qx * pow(dt, 2) / 2;
    Q(1, 0) = qx * pow(dt, 2) / 2;
    Q(1, 1) = qx * dt;
    Q(2, 2) = qy * pow(dt, 3) / 3;
    Q(2, 3) = qy * pow(dt, 2) / 2;
    Q(3, 2) = qy * pow(dt, 2) / 2;
    Q(3, 3) = qy * dt;
    Q(4, 4) = qz * pow(dt, 3) / 3;
    Q(4, 5) = qz * pow(dt, 2) / 2;
    Q(5, 4) = qz * pow(dt, 2) / 2;
    Q(5, 5) = qz * dt;
    Q(6, 6) = qh * pow(dt, 3) / 3;
    Q(6, 7) = qh * pow(dt, 2) / 2;
    Q(7, 6) = qh * pow(dt, 2) / 2;
    Q(7, 7) = qh * dt;
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
    z << meas.x, meas.y, meas.z, meas.yaw;
    state += K * (z - H * state);
    state(6) = fmod(state(6), 2*M_PI);
    P *= (I - K * H);
}

Pose KalmanFilter::getPose() const {
    return {Point(state(0), state(2), state(4)),
            Rotate(state(6), 0, 0)};
}
