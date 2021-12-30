#include <cmath>
#include "kalman.h"


KalmanFilter::KalmanFilter(float dt, float q) {
    state << Eigen::MatrixXf::Zero(6, 1);

    P << Eigen::MatrixXf::Zero(6, 6);
    P.diagonal() << .1, .1, .1, 2500, 2500, 25;

    R << Eigen::MatrixXf::Zero(3, 3);
    R.diagonal() << .1, .1, .1;

    H << Eigen::MatrixXf::Identity(3, 6);

    F << Eigen::MatrixXf::Identity(6, 6);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    float cub = pow(dt, 3) * q / 3;
    float qua = pow(dt, 2) * q / 2;
    float lin = dt * q;
    Q << cub, 0, 0, qua, 0, 0,
         0, cub, 0, 0, qua, 0,
         0, 0, cub, 0, 0, qua,
         qua, 0, 0, lin, 0, 0,
         0, qua, 0, 0, lin, 0,
         0, 0, qua, 0, 0, lin;

    I << Eigen::MatrixXf::Identity(6, 6);
}

void KalmanFilter::Predict() {
    state = F * state;
    P = F * P * F.transpose() + Q;
}

Eigen::Vector3f KalmanFilter::stateToMeas() const {
    return state.topRows(3); // H * state
}

void KalmanFilter::Update(const Pose& meas) {
    Predict();
    z << meas.position.x, meas.position.y, meas.position.z;
    S = P.topLeftCorner(3, 3) + R; // H * P * H.transpose() + R
    K = P.leftCols(3) * S.inverse(); // P * H.transpose() * S.inverse();
    state += K * (z - stateToMeas());
    P *= (I - K * H);
}

Point KalmanFilter::getPosition() const {
    return Point(state(0), state(1), state(2));
}
