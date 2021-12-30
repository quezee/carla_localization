#pragma once

// #include <Eigen/Geometry>
#include <Eigen/Dense>
#include "helper.h"

class KalmanFilter {
private:
    Eigen::Matrix<float, 6, 1> state;
    Eigen::Matrix<float, 6, 6> P; // estimation error covar.
    Eigen::Matrix<float, 3, 3> R; // measurement noise covar.
    Eigen::Matrix<float, 3, 6> H; // state to measurement transform
    Eigen::Matrix<float, 6, 6> F; // state transition
    Eigen::Matrix<float, 6, 6> Q; // process noise covar.
    Eigen::Matrix<float, 6, 6> I;
    void Predict();
    Eigen::Vector3f stateToMeas() const;
public:
    KalmanFilter(float dt, float q);
    void Update(const Pose& meas);
    Point getPosition() const;
};