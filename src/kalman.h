#pragma once

#include <Eigen/Dense>
#include "helper.h"

struct Measurement {
    double x, y, z, yaw;
    Measurement() : x(0), y(0), z(0), yaw(0) {}
};

class KalmanFilter {
private:
    Eigen::Matrix<double, 8, 1> state;
    Eigen::Matrix<double, 4, 1> z; // measurement
    Eigen::Matrix<double, 8, 8> P; // estimation error covar.
    Eigen::Matrix<double, 4, 4> R; // measurement noise covar.
    Eigen::Matrix<double, 4, 8> H; // state to measurement transform
    Eigen::Matrix<double, 8, 8> F; // state transition
    Eigen::Matrix<double, 8, 8> Q; // process noise covar.
    Eigen::Matrix<double, 4, 4> S; // residual covar.
    Eigen::Matrix<double, 8, 4> K; // Kalman gain
    Eigen::Matrix<double, 8, 8> I; // identity
    double qx, qy, qz, qh;         // process noise variances
    void setF(double dt);
    void setQ(double dt);
    void Predict(double dt);
public:
    KalmanFilter(double qx, double qy, double qz, double qh);
    void Update(const Measurement& meas, double dt);
    Pose getPose() const;
};