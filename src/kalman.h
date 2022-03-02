#pragma once

#include <Eigen/Dense>
#include "helper.h"

struct Measurement {
    double x, ax, y, ay, yaw, v_yaw;
    Measurement()
        : x(0), ax(0), y(0), ay(0),
          yaw(0), v_yaw(0) {}
};

class KalmanFilter {
private:
    Eigen::Matrix<double, 8, 1> state;
    Eigen::Matrix<double, 6, 1> z; // measurement
    Eigen::Matrix<double, 8, 8> P; // estimation error covar.
    Eigen::Matrix<double, 6, 6> R; // measurement noise covar.
    Eigen::Matrix<double, 6, 8> H; // state to measurement transform
    Eigen::Matrix<double, 8, 8> F; // state transition
    Eigen::Matrix<double, 8, 8> Q; // process noise covar.
    Eigen::Matrix<double, 6, 6> S; // residual covar.
    Eigen::Matrix<double, 8, 6> K; // Kalman gain
    Eigen::Matrix<double, 8, 8> I; // identity
    double q;                      // process noise var.
    void setF(double dt);
    void setQ(double dt);
    void Predict();
public:
    KalmanFilter(double q);
    void Update(const Measurement& meas, double dt);
    Pose getPose() const;
};