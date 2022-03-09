#pragma once

#include <Eigen/Dense>
#include "helper.h"

using Eigen::Matrix;

struct Measurement {
    double x, y, a, yaw, w;
    Measurement() : x(0), y(0), a(0), yaw(0), w(0) {}
};

class KalmanFilter {
private:
    size_t n_x = 6;
    size_t n_aug = 8;
    size_t n_z = 5;
    double lambda = 3 - n_aug;
    Matrix<double, 6, 1> x;         // state [px, py, v, a, yaw, w]
    Matrix<double, 8, 1> x_aug;     // augmented state (+ [q_j, q_wd])
    Matrix<double, 6, 6> P;         // estimation error covar.
    Matrix<double, 8, 8> P_aug;     // augmented estimation error covar.
    Matrix<double, 8, 8> L;         // square root of P_aug
    Matrix<double, 8, 17> Xsig_aug; // augmented sigma points
    Matrix<double, 6, 17> Xsig_pred;// predicted sigma points
    Matrix<double, 5, 17> Zsig_pred;// predicted measuremenmts
    Matrix<double, 17, 1> weights;  // sigma points weights
    Matrix<double, 5, 1> z;         // input measurement
    Matrix<double, 5, 1> z_pred;    // predicted measurement
    Matrix<double, 5, 5> S;         // predicted measurement covar.
    Matrix<double, 5, 5> R;         // measurement noise covar.
    Matrix<double, 5, 6> H;         // state to measurement transform
    Matrix<double, 6, 5> T;         // cross-corr. between sigmas in state vs measurement spaces
    Matrix<double, 6, 5> K;         // Kalman gain
    Matrix<double, 6, 6> I;         // identity
    double var_j, var_wd;           // process noise var.
    double var_x, var_y, var_a,
                var_yaw, var_w;     // measurement noise var.
    void CalculateSigmaPoints();
    void PredictSigmaPoints(double dt);
    void PredictMeanAndCovariance();
    void PredictMeasurement();
public:
    KalmanFilter(double var_x, double var_y, double var_a, double var_yaw, double var_w,
                 double std_j, double std_wd);
    void Update(const Measurement& meas, double dt);
    Pose getPose() const;
};