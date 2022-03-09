#pragma once

#include <Eigen/Dense>
#include "helper.h"

using Eigen::Matrix;

struct Measurement {
    double x, y, yaw, w;
    Measurement() : x(0), y(0), yaw(0), w(0) {}
};

class KalmanFilter {
private:
    size_t n_x = 5;
    size_t n_aug = 7;
    size_t n_z = 4;
    double lambda = 3 - n_aug;
    Matrix<double, 5, 1> x;         // state [x, y, v, yaw, w]
    Matrix<double, 7, 1> x_aug;     // augmented state
    Matrix<double, 5, 5> P;         // estimation error covar.
    Matrix<double, 7, 7> P_aug;     // augmented estimation error covar.
    Matrix<double, 7, 7> L;         // square root of P_aug
    Matrix<double, 7, 15> Xsig_aug; // 
    Matrix<double, 5, 15> Xsig_pred;
    Matrix<double, 4, 15> Zsig_pred;
    Matrix<double, 15, 1> weights;
    Matrix<double, 4, 1> z;
    Matrix<double, 4, 1> z_pred;
    Matrix<double, 4, 4> S;         // predicted measurement covar.
    Matrix<double, 4, 4> R;         // measurement noise covar.
    Matrix<double, 4, 5> H;         // state to measurement transform
    Matrix<double, 5, 4> T;         // cross-corr. between sigmas in state vs measurement spaces
    Matrix<double, 5, 4> K;         // Kalman gain
    Matrix<double, 5, 5> I;         // identity
    double var_a, var_wd;        // process noise var.
    double var_x, var_y, var_yaw, var_w;   // measurement noise var.
    void CalculateSigmaPoints();
    void PredictSigmaPoints(double delta_t);
    void PredictMeanAndCovariance();
    void PredictMeasurement();
public:
    KalmanFilter(double var_x, double var_y, double var_yaw, double var_w,
                 double std_a, double std_wd);
    void Update(const Measurement& meas, double delta_t);
    Pose getPose() const;
};