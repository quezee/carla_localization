#pragma once

#include <Eigen/Dense>
#include "helper.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct Measurement {
    double x, y, a, yaw, w;
    Measurement() : x(0), y(0), a(0), yaw(0), w(0) {}
};

class KalmanFilter {
private:
    size_t n_x, n_aug, n_z, n_sigmas;
    double lambda;
    VectorXd x;            // state [px, py, v, a, yaw, w]
    VectorXd z;            // input measurement [px, py, a, yaw, w]
    VectorXd x_aug;        // augmented state (+ [q_j, q_wd])
    MatrixXd P;            // estimation error covar.
    MatrixXd P_aug;        // augmented estimation error covar.
    MatrixXd L;            // square root of P_aug
    MatrixXd Xsig_aug;     // augmented sigma points
    MatrixXd Xsig_pred;    // predicted sigma points
    MatrixXd Zsig_pred;    // predicted measuremenmts
    VectorXd weights;      // sigma points weights
    VectorXd z_pred;       // predicted measurement
    MatrixXd S;            // predicted measurement covar.
    MatrixXd R;            // measurement noise covar.
    MatrixXd H;            // state to measurement transform
    MatrixXd T;            // cross-corr. between sigmas in state vs measurement spaces
    MatrixXd K;            // Kalman gain
    MatrixXd I;            // identity
    double var_j, var_wd;  // process noise var.
    void CalculateSigmaPoints();
    void PredictSigmaPoints(double dt);
    void PredictMeanAndCovariance();
    void PredictMeasurement();
    // void RegisterMeasurement(const Measurement& meas);
public:
    KalmanFilter(const VectorXd& meas_noise, double std_j, double std_wd,
                 size_t n_x = 6, size_t n_aug = 8, size_t n_z = 5);
    void Update(const Measurement& meas, double dt);
    Pose getPose() const;
};