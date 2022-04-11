#pragma once

#include <chrono>
#include <Eigen/Dense>
#include "helper.h"

using namespace std::chrono;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;

class KalmanFilter {
private:
    size_t n_x = 5;
    size_t n_sigmas = 1 + 2 * n_x;
    double lambda = 3. - n_x;
    Pose poseRef;
	time_point<system_clock> lastUpdateTime;

    Vector5d x;                      // state [pos.x, pos.y, yaw, vel.x, vel.y]
    Matrix5d P;                      // estimation error covar.
    Matrix5d Q;                      // process noise covar.
    Matrix2d R_gnss;                  // measurement noise covar.
    Matrix3d R_lidar;
    Eigen::Matrix<double, 5, 11> Xsig;      // sigma points
    Eigen::Matrix<double, 11, 1> weights;   // sigma points weights
    
    void CalculateSigmaPoints();
    double GetTimedelta();
    Vector5d ApplyMotion(const Vector5d& state, const IMUMeasurement& meas, double dt);
    void TruncateYaw(double& yaw);
public:
    KalmanFilter(const Matrix2d& R_gnss, const Matrix3d& R_lidar, const Matrix5d& Q,
                 double P_init, const Pose& poseRef);
    void Predict(const IMUMeasurement& meas);
    void Correct(const LidarMeasurement& meas);
    void Correct(const GNSSMeasurement& meas);    
    Pose getPose() const;
};