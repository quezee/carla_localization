#pragma once

#include <chrono>
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include "helper.h"

namespace po = boost::program_options;
using namespace std::chrono;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

class KalmanFilter {
private:
    size_t n_x = 5;
    size_t n_a = 7;
    size_t n_sigmas = 1 + 2 * n_a;
    double lambda = 3. - n_a;
    Pose poseRef;
	time_point<system_clock> lastUpdateTime;
    double var_jerk, var_yaw_a;            // process noise covar.

    Vector5d x;                            // state [pos.x, pos.y, yaw, vel.x, vel.y]
    Vector7d x_a;                          // augmented state (state + [jerk, yaw_a])
    Matrix5d P;                            // estimation error covar.
    Matrix7d P_a;                          // augmented estimation error covar.
    Matrix2d R_gnss;                       // measurement noise covar. for GNSS
    Matrix3d R_lidar;                      // measurement noise covar. for lidar
    Eigen::Matrix<double, 7, 15> Xsig;     // sigma points
    Eigen::Matrix<double, 15, 1> weights;  // sigma points weights
    
    void CalculateSigmaPoints();
    double GetTimedelta();
    Vector5d ApplyMotion(const Vector7d& state, const IMUMeasurement& meas, double dt);
    void TruncateYaw(double& yaw);
    void CorrectInner(const MatrixXd& Zsig, const VectorXd& z_pred,
                      const MatrixXd& R, const VectorXd& z);    
public:
    KalmanFilter(const po::variables_map& vm, const Pose& poseRef);
    void Predict(const IMUMeasurement& meas);
    void Correct(const LidarMeasurement& meas);
    void Correct(const GNSSMeasurement& meas);    
    Pose getPose() const;
};