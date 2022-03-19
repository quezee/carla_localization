#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include "localizer.h"

namespace po = boost::program_options;
namespace csd = carla::sensor::data;


Localizer::Localizer(const po::variables_map& vm, cc::World& world,
                     boost::shared_ptr<cc::BlueprintLibrary> bpl,
                     boost::shared_ptr<cc::Actor> ego, PointCloudT::Ptr mapCloud)
    : ego(ego)
    , batch_size(vm["lidar.batch_size"].as<size_t>())
    , min_pnt_dist(vm["lidar.min_pnt_dist"].as<float>())
    , currentCloud(make_shared<PointCloudT>())
    , cloudFiltered(make_shared<PointCloudT>())
    , ndtReady(false), imuReady(false)
    , lastUpdateTime(system_clock::now())
{
    initLidar(world, bpl, ego,
                vm["lidar.rot_frq"].as<string>(),
                vm["lidar.pts_per_sec"].as<string>());
    initNDT(mapCloud,
            vm["ndt.trans_eps"].as<float>(),
            vm["ndt.resolution"].as<float>(),
            vm["ndt.step_size"].as<float>(),
            vm["ndt.max_iter"].as<size_t>());
    initIMU(world, bpl, ego);
    if (vm["kalman.use"].as<bool>()) {
        VectorXd meas_noise (5);
        meas_noise << vm["kalman.var_x"].as<double>(), vm["kalman.var_y"].as<double>(),
                        vm["kalman.var_a"].as<double>(), vm["kalman.var_yaw"].as<double>(),
                        vm["kalman.var_w"].as<double>();
        kalman = KalmanFilter(meas_noise,
                                vm["kalman.std_j"].as<double>(),
                                vm["kalman.std_wd"].as<double>());
    }
}

void Localizer::initLidar(cc::World& world, boost::shared_ptr<cc::BlueprintLibrary> bpl,
                          boost::shared_ptr<cc::Actor> ego, const string& rot_frq,
                          const string& pts_per_sec)
{
    auto lidar_bp = *(bpl->Find("sensor.lidar.ray_cast"));
    lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
    lidar_bp.SetAttribute("rotation_frequency", rot_frq);
    lidar_bp.SetAttribute("points_per_second", pts_per_sec);
    auto lidar_actor = world.SpawnActor(lidar_bp,
                                        cg::Transform(cg::Location(-0.5, 0, 1.8)),
                                        ego.get());
    lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
    lidar->Listen([this] (auto data) {
        if (!ndtReady) {
            auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
            for (auto detection : *scan)
                if (std::hypot(detection.point.x, detection.point.y, detection.point.z) > min_pnt_dist)
                    currentCloud->emplace_back(detection.point.x, detection.point.y, detection.point.z);
            if (currentCloud->size() >= batch_size)
                ndtReady = true;
        }
    });
}

void Localizer::initNDT(PointCloudT::Ptr mapCloud, float trans_eps,
				        float resolution, float step_size, size_t max_iter)
{
    ndt.setInputTarget(mapCloud);
    ndt.setInputSource(currentCloud);
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setResolution(resolution);
    ndt.setStepSize(step_size);
    ndt.setMaximumIterations(max_iter);
}	

void Localizer::initIMU(cc::World& world, boost::shared_ptr<cc::BlueprintLibrary> bpl,
                        boost::shared_ptr<cc::Actor> ego)
{
    auto imu_bp = *(bpl->Find("sensor.other.imu"));
    imu_bp.SetAttribute("noise_accel_stddev_x", "0");
    imu_bp.SetAttribute("noise_accel_stddev_y", "0");
    imu_bp.SetAttribute("noise_gyro_stddev_z", "0");
    auto imu_actor = world.SpawnActor(imu_bp,
                                        cg::Transform(cg::Location(-0.5, 0, 1.8)),
                                        ego.get());
    imu = boost::static_pointer_cast<cc::Sensor>(imu_actor);
    imu->Listen([this] (auto data) {
        if (!imuReady) {
            auto imu_meas = boost::static_pointer_cast<csd::IMUMeasurement>(data);
            cg::Vector3D accel = imu_meas->GetAccelerometer();
            meas.a = accel.x;
            meas.w = imu_meas->GetGyroscope().z;
            // cout << accel.x << ' ' << accel.y << ' ' << accel.z << ' ' << meas.w << endl;
            imuReady = true;
        }
    });
}

pair<Matrix4f, PointCloudT::Ptr> Localizer::NDT() {
    Matrix4f startingTrans = getTransform(pose);

    PointCloudT::Ptr aligned (make_shared<PointCloudT>());
    ndt.align(*aligned, startingTrans);

    if (!ndt.hasConverged()) {
        std::cout << "didn't converge" << std::endl;
        return {Matrix4f::Identity(), aligned};
    }
    Matrix4f transform = ndt.getFinalTransformation();
    return {transform, aligned};
}

const PointCloudT::Ptr Localizer::Localize() {
    // Find pose transform by using NDT matching
    auto [transform, scanAligned] = NDT();
    pose = getPose(transform);

    if (kalman.has_value()) {
        // Fulfill measurement
        meas.x = pose.position.x;
        meas.y = pose.position.y;
        meas.yaw = pose.rotation.yaw;

        // Update timedelta
        auto dt = duration_cast<milliseconds>(
            system_clock::now() - lastUpdateTime
        );
        double dt_sec = (double)dt.count() / 1000;

        // Get updated KF state
        kalman.value().Update(meas, dt_sec);
        pose = kalman.value().getPose();
        transform = getTransform(pose);
    }
    // Transform scan so it aligns with ego's actual pose and render the scan
    pcl::transformPointCloud(*currentCloud, *scanAligned, transform);
    currentCloud->clear();
    ndtReady = false;
    imuReady = false;
    lastUpdateTime = system_clock::now();
    return scanAligned;
}
