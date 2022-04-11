#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include "localizer.h"

using Vector5d = Eigen::Matrix<double, 5, 1>;


Localizer::Localizer(const po::variables_map& vm, KalmanFilter& kalman, cc::World& world,
                     boost::shared_ptr<cc::BlueprintLibrary> bpl,
                     boost::shared_ptr<cc::Actor> ego, PointCloudT::Ptr mapCloud)
    : kalman(kalman), ego(ego)
    , batch_size(vm["lidar.batch_size"].as<size_t>())
    , min_pnt_dist(vm["lidar.min_pnt_dist"].as<float>())
    , transform(getTransform(pose))
    , cloudCurrent(make_shared<PointCloudT>())
    , cloudFiltered(make_shared<PointCloudT>())
    , cloudAligned(make_shared<PointCloudT>())
{   
    if (vm["general.use_gnss"].as<bool>())
        initGNSS(vm, world, bpl, ego);

    if (vm["general.use_imu"].as<bool>())
        initIMU(vm, world, bpl, ego);
    
    if (vm["general.use_lidar"].as<bool>()) {
        initLidar(vm, world, bpl, ego);
        initNDT(vm, mapCloud);
    }
}

void Localizer::initGNSS(const po::variables_map& vm, cc::World& world,
                         boost::shared_ptr<cc::BlueprintLibrary> bpl,
                         boost::shared_ptr<cc::Actor> ego)
{
    auto bp = *(bpl->Find("sensor.other.gnss"));
    bp.SetAttribute("noise_lat_bias", vm["gnss.lat_bias"].as<string>());
    bp.SetAttribute("noise_lat_stddev", vm["gnss.lat_stddev"].as<string>());
    bp.SetAttribute("noise_lon_bias", vm["gnss.lon_bias"].as<string>());
    bp.SetAttribute("noise_lon_stddev", vm["gnss.lon_stddev"].as<string>());
    bp.SetAttribute("sensor_tick", vm["gnss.sensor_tick"].as<string>());
    auto actor = world.SpawnActor(bp,
                                  cg::Transform(cg::Location(-0.5, 0, 1.8)),
                                  ego.get());
    gnss = boost::static_pointer_cast<cc::Sensor>(actor);
    gnss->Listen([this] (auto data) {
        if (!gnss_meas) {
            auto meas = boost::static_pointer_cast<csd::GnssMeasurement>(data);
            gnss_meas.emplace(meas->GetLatitude(), meas->GetLongitude());
        }
    });    
}

void Localizer::initLidar(const po::variables_map& vm, cc::World& world,
                          boost::shared_ptr<cc::BlueprintLibrary> bpl,
                          boost::shared_ptr<cc::Actor> ego)
{
    auto lidar_bp = *(bpl->Find("sensor.lidar.ray_cast"));
    lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
    lidar_bp.SetAttribute("rotation_frequency", vm["lidar.rot_frq"].as<string>());
    lidar_bp.SetAttribute("points_per_second", vm["lidar.pts_per_sec"].as<string>());
    lidar_bp.SetAttribute("sensor_tick", vm["lidar.sensor_tick"].as<string>());
    auto lidar_actor = world.SpawnActor(lidar_bp,
                                        cg::Transform(cg::Location(-0.5, 0, 1.8)),
                                        ego.get());
    lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
    lidar->Listen([this] (auto data) {
        if (!lidar_meas) {
            auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
            for (auto detection : *scan)
                if (std::hypot(detection.point.x, detection.point.y, detection.point.z) > min_pnt_dist)
                    cloudCurrent->emplace_back(detection.point.x, detection.point.y, detection.point.z);
            
            if (cloudCurrent->size() >= batch_size) {
                ndt.align(*cloudAligned, transform);
                if (ndt.hasConverged()) {
                    Matrix4f new_transform = ndt.getFinalTransformation();
                    lidar_meas.emplace(getX(new_transform), getY(new_transform), getYaw(new_transform));
                } else {
                    std::cout << "NDT didn't converge" << std::endl;
                    lidar_meas.emplace(getX(transform), getY(transform), getYaw(transform));
                }
            }
        }
    });
}

void Localizer::initNDT(const po::variables_map& vm, PointCloudT::Ptr mapCloud)
{
    ndt.setInputTarget(mapCloud);
    ndt.setInputSource(cloudCurrent);
    ndt.setTransformationEpsilon(vm["ndt.trans_eps"].as<float>());
    ndt.setResolution(vm["ndt.resolution"].as<float>());
    ndt.setStepSize(vm["ndt.step_size"].as<float>());
    ndt.setMaximumIterations(vm["ndt.max_iter"].as<size_t>());
}	

void Localizer::initIMU(const po::variables_map& vm, cc::World& world,
                        boost::shared_ptr<cc::BlueprintLibrary> bpl,
                        boost::shared_ptr<cc::Actor> ego)
{
    auto imu_bp = *(bpl->Find("sensor.other.imu"));
    imu_bp.SetAttribute("noise_accel_stddev_x", "0");
    imu_bp.SetAttribute("noise_accel_stddev_y", "0");
    imu_bp.SetAttribute("noise_gyro_stddev_z", "0");
    imu_bp.SetAttribute("sensor_tick", vm["imu.sensor_tick"].as<string>());
    auto imu_actor = world.SpawnActor(imu_bp,
                                        cg::Transform(cg::Location(-0.5, 0, 1.8)),
                                        ego.get());
    imu = boost::static_pointer_cast<cc::Sensor>(imu_actor);
    imu->Listen([this] (auto data) {
        if (!imu_meas) {
            auto meas = boost::static_pointer_cast<csd::IMUMeasurement>(data);
            imu_meas.emplace(max(min(meas->GetAccelerometer().x, max_accel), -max_accel),
                             max(min(meas->GetAccelerometer().y, max_accel), -max_accel),
                             meas->GetGyroscope().z);
        }
    });
}

void Localizer::Localize() {
    if (imu_meas) {
        kalman.Predict(*imu_meas);
        imu_meas.reset();
    }
    if (gnss_meas) {
        kalman.Correct(*gnss_meas);
        gnss_meas.reset();
    }
    if (lidar_meas) {
        kalman.Correct(*lidar_meas);
        lidar_meas.reset();
    }
    // // Get updated KF state
    pose = kalman.getPose();
    transform = getTransform(pose);

    // Transform scan so it aligns with ego's actual pose and render the scan
    pcl::transformPointCloud(*cloudCurrent, *cloudAligned, transform);
    cloudCurrent->clear();
}
