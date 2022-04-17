#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include "localizer.h"

using Vector5d = Eigen::Matrix<double, 5, 1>;


Localizer::Localizer(const po::variables_map& vm, KalmanFilter& kalman, cc::World& world,
                     boost::shared_ptr<cc::Actor> ego, PointCloudT::Ptr mapCloud)
    : kalman(kalman), ego(ego)
    , lidar_batch_size(vm["lidar.batch_size"].as<size_t>())
    , lidar_min_pnt_dist(vm["lidar.min_pnt_dist"].as<float>())
    , transform(getTransform(pose))
    , cloudCurrent(make_shared<PointCloudT>())
    , cloudFiltered(make_shared<PointCloudT>())
    , cloudAligned(make_shared<PointCloudT>())
{   
    if (vm["general.use_gnss"].as<bool>())
        initGNSS(vm, world, ego);

    if (vm["general.use_imu"].as<bool>())
        initIMU(vm, world, ego);
    
    if (vm["general.use_lidar"].as<bool>())
        initLidar(vm, world, ego, mapCloud);
}

void Localizer::initGNSS(const po::variables_map& vm, cc::World& world,
                         boost::shared_ptr<cc::Actor> ego)
{
    auto bpl = world.GetBlueprintLibrary();
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
                          boost::shared_ptr<cc::Actor> ego, PointCloudT::Ptr mapCloud)
{
    auto bpl = world.GetBlueprintLibrary();
    auto bp = *(bpl->Find("sensor.lidar.ray_cast"));
    bp.SetAttribute("upper_fov", "15");
    bp.SetAttribute("lower_fov", "-25");
    bp.SetAttribute("channels", "32");
    bp.SetAttribute("range", "30");
    bp.SetAttribute("rotation_frequency", vm["lidar.rot_frq"].as<string>());
    bp.SetAttribute("points_per_second", vm["lidar.pts_per_sec"].as<string>());
    bp.SetAttribute("sensor_tick", vm["lidar.sensor_tick"].as<string>());
    auto actor = world.SpawnActor(bp,
                                  cg::Transform(cg::Location(-0.5, 0, 1.8)),
                                  ego.get());
    lidar = boost::static_pointer_cast<cc::Sensor>(actor);
    lidar->Listen([this] (auto data) {
        if (!lidar_meas) {
            auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
            for (auto detection : *scan)
                if (std::hypot(detection.point.x, detection.point.y, detection.point.z) > lidar_min_pnt_dist)
                    cloudCurrent->emplace_back(detection.point.x, detection.point.y, detection.point.z);
            
            if (cloudCurrent->size() >= lidar_batch_size) {
                // get NDT transform
                transform = getTransform(pose);
                ndt.align(*cloudAligned, transform);
                const Matrix4f& new_transform = ndt.getFinalTransformation();
                lidar_meas.emplace(getX(new_transform), getY(new_transform), getYaw(new_transform));
                // align scan so it aligns with ego's actual pose
                if (ndt.hasConverged())
                    pcl::transformPointCloud(*cloudCurrent, *cloudAligned, new_transform);
                else
                    pcl::transformPointCloud(*cloudCurrent, *cloudAligned, transform);
                cloudCurrent->clear();
            }
        }
    });
    // set NDT params
    ndt.setInputTarget(mapCloud);
    ndt.setInputSource(cloudCurrent);
    ndt.setTransformationEpsilon(vm["ndt.trans_eps"].as<float>());
    ndt.setResolution(vm["ndt.resolution"].as<float>());
    ndt.setStepSize(vm["ndt.step_size"].as<float>());
    ndt.setMaximumIterations(vm["ndt.max_iter"].as<size_t>());
}

void Localizer::initIMU(const po::variables_map& vm, cc::World& world,
                        boost::shared_ptr<cc::Actor> ego)
{
    auto bpl = world.GetBlueprintLibrary();
    auto bp = *(bpl->Find("sensor.other.imu"));
    bp.SetAttribute("noise_accel_stddev_x", "0");
    bp.SetAttribute("noise_accel_stddev_y", "0");
    bp.SetAttribute("noise_gyro_stddev_z", "0");
    bp.SetAttribute("sensor_tick", vm["imu.sensor_tick"].as<string>());
    auto actor = world.SpawnActor(bp,
                                  cg::Transform(cg::Location(-0.5, 0, 1.8)),
                                  ego.get());
    imu = boost::static_pointer_cast<cc::Sensor>(actor);
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
        pose = kalman.getPose();
        imu_meas.reset();
    }
    if (gnss_meas) {
        kalman.Correct(*gnss_meas);
        pose = kalman.getPose();
        gnss_meas.reset();
    }
    if (lidar_meas) {
        if (ndt.hasConverged()) {
            kalman.Correct(*lidar_meas);
            pose = kalman.getPose();
        } else
            std::cout << "NDT didn't converge" << std::endl;
        lidar_meas.reset();
    }
}
