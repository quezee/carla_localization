#pragma once

#include <boost/program_options.hpp>
#include <carla/client/Sensor.h>
#include <carla/client/BlueprintLibrary.h>
#include <pcl/registration/ndt.h>

#include "helper.h"
#include "kalman.h"

namespace cs = carla::sensor;
namespace csd = carla::sensor::data;
namespace po = boost::program_options;

using std::string;
using std::pair;
using std::optional;
using std::make_shared;
using Eigen::Matrix4f;


class Localizer {
private:
	Pose pose;
	Matrix4f transform;
	size_t batch_size;
	float min_pnt_dist;
	KalmanFilter kalman;
	PointCloudT::Ptr cloudCurrent, cloudFiltered, cloudAligned;
	boost::shared_ptr<cc::Actor> ego;
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	float max_accel = 10;
	// sensors
	boost::shared_ptr<cc::Sensor> gnss, lidar, imu;
	optional<IMUMeasurement> imu_meas;
	optional<GNSSMeasurement> gnss_meas;
	optional<LidarMeasurement> lidar_meas;


	void initGNSS(const po::variables_map& vm, cc::World& world,
				  boost::shared_ptr<cc::BlueprintLibrary> bpl,
				  boost::shared_ptr<cc::Actor> ego);

	void initLidar(const po::variables_map& vm, cc::World& world,
				   boost::shared_ptr<cc::BlueprintLibrary> bpl,
                   boost::shared_ptr<cc::Actor> ego);

	void initNDT(const po::variables_map& vm, PointCloudT::Ptr mapCloud);

	void initIMU(const po::variables_map& vm, cc::World& world,
				 boost::shared_ptr<cc::BlueprintLibrary> bpl,
                 boost::shared_ptr<cc::Actor> ego);

public:
	Localizer(const po::variables_map& vm, KalmanFilter& kalman, cc::World& world,
			  boost::shared_ptr<cc::BlueprintLibrary> bpl,
              boost::shared_ptr<cc::Actor> ego, PointCloudT::Ptr mapCloud);

	void Localize();

	bool MeasurementIsReady() const {
		return imu_meas || gnss_meas || lidar_meas;
	}

	const Pose& GetPose() const {
		return pose;
	}

	const PointCloudT::Ptr GetCloudAligned() const {
		return cloudAligned;
	}

	~Localizer() {
		cout << "Destroyed:"
			 << "\n -ego: " << ego->Destroy();
		if (gnss)  cout << "\n -gnss: "  << gnss->Destroy();
		if (imu)   cout << "\n -imu: "   << imu->Destroy();
		if (lidar) cout << "\n -lidar: " << lidar->Destroy();
		cout << endl;
	}
};