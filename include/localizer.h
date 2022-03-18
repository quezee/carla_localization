#pragma once

#include <chrono>
#include <boost/program_options.hpp>
#include <carla/client/Sensor.h>
#include <carla/client/BlueprintLibrary.h>
#include <pcl/registration/ndt.h>

#include "helper.h"
#include "kalman.h"

namespace po = boost::program_options;
using namespace std::chrono;

using std::string;
using std::pair;
using std::optional;
using std::make_shared;
using Eigen::Matrix4f;


class Localizer {
private:
	Pose pose;
	bool ndtReady, imuReady;
	size_t batch_size;
	float min_pnt_dist;
	Measurement meas;
	optional<KalmanFilter> kalman;
	time_point<system_clock> lastUpdateTime;
	PointCloudT::Ptr currentCloud, cloudFiltered;
	boost::shared_ptr<cc::Actor> ego;
	boost::shared_ptr<cc::Sensor> lidar, imu;
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;

	void initLidar(cc::World& world, boost::shared_ptr<cc::BlueprintLibrary> bpl,
                   boost::shared_ptr<cc::Actor> ego, const string& rot_frq, const string& pts_per_sec);

	void initNDT(PointCloudT::Ptr mapCloud, float trans_eps,
				 float resolution, float step_size, size_t max_iter);

	void initIMU(cc::World& world, boost::shared_ptr<cc::BlueprintLibrary> bpl,
                 boost::shared_ptr<cc::Actor> ego);

public:
	Localizer(const po::variables_map& vm, cc::World& world,
			  boost::shared_ptr<cc::BlueprintLibrary> bpl,
              boost::shared_ptr<cc::Actor> ego, PointCloudT::Ptr mapCloud);

	pair<Matrix4f, PointCloudT::Ptr> NDT();
	const PointCloudT::Ptr Localize();

	bool MeasurementIsReady() const {
		return ndtReady && imuReady;
	}

	const Pose& GetPose() const {
		return pose;
	}

	~Localizer() {
		cout << "Destroyed:"
			 << "\n -lidar: " << lidar->Destroy()		
			 << "\n -imu: " << imu->Destroy()
			 << "\n -ego: " << ego->Destroy()
			 << endl;
	}
};