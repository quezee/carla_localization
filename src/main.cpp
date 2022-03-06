#include <string>
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <cmath>
#include <thread>
#include <boost/program_options.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>

#include <carla/client/Vehicle.h>
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>

#include "helper.h"
#include "kalman.h"

using namespace std;
using namespace chrono;
using namespace chrono_literals;
using namespace string_literals;
namespace po = boost::program_options;
namespace csd = carla::sensor::data;

template <class T>
using cptr = carla::SharedPtr<T>;

PointCloudT pclCloud;
cc::Vehicle::Control control;
time_point<system_clock> currentTime;
vector<ControlState> cs;
bool refresh_view = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
	if (event.getKeySym() == "Right" && event.keyDown())
		cs.push_back(ControlState(0, -0.02, 0));
	else if (event.getKeySym() == "Left" && event.keyDown())
		cs.push_back(ControlState(0, 0.02, 0)); 

  	if (event.getKeySym() == "Up" && event.keyDown())
		cs.push_back(ControlState(0.1, 0, 0));
	else if (event.getKeySym() == "Down" && event.keyDown())
		cs.push_back(ControlState(-0.1, 0, 0)); 

	if (event.getKeySym() == "a" && event.keyDown())
		refresh_view = true;
}

class Localizer {
private:
	Pose pose;
	bool ndtReady, imuReady;
	time_point<system_clock> lastUpdateTime;
	PointCloudT::Ptr currentCloud, cloudFiltered;
	cptr<cc::Actor> ego;
	boost::shared_ptr<cc::Sensor> lidar, imu;
	size_t batch_size;
	float min_pnt_dist;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	Measurement meas;
	KalmanFilter kalman;

	// void initGPS

	void initLidar(cc::World& world, cptr<cc::BlueprintLibrary> bpl, cptr<cc::Actor> ego,
				   const string& rot_frq, const string& pts_per_sec) {
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
				for (auto detection : *scan) {
					if (std::sqrt(std::hypot(detection.point.x, detection.point.y, detection.point.z)) > min_pnt_dist)
						pclCloud.points.push_back(PointT(detection.point.x, detection.point.y, detection.point.z));
				}
				if (pclCloud.size() >= batch_size) {
					// cout << scanCloud->size() << endl;
					*currentCloud = pclCloud;
					ndtReady = true;
				}
			}
		});
	}
	void initNDT(PointCloudT::Ptr mapCloud, float trans_eps,
				 float resolution, float step_size, size_t max_iter) {
		ndt.setInputTarget(mapCloud);
		ndt.setInputSource(currentCloud);
		ndt.setTransformationEpsilon(trans_eps);
		ndt.setResolution(resolution);
		ndt.setStepSize(step_size);
		ndt.setMaximumIterations(max_iter);
	}	
	void initIMU(cc::World& world, cptr<cc::BlueprintLibrary> bpl, cptr<cc::Actor> ego) {
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
				meas.ax = accel.x;
				meas.ay = accel.y;
				meas.v_yaw = -imu_meas->GetGyroscope().z;
				imuReady = true;
			}
		});
	}
public:
	Localizer(const po::variables_map& vm, cc::World& world,
			  cptr<cc::BlueprintLibrary> bpl, cptr<cc::Actor> ego,
			  PointCloudT::Ptr mapCloud)
		: ego(ego)
		, batch_size(vm["lidar.batch_size"].as<size_t>())
		, min_pnt_dist(vm["lidar.min_pnt_dist"].as<float>())
		, currentCloud(new PointCloudT)
		, cloudFiltered(new PointCloudT)
		, ndtReady(false), imuReady(false)
		, lastUpdateTime(system_clock::now())
		, kalman(0)
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
	}
	bool MeasurementIsReady() const {
		return ndtReady && imuReady;
	}
	const Pose& GetPose() const {
		return pose;
	}
	pair<Eigen::Matrix4f, PointCloudT::Ptr> NDT(const Pose& startingPose) {
		Eigen::Matrix4f startingTrans = getTransform(startingPose);

		PointCloudT::Ptr aligned (new PointCloudT);
		ndt.align(*aligned, startingTrans);

		if (!ndt.hasConverged()) {
			std::cout << "didn't converge" << std::endl;
			return {Eigen::Matrix4f::Identity(), aligned};
		}
		Eigen::Matrix4f transform = ndt.getFinalTransformation();
		return {transform, aligned};
	}
	const std::shared_ptr<PointCloudT> Localize() {
		// Filter scan using voxel filter
		// cout << "Filtered cloud size: " << cloudFiltered->size() << endl;
		// cout << "Scan cloud size: " << scanCloud->size() << endl;

		// Find pose transform by using NDT matching
		auto [transform, scanAligned] = NDT(pose);
		pose = getPose(transform);

		// Fulfill measurement
		// meas.x = pose.position.x;
		// meas.y = pose.position.y;
		// meas.yaw = pose.rotation.yaw;

		// Update timedelta
		// auto dt = duration_cast<milliseconds>(
		// 	system_clock::now() - lastUpdateTime
		// );
		// double dt_sec = (double)dt.count() / 1000;

		// Get updated KF state
		// kalman.Update(meas, dt_sec);
		// pose = kalman.getPose();

		// Transform scan so it aligns with ego's actual pose and render the scan
		transform = getTransform(pose);
		pcl::transformPointCloud(*currentCloud, *scanAligned, transform);
		ndtReady = false;
		imuReady = false;
		lastUpdateTime = system_clock::now();
		return scanAligned;
	}
	~Localizer() {
		cout << "Destroyed:"
			 << "\n -ego: " << ego->Destroy()
			 << "\n -imu: " << imu->Destroy()
			 << "\n -lidar: " << lidar->Destroy()
			 << endl;
	}
};

po::variables_map parse_config(int argc, char *argv[]) {
	po::options_description desc("Configuration");
	desc.add_options()
		("config", po::value<string>()->default_value("config.cfg"), "path to config")
		("data.map", po::value<string>()->required(), "path to pcl map")

		("lidar.batch_size", po::value<size_t>()->required(), "how many points to accumulate to match scans")
		("lidar.rot_frq", po::value<string>()->required(), "rotation frequency")
		("lidar.pts_per_sec", po::value<string>()->required(), "points per second")
		("lidar.min_pnt_dist", po::value<float>()->required(), "filter out closest points within this radius")

		("ndt.trans_eps", po::value<float>()->required(), "transformation epsilon")
		("ndt.resolution", po::value<float>()->required(), "voxel grid resolution")
		("ndt.step_size", po::value<float>()->required(), "newton line search max step")
		("ndt.max_iter", po::value<size_t>()->required(), "newton max iterations")
	;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	string cfg_fname {vm["config"].as<string>()}; 
	ifstream ifs (cfg_fname);
	if (ifs.fail())
		throw invalid_argument("Failed to open config file: " + cfg_fname);

	po::store(po::parse_config_file(ifs, desc), vm);
	po::notify(vm);

	return vm;
}

int main(int argc, char *argv[]) {

	po::variables_map vm {parse_config(argc, argv)};
	
	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(10s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto sp_transform = map->GetRecommendedSpawnPoints()[1];
	auto ego = world.SpawnActor((*vehicles)[12], sp_transform);

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego);

	// Load map
	PointCloudT::Ptr mapCloud (new PointCloudT);
  	pcl::io::loadPCDFile(vm["data.map"].as<string>(), *mapCloud);
  	cout << "Loaded " << mapCloud->size() << " data points from map" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	Localizer localizer (vm, world, blueprint_library, ego, mapCloud);

	Pose poseRef = getTruePose(vehicle);
	double maxError = 0;
  
	while (!viewer->wasStopped())
  	{
		while (!localizer.MeasurementIsReady())
			std::this_thread::sleep_for(0.1s);

		if (refresh_view) {
			const Pose& pose = localizer.GetPose();
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = getTruePose(vehicle, poseRef);
		drawCar(truePose, 0, Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));

		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

  		viewer->spinOnce ();
		
		if (localizer.MeasurementIsReady()) {
			viewer->removePointCloud("scan");
			renderPointCloud(viewer, localizer.Localize(), "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			const Pose& pose = localizer.GetPose();
			drawCar(pose, 1, Color(0,1,0), 0.35, viewer);
          
		  	double poseError = std::hypot(truePose.position.x - pose.position.x,
										  truePose.position.y - pose.position.y);
			maxError = max(maxError, poseError);
			double distDriven = sqrt( pow(truePose.position.x, 2) + pow(truePose.position.y, 2) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if (maxError > 1.2 || distDriven >= 170.0)
				viewer->removeShape("eval");
			if (maxError > 1.2)
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
			else
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
		}
		pclCloud.points.clear();
	}
}
