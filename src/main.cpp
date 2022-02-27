#include <string>
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <cmath>
#include <thread>

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

#include "helper.h"
#include "kalman.h"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace std;
namespace csd = carla::sensor::data;

template <class T>
using cptr = carla::SharedPtr<T>;

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;
bool refresh_view = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
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
	bool scan_is_ready;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;
	typename pcl::PointCloud<PointT>::Ptr scanCloud, cloudFiltered;
	boost::shared_ptr<cc::Sensor> lidar;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	pcl::VoxelGrid<PointT> vg;

	void initLidar(cc::World& world, cptr<cc::BlueprintLibrary> bpl, cptr<cc::Actor> ego) {
		auto lidar_bp = *(bpl->Find("sensor.lidar.ray_cast"));
		lidar_bp.SetAttribute("upper_fov", "15");
		lidar_bp.SetAttribute("lower_fov", "-25");
		lidar_bp.SetAttribute("channels", "32");
		lidar_bp.SetAttribute("range", "30");
		lidar_bp.SetAttribute("rotation_frequency", "60");
		lidar_bp.SetAttribute("points_per_second", "500000");		
		auto lidar_actor = world.SpawnActor(lidar_bp,
											cg::Transform(cg::Location(-0.5, 0, 1.8)),
											ego.get());
		lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
		lidar->Listen([this] (auto data) {
			if (!scan_is_ready) {
				auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
				for (auto detection : *scan){
					if (std::hypot(detection.point.x, detection.point.y, detection.point.z) > 10) // Don't include points touching ego
						pclCloud.points.push_back(PointT(detection.point.x, detection.point.y, detection.point.z));
				}
				if (pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions
					lastScanTime = std::chrono::system_clock::now();
					*scanCloud = pclCloud;
					scan_is_ready = true;
				}
			}
		});
	}
	void initNDT(PointCloudT::Ptr mapCloud) {
		ndt.setInputTarget(mapCloud);
		ndt.setInputSource(cloudFiltered);
		ndt.setTransformationEpsilon(1e-3);
		ndt.setResolution(2);
		ndt.setStepSize(1);
		ndt.setMaximumIterations(10);
	}
	void initVG() {
		vg.setInputCloud(scanCloud);
		vg.setLeafSize(1.5, 1.5, 1.5);
	}
			// auto imu_bp = *(bpl->Find("sensor.other.imu"));
public:
	Localizer(cc::World& world, cptr<cc::BlueprintLibrary> bpl,
			  cptr<cc::Actor> ego, PointCloudT::Ptr mapCloud)
		: pose(Point(0,0,0), Rotate(0,0,0))
		, scanCloud(new pcl::PointCloud<PointT>)
		, cloudFiltered(new pcl::PointCloud<PointT>)
		, scan_is_ready(false)
	{
		initLidar(world, bpl, ego);
		initNDT(mapCloud);
		initVG();
	}
	bool ScanIsReady() const {
		return scan_is_ready;
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
		vg.filter(*cloudFiltered);

		// Find pose transform by using NDT matching
		auto [transform, scanAligned] = NDT(pose);
		pose = getPose(transform);

		// Transform scan so it aligns with ego's actual pose and render that scan
		// kalman.Update(pose);
		// pose.position = kalman.getPosition();
		transform = getTransform(pose);
		pcl::transformPointCloud(*cloudFiltered, *scanAligned, transform);
		scan_is_ready = false;
		return scanAligned;
	}
};



int main() {

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
  	pcl::io::loadPCDFile("../map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	Localizer localizer (world, blueprint_library, ego, mapCloud);

	// Init KalmanFilter
	KalmanFilter kalman(1, 3);

	Pose poseRef = getTruePose(vehicle);
	double maxError = 0;
  
	while (!viewer->wasStopped())
  	{
		while (!localizer.ScanIsReady())
			std::this_thread::sleep_for(0.1s);

		if (refresh_view) {
			const Pose& pose = localizer.GetPose();
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = getTruePose(vehicle, poseRef);
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
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
		
		if (localizer.ScanIsReady()) {
			viewer->removePointCloud("scan");
			renderPointCloud(viewer, localizer.Localize(), "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			const Pose& pose = localizer.GetPose();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          
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
