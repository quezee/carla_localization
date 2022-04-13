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
#include <carla/sensor/data/IMUMeasurement.h>

#include "helper.h"

using namespace std;
using namespace chrono;
using namespace chrono_literals;
using namespace string_literals;
namespace csd = carla::sensor::data;

template <class T>
using cptr = carla::SharedPtr<T>;

PointCloudT::Ptr pclCloud (new PointCloudT);
cc::Vehicle::Control control;
vector<ControlState> cs;
bool refresh_view = false;
bool save_map = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}

	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
	if(event.getKeySym() == "s" && event.keyDown()){
		save_map = true;
	}
}

class Mapper {
private:
	Pose truePose, poseRef;
	Eigen::Matrix4f trueTransform;
	bool scanIsReady = false;
	boost::shared_ptr<cc::Vehicle> vehicle;
	boost::shared_ptr<cc::Sensor> lidar;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	int batchSize;
	int batchCounter;

	void initLidar(cc::World& world, cptr<cc::BlueprintLibrary> bpl, cptr<cc::Actor> ego) {
		auto lidar_bp = *(bpl->Find("sensor.lidar.ray_cast"));
		lidar_bp.SetAttribute("upper_fov", "15");
		lidar_bp.SetAttribute("lower_fov", "-25");
		lidar_bp.SetAttribute("channels", "32");
		lidar_bp.SetAttribute("range", "30");
		lidar_bp.SetAttribute("rotation_frequency", "60");
		lidar_bp.SetAttribute("points_per_second", "50000");
		auto lidar_actor = world.SpawnActor(lidar_bp,
											cg::Transform(cg::Location(-0.5, 0, 1.8)),
											ego.get());
		lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
		lidar->Listen([this] (auto data) {
			if (!scanIsReady) {
				truePose = getTruePose(vehicle, poseRef);
				trueTransform = getTransform(truePose);
				auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
				for (auto detection : *scan){
					if (std::hypot(detection.point.x, detection.point.y, detection.point.z) > 8) {
						Eigen::Vector4f local (detection.point.x, detection.point.y, detection.point.z, 1);
						Eigen::Vector4f global (trueTransform * local);
						pclCloud->points.push_back(PointT(global[0], global[1], global[2]));
						batchCounter++;
					}
				}
				if (batchCounter >= batchSize) {
					viewer->removePointCloud("map");
					renderPointCloud(viewer, pclCloud, "map", Color(0,0,1));
					scanIsReady = true;
					batchCounter = 0;
				}
			}
		});
	}
public:
	Mapper(cc::World& world, cptr<cc::BlueprintLibrary> bpl,
		   cptr<cc::Actor> ego, boost::shared_ptr<cc::Vehicle> vehicle,
		   pcl::visualization::PCLVisualizer::Ptr viewer, const Pose& poseRef,
		   int batchSize = 25)
		: vehicle(vehicle)
		, viewer(viewer)
		, poseRef(poseRef)
		, batchSize(batchSize)
	{
		initLidar(world, bpl, ego);
	}
	bool& ScanIsReady() {
		return scanIsReady;
	}
	~Mapper() {
		cout << "Destroyed:"
			 << "\n -ego: " << vehicle->Destroy()
			 << "\n -lidar: " << lidar->Destroy()
			 << endl;
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
	vehicle->SetAutopilot();

	Pose poseRef = getTruePose(vehicle);
	Mapper mapper (world, blueprint_library, ego, vehicle, viewer, poseRef);

	while (!viewer->wasStopped () && !save_map)
  	{
		while (!mapper.ScanIsReady())
			std::this_thread::sleep_for(0.1s);

		Pose truePose = getTruePose(vehicle, poseRef);

		if (refresh_view) {
			viewer->setCameraPosition(truePose.position.x, truePose.position.y, 60,
									  truePose.position.x+1, truePose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeAllShapes();
		drawCar(truePose, 0, Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));

		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}
		// skip traffic light
		auto tl = vehicle->GetTrafficLight();
		if (tl)
			tl->SetState(carla::rpc::TrafficLightState::Green);

  		viewer->spinOnce ();
		mapper.ScanIsReady() = false;
	}
    pclCloud->width = pclCloud->size();
    pclCloud->height = 1;
	pcl::io::savePCDFileASCII ("data/map1.pcd", *pclCloud);
	cout << "saved pcd map of size: " << pclCloud->size() << endl;
}
