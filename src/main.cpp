#include <thread>
#include <carla/client/Client.h>
#include "localizer.h"
#include <carla/client/Map.h>

namespace pv = pcl::visualization;
using namespace Eigen;
using Matrix5d = Matrix<double, 5, 5>;

cc::Vehicle::Control control;
time_point<system_clock> currentTime;
vector<ControlState> controls;
bool refresh_view = false;


po::variables_map parse_config(int argc, char *argv[]) {
	po::options_description desc("Configuration");
	desc.add_options()
		("general.autopilot", po::value<bool>()->required(), "whether to use autopilot for ego")
		("general.speed_decr", po::value<float>()->required(), "decrease in velocity with respect to speed limit")
		("general.map", po::value<string>()->required(), "path to pcl map")
		
		("general.use_gnss", po::value<bool>()->required())
		("general.use_imu", po::value<bool>()->required())
		("general.use_lidar", po::value<bool>()->required())

		("imu.sensor_tick", po::value<string>()->required())

		("gnss.var_lat", po::value<double>()->required())
		("gnss.var_lon", po::value<double>()->required())
		("gnss.lat_bias", po::value<string>()->required())
		("gnss.lat_stddev", po::value<string>()->required())
		("gnss.lon_bias", po::value<string>()->required())
		("gnss.lon_stddev", po::value<string>()->required())
		("gnss.sensor_tick", po::value<string>()->required())

		("lidar.var_x", po::value<double>()->required())
		("lidar.var_y", po::value<double>()->required())
		("lidar.var_yaw", po::value<double>()->required())
		("lidar.batch_size", po::value<size_t>()->required(), "how many points to accumulate to match scans")
		("lidar.rot_frq", po::value<string>()->required(), "rotation frequency")
		("lidar.pts_per_sec", po::value<string>()->required(), "points per second")
		("lidar.min_pnt_dist", po::value<float>()->required(), "filter out closest points within this radius")
		("lidar.sensor_tick", po::value<string>()->required())

		("ndt.trans_eps", po::value<float>()->required(), "transformation epsilon")
		("ndt.resolution", po::value<float>()->required(), "voxel grid resolution")
		("ndt.step_size", po::value<float>()->required(), "newton line search max step")
		("ndt.max_iter", po::value<size_t>()->required(), "newton max iterations")

		("kalman.P_init", po::value<double>()->required())
		("kalman.var_jerk", po::value<double>()->required())
		("kalman.var_yaw_a", po::value<double>()->required())
	;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	string cfg_fname {"config.cfg"}; 
	ifstream ifs (cfg_fname);
	if (ifs.fail())
		throw std::invalid_argument("Failed to open config file: " + cfg_fname);

	po::store(po::parse_config_file(ifs, desc), vm);
	po::notify(vm);

	return vm;
}

void keyboardEventOccurred(const pv::KeyboardEvent &event, void* viewer)
{
	if (event.getKeySym() == "Right" && event.keyDown())
		controls.emplace_back(0, -0.02, 0);
	else if (event.getKeySym() == "Left" && event.keyDown())
		controls.emplace_back(0, 0.02, 0); 

  	if (event.getKeySym() == "Up" && event.keyDown())
		controls.emplace_back(0.1, 0, 0);
	else if (event.getKeySym() == "Down" && event.keyDown())
		controls.emplace_back(-0.1, 0, 0); 

	if (event.getKeySym() == "a" && event.keyDown())
		refresh_view = true;
}


int main(int argc, char *argv[]) {

	po::variables_map vm {parse_config(argc, argv)};

	// set world and ego
	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(10s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto sp_transform = map->GetRecommendedSpawnPoints()[1];
	auto ego = world.SpawnActor((*vehicles)[12], sp_transform);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego);
	if (vm["general.autopilot"].as<bool>()) {
		vehicle->SetAutopilot();
		auto tm = client.GetInstanceTM();
		tm.SetPercentageSpeedDifference(ego, vm["general.speed_decr"].as<float>());
	}
	// set viewer
	pv::PCLVisualizer::Ptr viewer = make_shared<pv::PCLVisualizer>("3D Viewer");
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);	

	// load map
	PointCloudT::Ptr mapCloud = make_shared<PointCloudT>();
  	pcl::io::loadPCDFile(vm["general.map"].as<string>(), *mapCloud);
  	cout << "Loaded " << mapCloud->size() << " data points from map" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	// set localizer
	Pose poseRef = getTruePose(vehicle);
	KalmanFilter kalman (vm, poseRef);
	Localizer localizer (vm, kalman, world, blueprint_library, ego, mapCloud);

	// main loop
	double maxError = 0;
	double meanError = 0;
	size_t n_steps = 0;
  
	while (!viewer->wasStopped())
  	{
		if (refresh_view) {
			const Pose& pose = localizer.GetPose();
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60,
									  pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = getTruePose(vehicle, poseRef);
		drawCar(truePose, 0, Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),
				  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z),
				  "steer", Color(0,1,0));

		ControlState accuate(0, 0, 1);
		if(controls.size() > 0){
			accuate = controls.back();
			controls.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}
		// skip traffic light
		auto tl = vehicle->GetTrafficLight();
		if (tl)
			tl->SetState(carla::rpc::TrafficLightState::Green);

  		viewer->spinOnce ();
		
		if (localizer.MeasurementIsReady()) {
			localizer.Localize();

			viewer->removePointCloud("scan");
			renderPointCloud(viewer, localizer.GetCloudAligned(), "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			const Pose& pose = localizer.GetPose();
			drawCar(pose, 1, Color(0,1,0), 0.35, viewer);
          
		  	double poseError = std::hypot(truePose.position.x - pose.position.x,
										  truePose.position.y - pose.position.y);
			n_steps++;
			maxError = max(maxError, poseError);
			meanError = meanError + ((poseError - meanError) / n_steps);
			double distDriven = sqrt( pow(truePose.position.x, 2) + pow(truePose.position.y, 2) );

			viewer->removeShape("meanE");
			viewer->addText("Mean Error: "+to_string(meanError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "meanE",0);
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 250, 32, 1.0, 1.0, 1.0, "dist",0);
		}
	}
}
