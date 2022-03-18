#include <thread>
#include <carla/client/Client.h>
#include "localizer.h"

namespace pv = pcl::visualization;

cc::Vehicle::Control control;
time_point<system_clock> currentTime;
vector<ControlState> cs;
bool refresh_view = false;


po::variables_map parse_config(int argc, char *argv[]) {
	po::options_description desc("Configuration");
	desc.add_options()
		("config", po::value<string>()->default_value("config.cfg"), "path to config")
		("general.autopilot", po::value<bool>()->required(), "whether to use autopilot for ego")
		("general.map", po::value<string>()->required(), "path to pcl map")

		("lidar.batch_size", po::value<size_t>()->required(), "how many points to accumulate to match scans")
		("lidar.rot_frq", po::value<string>()->required(), "rotation frequency")
		("lidar.pts_per_sec", po::value<string>()->required(), "points per second")
		("lidar.min_pnt_dist", po::value<float>()->required(), "filter out closest points within this radius")

		("ndt.trans_eps", po::value<float>()->required(), "transformation epsilon")
		("ndt.resolution", po::value<float>()->required(), "voxel grid resolution")
		("ndt.step_size", po::value<float>()->required(), "newton line search max step")
		("ndt.max_iter", po::value<size_t>()->required(), "newton max iterations")

		("kalman.use", po::value<bool>()->required(), "whether to use kalman filter")
		("kalman.var_x", po::value<double>()->required())
		("kalman.var_y", po::value<double>()->required())
		("kalman.var_v", po::value<double>()->required())
		("kalman.var_a", po::value<double>()->required())
		("kalman.var_yaw", po::value<double>()->required())
		("kalman.var_w", po::value<double>()->required())
		("kalman.std_j", po::value<double>()->required(), "process noise std for linear jerk")
		("kalman.std_wd", po::value<double>()->required(), "process noise std for yaw acceleration")
	;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	string cfg_fname {vm["config"].as<string>()}; 
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
	if (vm["general.autopilot"].as<bool>())
		vehicle->SetAutopilot();

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
	Localizer localizer (vm, world, blueprint_library, ego, mapCloud);

	// main loop
	Pose poseRef = getTruePose(vehicle);
	double maxError = 0;
	double meanError = 0;
	size_t n_steps = 0;
  
	while (!viewer->wasStopped())
  	{
		while (!localizer.MeasurementIsReady())
			std::this_thread::sleep_for(0.1s);

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
		
		if (localizer.MeasurementIsReady()) {
			viewer->removePointCloud("scan");
			renderPointCloud(viewer, localizer.Localize(), "scan", Color(1,0,0) );

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
