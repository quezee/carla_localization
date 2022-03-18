#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <vector>
#include <Eigen/Geometry>
#include <carla/client/Vehicle.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using std::vector;
using std::to_string;
using std::min;
using std::max;

namespace cc = carla::client;
namespace cg = carla::geom;

const double pi = M_PI;

struct Point{
	double x, y, z;

	Point()
		: x(0), y(0), z(0){}

	Point(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ){}

	Point(cg::Vector3D vec)
		: x(vec.x), y(vec.y), z(vec.z){}

	void Print(){
		cout << "x: " << x << " y: " << y << " z: " << z << endl;
	}

	Point& operator+=(const Point& rhs) {
		this->x += rhs.x;
		this->y += rhs.y;
		this->z += rhs.z;
		return *this;
	}
};

struct Rotate{
	double yaw, pitch, roll;

	Rotate()
		: yaw(0), pitch(0), roll(0){}

	Rotate(double setYaw, double setPitch, double setRoll)
		: yaw(setYaw), pitch(setPitch), roll(setRoll){}

	Rotate(cg::Vector3D vec)
		: yaw(vec.z), pitch(vec.y), roll(vec.x){}

	Rotate& operator+=(const Rotate& rhs) {
		this->yaw += rhs.yaw;
		this->pitch += rhs.pitch;
		this->roll += rhs.roll;
		return *this;
	}
	Rotate& operator%=(double mod) {
		this->yaw = fmod(this->yaw, mod);
		this->pitch = fmod(this->pitch, mod);
		this->roll = fmod(this->roll, mod);
		return *this;
	}	

	void Print(){
		cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl;
	}
};

struct Pose{

	Point position;
	Rotate rotation;

	Pose()
		: position(Point(0, 0, 0)), rotation(Rotate(0, 0, 0)){}

	Pose(Point setPos, Rotate setRotation)
		: position(setPos), rotation(setRotation) {}

	Pose operator-(const Pose& p)
    {
        Pose result(Point(position.x-p.position.x, position.y-p.position.y, position.z-p.position.z), Rotate(rotation.yaw-p.rotation.yaw, rotation.pitch-p.rotation.pitch, rotation.roll-p.rotation.roll) );
        return result;
    }
};

struct ControlState{

	float t;
	float s;
	float b;

	ControlState(float setT, float setS, float setB)
		: t(setT), s(setS), b(setB) {}
};

struct Color
{

        float r, g, b;

        Color(float setR, float setG, float setB)
                : r(setR), g(setG), b(setB)
        {}
};

struct BoxQ
{
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length;
    float cube_width;
    float cube_height;
};

Eigen::Matrix4f transform2D(double theta, double xt, double yt);
Eigen::Matrix4f transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt);
Pose getPose(Eigen::Matrix4f matrix);
Eigen::Matrix4f getTransform (Pose pose);
double getDistance(Point p1, Point p2);
double minDistance(Point p1, vector<Point> points);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color, int renderSize = 4);
void renderRay(pcl::visualization::PCLVisualizer::Ptr& viewer, Point p1, Point p2, std::string name, Color color);
void renderPath(pcl::visualization::PCLVisualizer::Ptr& viewer, const PointCloudT::Ptr& cloud, std::string name, Color color);
Eigen::Quaternionf getQuaternion(float theta);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity);
void Accuate(ControlState response, cc::Vehicle::Control& state);
void drawCar(const Pose& pose, int num, const Color& color, double alpha,
             pcl::visualization::PCLVisualizer::Ptr& viewer);
Pose getTruePose(const boost::shared_ptr<cc::Vehicle>& vehicle, Pose poseRef = Pose());
