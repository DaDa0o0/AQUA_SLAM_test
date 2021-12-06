#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;

int main(int argc, char **argv)
{
	Eigen::Vector3d t;
	// x(roll) y(pitch) z(yaw) eular angle
	Eigen::Vector3d eular;
	Eigen::AngleAxisd r_x;
	Eigen::AngleAxisd r_y;
	Eigen::AngleAxisd r_z;
	Eigen::Quaterniond q_r(Eigen::Matrix3d::Identity());

	Eigen::Isometry3d T_dvl_camera=Eigen::Isometry3d::Identity();
	t<<0.0, 0.0, 0.0;
	eular<<-0.5 * M_PI, 0, -0.5 * M_PI;
	r_z=Eigen::AngleAxisd(eular.z(),Eigen::Vector3d::UnitZ());
	r_y=Eigen::AngleAxisd(eular.y(),Eigen::Vector3d::UnitY());
	r_x=Eigen::AngleAxisd(eular.x(),Eigen::Vector3d::UnitX());
//	q_r=q_r*r_y*r_z;

//	T_dvl_orb.rotate(r_z);
	T_dvl_camera.rotate(r_z);
	T_dvl_camera.rotate(r_x);
	T_dvl_camera.pretranslate(t);


	Eigen::Isometry3d T_gyros_camera=Eigen::Isometry3d::Identity();
	t<<0, 0, 0;
	eular<<-0.5 * M_PI, 0, -0.5 * M_PI;
	r_z=Eigen::AngleAxisd(eular.z(),Eigen::Vector3d::UnitZ());
	r_y=Eigen::AngleAxisd(eular.y(),Eigen::Vector3d::UnitY());
	r_x=Eigen::AngleAxisd(eular.x(),Eigen::Vector3d::UnitX());
	q_r=q_r*r_z;
	T_gyros_camera.rotate(r_z);
	T_gyros_camera.rotate(r_x);




	cout<<"R_gyros_camera: \n"<<T_gyros_camera.matrix()<<"\n"
	<<"R_dvl_orb(euler yaw-pitch-roll)"<<T_gyros_camera.rotation().normalized().eulerAngles(2,1,0)<<"\n"
	<<endl;
	cout<<"T_dvl_rcamera: \n"<<T_dvl_camera.matrix()<<"\n"
		<<"R_dvl_rcamera(euler yaw-pitch-roll)"<<T_dvl_camera.rotation().normalized().eulerAngles(2,1,0)<<"\n"
		<<endl;










	return 0;
}

