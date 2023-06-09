#include <Eigen/Core>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <chrono>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_listener");

    ros::NodeHandle nh;
    std::map<double, Eigen::Isometry3d> estimation_traj;
    std::string pose_path = "/home/da/project/ros/orb_dvl2_ws/src/dvl2/dvl2_results/dense/WholeTank_Medium_traj.txt";
    if (nh.getParam("/dense_ros/pose_path", pose_path))
    {
        ROS_INFO("Got pose_path: %s", pose_path.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param '/dense_ros/pose_path'");
    }
    std::ifstream estimation_file(pose_path);
    if (!estimation_file.is_open()) {
        std::cout << "Failed to open file:"<<pose_path << std::endl;
        return -1;
    }
    std::string line;
    while (std::getline(estimation_file, line)) {
        std::istringstream iss(line);
        if (line[0] == '#') {
            continue;
        }
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            std::cout << "Failed to parse line: " << line << std::endl;
            continue;
        }

        Eigen::Quaterniond quaternion(qw, qx, qy, qz);
        Eigen::Matrix3d R = quaternion.normalized().toRotationMatrix();
        Eigen::Vector3d t(tx, ty, tz);
        Eigen::Isometry3d T;
        T.setIdentity();
        T.pretranslate(t);
        T.rotate(R);
        estimation_traj.insert(std::make_pair(timestamp, T));

    }

    estimation_file.close();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string pcd_file = "";
    if (nh.getParam("/dense_ros/pcd_path", pcd_file))
    {
        ROS_INFO("Got pose_path: %s", pcd_file.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param '/dense_ros/pcd_path'");
    }
    pcl::io::loadPCDFile(pcd_file,*global_map);

    pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
    vgf.setInputCloud(global_map);
    vgf.setLeafSize(0.01, 0.01, 0.01);
    vgf.filter(*global_map);
    // Initialize visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor(1,1,1);
    viewer->initCameraParameters();

//    viewer->setCameraPosition(2.30598, -17.8696, 0.269679, 1.40695, 0.316864, 1.29969, 0.970534, 0.0344703, 0.238487); // dvl2 odom
    viewer->setCameraPosition(2.36829, -21.6902, -0.0232327, 1.40695, 0.316864, 1.29969, 0.961521, 0.0255591, 0.273539); // dvl2 slam
    viewer->setCameraFieldOfView(0.523599);
    viewer->setCameraClipDistances(0.00522511, 50);
    // Assuming that "cloud" is your point cloud
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(global_map);
    viewer->addPointCloud<pcl::PointXYZRGB> (global_map, rgb, "global cloud");

    // Assuming "estimation_traj" is your map of Eigen::Isometry3d
    std::map<double, Eigen::Isometry3d>::iterator it = estimation_traj.begin();
    Eigen::Isometry3d prev_transform = it->second;

    Eigen::Vector3d v_color(0,0,1);
    // Convert Eigen::Isometry3d to pcl::PointXYZ and add lines
    for (it++; it != estimation_traj.end(); it++) {
        Eigen::Isometry3d current_transform = it->second;

        // Convert Eigen::Isometry3d translations to pcl::PointXYZ
        pcl::PointXYZ prev_point(prev_transform.translation().x(), prev_transform.translation().y(), prev_transform.translation().z());
        pcl::PointXYZ current_point(current_transform.translation().x(), current_transform.translation().y(), current_transform.translation().z());


        // Add the line to the viewer with id as the timestamp
        std::string line_id = std::to_string(it->first);
        viewer->addLine<pcl::PointXYZ>(prev_point, current_point, v_color.x(), v_color.y(), v_color.z(), line_id);


        // Update the previous transform
        prev_transform = current_transform;
    }

    // Main visualization loop
    while (!viewer->wasStopped ()) {
        double x, y, z, pos_x, pos_y, pos_z;
        std::vector<pcl::visualization::Camera> cam;

        //Save the position of the camera
        viewer->getCameras(cam);

        //Print recorded points on the screen:
        cout << "Cam: " << endl
             << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl
             << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl
             << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;

//        viewer->setCameraPosition()

        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
