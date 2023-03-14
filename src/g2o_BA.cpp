#include "DvlGyroOptimizer.h"

#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>

#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "G2oTypes.h"
#include "Converter.h"
#include "System.h"

#include <mutex>
#include <thread>
#include <chrono>

#include "OptimizableTypes.h"
#include "RosHandling.h"
#include "ORB_DVL2/SetInfof.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


struct vertex_cmp
{
    bool operator()(const g2o::OptimizableGraph::Vertex* v1, const g2o::OptimizableGraph::Vertex* v2) const
    {
        return v1->id() < v2->id();
    }
};

//number of fixed keyframes
const int fixedKFNum = 10;

// Setup optimizer
g2o::SparseOptimizer* optimizer = nullptr;
g2o::BlockSolverX::LinearSolverType* linearSolver = nullptr;
g2o::BlockSolverX* solver_ptr = nullptr;
g2o::OptimizationAlgorithmLevenberg* solver = nullptr;
// init vector for edges pointer
std::vector<EdgePriorAcc*> edge_prior_acc;
std::vector<EdgePriorGyro*> edge_prior_gyro;
std::vector<EdgeDvlVelocity*> edge_dvl_velocity;
std::vector<EdgeMonoBA_DvlGyros*> edge_mono;
std::vector<EdgeStereoBA_DvlGyros*> edge_stereo;
// std::vector<EdgeMonoBA_DvlGyros*> edge_mono_firstKF;
// std::vector<EdgeStereoBA_DvlGyros*> edge_stereo_firstKF;
std::vector<EdgeDvlIMU*> edge_dvl_imu;
// init set for vertices pointer
std::set<VertexPoseDvlIMU*, vertex_cmp> vertex_pose;
std::set<VertexPoseDvlIMU*, vertex_cmp> vertex_pose_Fixed;
std::set<g2o::VertexSBAPointXYZ*, vertex_cmp> vertex_point;
std::set<g2o::VertexSBAPointXYZ*, vertex_cmp> vertex_point_Fixed;
// record map point observation map: map_point -> KeyFrame
std::map<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*> map_point_observation;
// record original pose of KF
std::map<VertexPoseDvlIMU*, Eigen::Isometry3d> map_pose_original;
// gravity direction
VertexGDir* v_GDir = nullptr;
g2o::VertexSE3Expmap* v_Tbd = nullptr;
g2o::VertexSE3Expmap* v_Tdc = nullptr;
// publisher pointer
boost::shared_ptr<ros::Publisher> p_markers_pub;

void DeleteGraphRviz()
{
    visualization_msgs::MarkerArray marker_delete;
    visualization_msgs::Marker marker_d;
    marker_d.header.frame_id = "orb_slam";
    marker_d.header.stamp = ros::Time();
    marker_d.action = visualization_msgs::Marker::DELETEALL;
    marker_delete.markers.push_back(marker_d);
    p_markers_pub->publish(marker_delete);
}

void PublishGraph()
{
    Eigen::Matrix3d R_b0w = v_GDir->estimate().Rwg;
    Eigen::Isometry3d T_b_d = v_Tbd->estimate();
    Eigen::Isometry3d T_d_c = v_Tdc->estimate();
    Eigen::Isometry3d T_b_c = T_b_d * T_d_c;
    Eigen::Matrix3d R_b_c = T_b_c.rotation();
    Eigen::Matrix3d R_c0w = R_b_c.transpose() * R_b0w;
    Eigen::Matrix3d R_wc0 = R_c0w.transpose();
    Eigen::Vector3d v_test;
    v_test = R_wc0 * v_test;


    visualization_msgs::MarkerArray marker_array;
    // add vertices
    for (auto v: vertex_pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "orb_slam";
        marker.header.stamp = ros::Time();
        marker.ns = "vertex";
        marker.id = v->id();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.g = 1.0;
        marker.color.a = 1.0;
        Eigen::Vector3d twc = R_wc0 * v->estimate().twc;
        marker.pose.position.x = twc[0];
        marker.pose.position.y = twc[1];
        marker.pose.position.z = twc[2];
        marker_array.markers.push_back(marker);
    }
    for (auto v: vertex_point) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "orb_slam";
        marker.header.stamp = ros::Time();
        marker.ns = "vertex";
        marker.id = v->id();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        Eigen::Vector3d p_w = R_wc0 * v->estimate();
        marker.pose.position.x = p_w[0];
        marker.pose.position.y = p_w[1];
        marker.pose.position.z = p_w[2];
        marker_array.markers.push_back(marker);
    }

    // add edge
    for (auto e: edge_mono) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "orb_slam";
        marker.header.stamp = ros::Time();
        marker.ns = "edge";
        marker.id = e->id();
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.0001;
        marker.color.b = 1.0;
        marker.color.a = 0.5;
        auto all_v = e->vertices();
        VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(all_v[0]);
        g2o::VertexSBAPointXYZ* v2 = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_v[1]);
        geometry_msgs::Point p1, p2;
        Eigen::Vector3d twc = R_wc0 * v1->estimate().twc;
        p1.x = twc[0];
        p1.y = twc[1];
        p1.z = twc[2];
        Eigen::Vector3d p_w = R_wc0 * v2->estimate();
        p2.x = p_w[0];
        p2.y = p_w[1];
        p2.z = p_w[2];
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        // marker_array.markers.push_back(marker);
    }
    for (auto e: edge_stereo) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "orb_slam";
        marker.header.stamp = ros::Time();
        marker.ns = "edge";
        marker.id = e->id();
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.0001;
        marker.color.b = 1.0;
        marker.color.a = 0.5;
        auto all_v = e->vertices();
        VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(all_v[0]);
        g2o::VertexSBAPointXYZ* v2 = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_v[1]);
        geometry_msgs::Point p1, p2;
        Eigen::Vector3d twc = R_wc0 * v1->estimate().twc;
        p1.x = twc[0];
        p1.y = twc[1];
        p1.z = twc[2];
        Eigen::Vector3d p_w = R_wc0 * v2->estimate();
        p2.x = p_w[0];
        p2.y = p_w[1];
        p2.z = p_w[2];
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        // marker_array.markers.push_back(marker);
    }
    p_markers_pub->publish(marker_array);
}

bool OptimizeBA(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    if (!optimizer) {
        ROS_ERROR_STREAM("Optimizer is not initialized.");
        return false;
    }
    // get error before optimization
    double visual_chi2 = 0, dvl_chi2 = 0;
    for (auto e: edge_mono) {
        e->computeError();
        e->setLevel(0);
        visual_chi2 += e->chi2();
    }
    for (auto e: edge_stereo) {
        e->computeError();
        e->setLevel(0);
        visual_chi2 += e->chi2();
    }
    for (auto e: edge_dvl_imu) {
        e->computeError();
        e->setLevel(0);
        dvl_chi2 += e->chi2();
    }
    ROS_INFO_STREAM("Before optimization, visual chi2: " << visual_chi2 << ", dvl chi2: " << dvl_chi2);
    //optimize the graph


    optimizer->setVerbose(true);
    for (int i = 0; i < 2; i++) {
        optimizer->initializeOptimization(0);
        optimizer->optimize(10);
        PublishGraph();
    }

    // get error after optimization
    visual_chi2 = 0, dvl_chi2 = 0;
    for (auto e: edge_mono) {
        e->computeError();
        visual_chi2 += e->chi2();
    }
    for (auto e: edge_stereo) {
        e->computeError();
        visual_chi2 += e->chi2();
    }
    for (auto e: edge_dvl_imu) {
        e->computeError();
        dvl_chi2 += e->chi2();
    }
    ROS_INFO_STREAM("After optimization " << " visual chi2: " << visual_chi2 << ", dvl chi2: " << dvl_chi2);

    return true;
}

bool OptimizePoseGraph(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    if (!optimizer) {
        ROS_ERROR_STREAM("Optimizer is not initialized.");
        return false;
    }
    /************************first pose graph optimization************************/
    double visual_chi2 = 0, dvl_chi2 = 0;
    for (auto e: edge_mono) {
        e->computeError();
        e->setLevel(1);
        visual_chi2 += e->chi2();
    }
    for (auto e: edge_stereo) {
        e->computeError();
        e->setLevel(1);
        visual_chi2 += e->chi2();
    }
    for (auto e: edge_dvl_imu) {
        e->computeError();
        e->setLevel(0);
        dvl_chi2 += e->chi2();
    }
    map_pose_original.clear();
    for (auto v: vertex_pose) {
        v->setFixed(false);
        Eigen::Isometry3d T_c0_cj = Eigen::Isometry3d::Identity();
        T_c0_cj.rotate(v->estimate().Rwc);
        T_c0_cj.pretranslate(v->estimate().twc);
        map_pose_original.insert(std::pair<VertexPoseDvlIMU*, Eigen::Isometry3d>(v, T_c0_cj));
    }
    for (auto v: vertex_pose_Fixed) {
        v->setFixed(true);
    }
    optimizer->setVerbose(false);
    for (int i = 0; i < 5; i++) {
        optimizer->initializeOptimization(0);
        optimizer->optimize(10);
        PublishGraph();
    }
    //update map point position
    for(auto kf_pose:map_pose_original){
        for (auto mp_kf:map_point_observation){
            if(mp_kf.second == kf_pose.first){
                Eigen::Isometry3d T_c0_cj = kf_pose.second;
                Eigen::Isometry3d T_c0_cjnew = Eigen::Isometry3d::Identity();
                T_c0_cjnew.rotate(kf_pose.first->estimate().Rwc);
                T_c0_cjnew.pretranslate(kf_pose.first->estimate().twc);
                Eigen::Isometry3d T_cjnew_cj = T_c0_cjnew.inverse()*T_c0_cj;
                Eigen::Vector3d mp_pos = mp_kf.first->estimate();
                Eigen::Vector3d mp_pos_new = T_c0_cjnew * T_c0_cj.inverse()*mp_pos;
                mp_kf.first->setEstimate(mp_pos_new);
            }
        }
    }
    PublishGraph();
    // sleep this thread for 1 second
    auto now1 = std::chrono::system_clock::now();
    auto later1 = now1 + std::chrono::seconds(1);
    std::this_thread::sleep_until(later1);

    /************************second map point optimization************************/
    for (auto e: edge_mono) {
        e->setLevel(0);
    }
    for (auto e: edge_stereo) {
        e->setLevel(0);
    }
    for (auto e: edge_dvl_imu) {
        e->setLevel(0);
    }
    for (auto v: vertex_pose) {
        v->setFixed(true);
    }
    for (auto v: vertex_point) {
        v->setFixed(false);
    }
    for(auto a: vertex_point_Fixed) {
        a->setFixed(true);
    }
    for (auto e: edge_mono) {
        e->computeError();
        if (e->chi2()>10) {
            // std::remove(edge_mono.begin(), edge_mono.end(), e);
            // g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*>(e->vertices()[1]);
            // vertex_point.erase(v);
            // vertex_point_Fixed.erase(v);
            // optimizer->vertices().erase(v->id());
            // optimizer->edges().erase(e);
            // e->setInformation(Eigen::Matrix2d::Identity() * 0.0001);
            e->setLevel(1);
        }
        else {
            e->setLevel(0);
            // e->setInformation(Eigen::Matrix2d::Identity() * 1);
        }

    }
    for (auto e: edge_stereo) {
        e->computeError();
        if (e->chi2()>10) {
            // std::remove(edge_stereo.begin(), edge_stereo.end(), e);
            // g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*>(e->vertices()[1]);
            // vertex_point.erase(v);
            // vertex_point_Fixed.erase(v);
            // optimizer->vertices().erase(v->id());
            // optimizer->edges().erase(e);
            // e->setInformation(Eigen::Matrix3d::Identity() * 0.0001);
            e->setLevel(1);
        }
        else {
            e->setLevel(0);
        }
    }
    optimizer->setVerbose(false);
    for (int i = 0; i < 100; i++) {
        optimizer->initializeOptimization(0);
        optimizer->optimize(5);
        PublishGraph();



    }
    // sleep this thread for 1 second
    // DeleteGraphRviz();
    auto now2 = std::chrono::system_clock::now();
    auto later2 = now2 + std::chrono::seconds(1);
    std::this_thread::sleep_until(later2);


    /************************third BA optimization************************/
    // DeleteGraphRviz();

    // get error after optimization
    visual_chi2 = 0, dvl_chi2 = 0;
    // for (auto e: edge_mono) {
    //     e->computeError();
    //     e->setLevel(0);
    //     visual_chi2 += e->chi2();
    // }
    // for (auto e: edge_stereo) {
    //     e->computeError();
    //     e->setLevel(0);
    //     visual_chi2 += e->chi2();
    // }
    // for (auto e: edge_dvl_imu) {
    //     e->computeError();
    //     e->setLevel(0);
    //     dvl_chi2 += e->chi2();
    // }
    for (auto v: vertex_pose) {
        v->setFixed(false);
    }
    for (auto v: vertex_point) {
        v->setFixed(false);
    }
    for (auto a: vertex_point_Fixed) {
        a->setFixed(true);
    }
    for (auto a: vertex_pose_Fixed) {
        a->setFixed(true);
    }
    optimizer->initializeOptimization(0);
    optimizer->optimize(50);
    PublishGraph();

    return true;
}

bool OptimizeExtrinsic(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    for (auto v: vertex_pose) {
        v->setFixed(true);
    }
    for (auto v: vertex_point) {
        v->setFixed(true);
    }
    v_GDir->setFixed(false);
    v_Tbd->setFixed(false);
    v_Tdc->setFixed(false);
    optimizer->setVerbose(true);
    optimizer->initializeOptimization(0);
    optimizer->optimize(50);
    v_GDir->setFixed(true);
    v_Tbd->setFixed(true);
    v_Tdc->setFixed(true);
    return true;
}

bool ReloadGraph(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    DeleteGraphRviz();
    // clear the vector
    edge_prior_acc.clear();
    edge_prior_gyro.clear();
    edge_dvl_velocity.clear();
    edge_mono.clear();
    edge_stereo.clear();
    edge_dvl_imu.clear();
    // clear the set
    vertex_pose.clear();
    vertex_pose_Fixed.clear();
    vertex_point.clear();
    vertex_point_Fixed.clear();
    // clear the map
    map_point_observation.clear();
    map_pose_original.clear();
    // load the graph
    optimizer->edges().clear();
    optimizer->vertices().clear();
    optimizer->clear();
    optimizer->load("/home/da/project/ros/orb_dvl2_ws/src/dvl2/data/g2o/full_BA.g2o");
    int edge_id = 0;
    for (auto e: optimizer->edges()) {
        if (EdgePriorAcc* e_pri_acc = dynamic_cast<EdgePriorAcc*>(e)) {
            // e_pri_acc->computeError();
            edge_prior_acc.push_back(e_pri_acc);
            e_pri_acc->setId(edge_id++);
            // std::cout << "EdgePriorAcc ID:"<<e_pri_acc->id()<<", error:"<<e_pri_acc->chi2()<<std::endl;
        }
        if (EdgePriorGyro* e_pri_gyr = dynamic_cast<EdgePriorGyro*>(e)) {
            // e_pri_gyr->computeError();
            edge_prior_gyro.push_back(e_pri_gyr);
            e_pri_gyr->setId(edge_id++);
            // std::cout << "EdgePriorGyro ID:"<<e_pri_gyr->id()<<", error:"<<e_pri_gyr->chi2()<<std::endl;
        }
        if (EdgeDvlVelocity* e_dvl_vel = dynamic_cast<EdgeDvlVelocity*>(e)) {
            // e_dvl_vel->computeError();
            edge_dvl_velocity.push_back(e_dvl_vel);
            e_dvl_vel->setId(edge_id++);
            // std::cout << "EdgeDvlVelocity ID:"<<e_dvl_vel->id()<<", error:"<<e_dvl_vel->chi2()<<std::endl;
        }
        if (EdgeMonoBA_DvlGyros* e_mono = dynamic_cast<EdgeMonoBA_DvlGyros*>(e)) {
            // e_mono->computeError();
            auto all_v = e_mono->vertices();
            edge_mono.push_back(e_mono);
            e_mono->setId(edge_id++);
            // std::cout << "EdgeMonoBA_DvlGyros from:"<<all_v[0]->id()<<", to:"<<all_v[1]->id()<<", error:"<<e_mono->chi2()<<std::endl;
            // add vertex to set, if it is not in the set
            auto all_verteices = e_mono->vertices();
            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                vertex_pose.insert(v_pose);
                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                    vertex_point.insert(v_point);
                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                        map_point_observation.insert(
                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                    }
                }
            }
        }
        if (EdgeStereoBA_DvlGyros* e_stereo = dynamic_cast<EdgeStereoBA_DvlGyros*>(e)) {
            // e_stereo->computeError();
            auto all_v = e_stereo->vertices();
            edge_stereo.push_back(e_stereo);
            e_stereo->setId(edge_id++);
            // std::cout << "EdgeStereoBA_DvlGyros from:"<<all_v[0]->id()<<", to:"<<all_v[1]->id()<<", error:"<<e_stereo->chi2()<<std::endl;
            // add vertex to set, if it is not in the set
            auto all_verteices = e_stereo->vertices();
            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                vertex_pose.insert(v_pose);
                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                    vertex_point.insert(v_point);
                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                        map_point_observation.insert(
                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                    }
                }
            }
        }
        if (EdgeDvlIMU* e_dvl_imu = dynamic_cast<EdgeDvlIMU*>(e)) {
            // e_dvl_imu->computeError();
            e_dvl_imu->setLevel(0);
            edge_dvl_imu.push_back(e_dvl_imu);
            e_dvl_imu->setId(edge_id++);
            // std::cout << "EdgeDvlIMU ID:"<<e_dvl_imu->id()<<", error:"<<e_dvl_imu->chi2()<<std::endl;
            auto all_verteices = e_dvl_imu->vertices();
            for (auto v: all_verteices) {
                if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(v)) {
                    vertex_pose.insert(v_pose);
                }
                if (VertexGDir* v_gdir = dynamic_cast<VertexGDir*>(v)) {
                    v_GDir = v_gdir;
                }
            }
            v_Tdc = dynamic_cast<g2o::VertexSE3Expmap*>(all_verteices[6]);
            v_Tbd = dynamic_cast<g2o::VertexSE3Expmap*>(all_verteices[7]);
        }
    }

    int first_vertex_id = (*vertex_pose.begin())->id();
    for (auto e: edge_mono) {
        if (e->vertices()[0]->id() < first_vertex_id + fixedKFNum) {
            vertex_pose_Fixed.insert(dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]));
            vertex_point_Fixed.insert(dynamic_cast<g2o::VertexSBAPointXYZ*>(e->vertices()[1]));
        }
    }
    for (auto e: edge_stereo) {
        if (e->vertices()[0]->id() < first_vertex_id + fixedKFNum) {
            vertex_pose_Fixed.insert(dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]));
            vertex_point_Fixed.insert(dynamic_cast<g2o::VertexSBAPointXYZ*>(e->vertices()[1]));
        }
    }
    for (auto v: vertex_pose) {
        Eigen::Isometry3d T_c0_cj = Eigen::Isometry3d::Identity();
        T_c0_cj.rotate(v->estimate().Rwc);
        T_c0_cj.pretranslate(v->estimate().twc);
        map_pose_original.insert(std::pair<VertexPoseDvlIMU*, Eigen::Isometry3d>(v, T_c0_cj));
    }
    ROS_INFO_STREAM("Load graph done, vertex_pose size: " << vertex_pose.size() << ", vertex_point size: "
                                                          << vertex_point.size() << ", edge_mono size: "
                                                          << edge_mono.size() << ", edge_stereo size: "
                                                          << edge_stereo.size() << ", edge_dvl_imu size: "
                                                          << edge_dvl_imu.size() << ", edge_prior_acc size: "
                                                          << edge_prior_acc.size() << ", edge_prior_gyro size: "
                                                          << edge_prior_gyro.size()
                                                          << ", edge_dvl_velocity size: "
                                                          << edge_dvl_velocity.size());
    ROS_INFO_STREAM("graph chi2: " << optimizer->chi2());
    PublishGraph();
    return true;
}

bool SetInfor(ORB_DVL2::SetInfofRequest &req, ORB_DVL2::SetInfofResponse &res)
{
    // set the dvl edge information matrix
    for (auto e: edge_dvl_imu) {
        e->setInformation(req.DVL_Infor * Eigen::Matrix<double, 9, 9>::Identity() *
                          (edge_mono.size() + edge_stereo.size()));
    }
    // set the visual edge information matrix
    for (auto e: edge_mono) {
        e->setInformation(req.Visual_Infor * Eigen::Matrix<double, 2, 2>::Identity());
    }
    for (auto e: edge_stereo) {
        e->setInformation(req.Visual_Infor * Eigen::Matrix<double, 3, 3>::Identity());
    }

    ROS_INFO_STREAM(
            "Set information done, DVL Infor: " << req.DVL_Infor << ", Visual Infor: " << req.Visual_Infor);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BAg2o");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("/g2oGraph/SetInfor", SetInfor);
    ros::ServiceServer service2 = nh.advertiseService("/g2oGraph/Reload", ReloadGraph);
    ros::ServiceServer service3 = nh.advertiseService("/g2oGraph/OptimizeBA", OptimizeBA);
    ros::ServiceServer service4 = nh.advertiseService("/g2oGraph/OptimizePose", OptimizePoseGraph);
    ros::ServiceServer service5 = nh.advertiseService("/g2oGraph/OptimizeExtrinsicAndG", OptimizeExtrinsic);
    ros::Publisher markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/g2oGraph/GraphMarker", 10);
    p_markers_pub = boost::shared_ptr<ros::Publisher>(boost::make_shared<ros::Publisher>(markers_pub));

    optimizer = new g2o::SparseOptimizer();
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer->setAlgorithm(solver);
    optimizer->setVerbose(true);


    ros::spin();
    return 0;
}
