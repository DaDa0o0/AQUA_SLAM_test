//
// Created by da on 03/12/2021.
//
#include "DenseMapper.h"
#include "KeyFrame.h"
#include "Map.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <chrono>

namespace ORB_SLAM3
{
DenseMapper::DenseMapper(string settingFile)
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it(nh_);
	image_transport::Publisher depth_pub = it.advertise("/ORBSLAM3_tight/dense_mapper/depth", 10);
	mDepthPub =
		boost::shared_ptr<image_transport::Publisher>(boost::make_shared<image_transport::Publisher>(depth_pub));
	image_transport::Publisher depth_conf_pub = it.advertise("/ORBSLAM3_tight/dense_mapper/depth_cpnfidence", 10);
	mDepthConfPub =
		boost::shared_ptr<image_transport::Publisher>(boost::make_shared<image_transport::Publisher>(depth_conf_pub));
	ros::Publisher pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/ORBSLAM3_tight/dense_map", 10);
	mMapPub = boost::shared_ptr<ros::Publisher>(boost::make_shared<ros::Publisher>(pointcloud_pub));

	FileStorage fs(settingFile, FileStorage::READ);
	FileNode node = fs["DenseMapper"];
	float P1 = (float)node["P1"];
	float P2 = (float)node["P2"];
	int correlation_window_size = (int)node["correlation_window_size"];
	int disp12MaxDiff = (int)node["disp12MaxDiff"];
	int disparity_range = (int)node["disparity_range"];
	int min_disparity = (int)node["min_disparity"];
	int prefilter_cap = (int)node["prefilter_cap"];
	int prefilter_size = (int)node["prefilter_size"];
	int speckle_range = (int)node["speckle_range"];
	int speckle_size = (int)node["speckle_size"];
	int texture_threshold = (int)node["texture_threshold"];
	float uniqueness_ratio = (float)node["uniqueness_ratio"];

	mParam = DepthEstParamters(P1,
							   P2,
							   correlation_window_size,
							   disp12MaxDiff,
							   disparity_range,
							   min_disparity,
							   prefilter_cap,
							   prefilter_size,
							   speckle_range,
							   speckle_size,
							   texture_threshold,
							   uniqueness_ratio);

	double image_scale = (double)fs["ImageScale"];
	fx = (double)fs["Camera.fx"] * image_scale;
	fy = (double)fs["Camera.fy"] * image_scale;
	cx = (double)fs["Camera.cx"] * image_scale;
	cy = (double)fs["Camera.cy"] * image_scale;
	bf = (double)fs["Camera.bf"] * image_scale;

	mEnable = !((float)fs["enable"] == 0);

}

void DenseMapper::InsertNewKF(KeyFrame *pKF)
{
	if (pKF->GetMap()->GetAllKeyFrames().size() < 5) {
		ROS_INFO_STREAM("DenserMapper: less than 5 KF in current map, skip");
		return;
	}
	else if (pKF->GetMap()->GetAllKeyFrames().size() == 5) {
		auto all_KF = pKF->GetMap()->GetAllKeyFrames();
		for (auto pKFi: all_KF) {
			std::lock_guard<std::mutex> lock(mKFQueueMutex);
			mKFQueue.push(pKFi);
		}
	}
	else {
		std::lock_guard<std::mutex> lock(mKFQueueMutex);
		mKFQueue.push(pKF);
	}
}
void DenseMapper::ComputeDisp(const Mat &left,
							  const Mat &right,
							  Mat &out,
							  Mat &out_conf,
							  DepthEstParamters &param)
{
	using namespace ximgproc;

	String algo = "bm";
	String filter = "wls_conf";
	bool no_downscale = true;

	double lambda = 8000;
	double sigma = 1.5;

	Mat left_for_matcher, right_for_matcher;
	Mat left_disp, right_disp;
	Mat filtered_disp, solved_disp, solved_filtered_disp;
	Mat conf_map = Mat(left.rows, left.cols, CV_8U);
	conf_map = Scalar(255);
	Rect ROI;
	Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
	double matching_time, filtering_time;
	double solving_time = 0;
//	if (param._disp12MaxDiff <= 0 || param._disp12MaxDiff % 16 != 0) {
//		cout << "Incorrect max_disparity value: it should be positive and divisible by 16";
//		return;
//	}
	if (param._correlation_window_size <= 0 || param._correlation_window_size % 2 != 1) {
		ROS_ERROR_STREAM("DenseMapper: Incorrect window_size value: it should be positive and odd");
		return;
	}

	if (filter == "wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
	{
		if (!no_downscale) {
			// downscale the views to speed-up the matching stage, as we will need to compute both left
			// and right disparity maps for confidence map computation
			//! [downscale]
//			param._disp12MaxDiff /= 2;
//			if (param._disp12MaxDiff % 16 != 0) {
//				param._disp12MaxDiff += 16 - (param._disp12MaxDiff % 16);
//			}
			// resize(left ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR_EXACT);
			// resize(right,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR_EXACT);
			resize(left, left_for_matcher, Size(), 0.5, 0.5, INTER_LINEAR);
			resize(right, right_for_matcher, Size(), 0.5, 0.5, INTER_LINEAR);
			//! [downscale]
		}
		else {
			left_for_matcher = left.clone();
			right_for_matcher = right.clone();
		}

		if (algo == "bm") {
			//! [matching]
			Ptr<StereoBM> left_matcher = StereoBM::create(param._disparity_range, param._correlation_window_size);
			left_matcher->setDisp12MaxDiff(param._disp12MaxDiff);
			left_matcher->setMinDisparity(param._min_disparity);
			left_matcher->setPreFilterCap(param._prefilter_cap);
			left_matcher->setPreFilterSize(param._prefilter_size);
			left_matcher->setSpeckleRange(param._speckle_range);
			left_matcher->setSpeckleWindowSize(param._speckle_size);
			left_matcher->setTextureThreshold(param._texture_threshold);
			left_matcher->setUniquenessRatio(param._uniqueness_ratio);
			wls_filter = createDisparityWLSFilter(left_matcher);
			Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

			if (left_for_matcher.channels() == 3 || left_for_matcher.channels() == 4) {
				cvtColor(left_for_matcher, left_for_matcher, COLOR_BGR2GRAY);
				cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
			}


			matching_time = (double)getTickCount();
			left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
			matching_time = ((double)getTickCount() - matching_time) / getTickFrequency();
			//! [matching]
		}
		else if (algo == "sgbm") {
			Ptr<StereoSGBM> left_matcher =
				StereoSGBM::create(param._min_disparity, param._disparity_range, param._correlation_window_size);
			left_matcher->setP1(param._P1);
			left_matcher->setP2(param._P2);
			left_matcher->setPreFilterCap(param._prefilter_cap);
			left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
			wls_filter = createDisparityWLSFilter(left_matcher);
			Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

			matching_time = (double)getTickCount();
			left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
			matching_time = ((double)getTickCount() - matching_time) / getTickFrequency();
		}
		else {
			cout << "Unsupported algorithm";
			return;
		}

		//! [filtering]
		wls_filter->setLambda(lambda);
		wls_filter->setSigmaColor(sigma);
		filtering_time = (double)getTickCount();
		wls_filter->filter(left_disp, left, filtered_disp, right_disp);
		filtering_time = ((double)getTickCount() - filtering_time) / getTickFrequency();
		//! [filtering]
		conf_map = wls_filter->getConfidenceMap();

		// Get the ROI that was used in the last filter call:
		ROI = wls_filter->getROI();
		if (!no_downscale) {
			// upscale raw disparity and ROI back for a proper comparison:
			resize(left_disp, left_disp, Size(), 2.0, 2.0, INTER_LINEAR);
			left_disp = left_disp * 2.0;
			ROI = Rect(ROI.x * 2, ROI.y * 2, ROI.width * 2, ROI.height * 2);
		}
	}
	Mat filtered_disp_vis;
	getDisparityVis(filtered_disp, filtered_disp_vis, 1.0);
//	out = filtered_disp;
	out = filtered_disp_vis.clone();
	out_conf = conf_map.clone();

}
void DenseMapper::GetSubMap(const Mat &img_l, const Mat &img_r, pcl::PointCloud<pcl::PointXYZRGB> &sub_map)
{
	cv::Mat disp, disp_conf;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_map_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_map_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	cv::Mat img_l_rgb, img_r_rgb;
	if (img_l.empty() || img_r.empty()) {
		ROS_WARN_STREAM("DenserMapper: found empty images!" << "timestamp: " << fixed << setprecision(12) << time);
		return;
	}
	img_l_rgb = img_l.clone();
	img_r_rgb = img_r.clone();
	ComputeDisp(img_l, img_r, disp, disp_conf, mParam);
	disp_conf.convertTo(disp_conf, CV_8UC1);

	sensor_msgs::Image::_header_type header;
	header.stamp = ros::Time::now();

	cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, disp);
	mDepthPub->publish(img_bridge.toImageMsg());
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, disp_conf);
	mDepthConfPub->publish(img_bridge.toImageMsg());

//		cv::Mat disp_f;
//		disp.convertTo(disp_f,CV_32F);

	for (int x = 0; x < img_l_rgb.cols; x++) {
		for (int y = 0; y < img_l_rgb.rows; y++) {
			float d = static_cast<float >(disp.at<unsigned char>(y, x));
			float d_f = disp_conf.at<float>(y, x);
			unsigned char b = static_cast<int>(img_l_rgb.at<Vec3b>(y, x)[0]);
			unsigned char g = static_cast<int>(img_l_rgb.at<Vec3b>(y, x)[1]);
			unsigned char r = static_cast<int>(img_l_rgb.at<Vec3b>(y, x)[2]);
//				cout << "disp confidence: " << d_f << endl;

			if (d_f <= 125 || (bf / d) > 10) {
				continue;
			}
			Eigen::Vector3d p_3d(0, 0, 0);
			Project2Dto3D(x, y, d, p_3d);
//				p_3d = T_c0_cj * p_3d;
//				p_3d = T_c0_cj * p_3d;
//				cout << "depth: " << (bf / d) << endl;
			pcl::PointXYZRGB p(r, g, b);
			p.x = p_3d.x();
			p.y = p_3d.y();
			p.z = p_3d.z();

			local_map->push_back(p);
		}
	}
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(local_map);
	sor.setMeanK(100);
	sor.setStddevMulThresh(0.5);
	sor.filter(sub_map);
	sub_map = *local_map;
}
void DenseMapper::MergeSubMap(pcl::PointCloud<pcl::PointXYZRGB> &sub_map, KeyFrame *pKF)
{
	pcl::PointCloud<pcl::PointXYZRGB> transformed_map;

	cv::Mat T_cj_c0_cv = pKF->GetPose().clone();
	Eigen::Isometry3d T_cj_c0, T_c0_cj, T_b_c, T_b0_cj;
	cv::cv2eigen(T_cj_c0_cv, T_cj_c0.matrix());
	T_c0_cj = T_cj_c0.inverse();
	cv::Mat T_b_c_cv = pKF->mImuCalib.mT_dvl_c.clone();
	cv::cv2eigen(T_b_c_cv, T_b_c.matrix());

	T_b0_cj = T_b_c * T_c0_cj;

	pcl::transformPointCloud(sub_map, transformed_map, T_b0_cj.matrix());

	std::lock_guard<std::mutex> lock(mDenseMapMutex);
	mGlobalMap += transformed_map;

}
void DenseMapper::Run()
{
	while (1) {
		{
			std::lock_guard<std::mutex> lock(mKFQueueMutex);
			if (!mKFQueue.empty()) {
				auto pKF = mKFQueue.front();
				mKFQueue.pop();
				if (pKF->isBad()) {
					continue;
				}
				if (pKF->GetPose().empty()) {
					continue;
				}
				if (mKFWithPointCloud.find(pKF) == mKFWithPointCloud.end()) {
					std::lock_guard<std::mutex> lock(mKFMutex);
					mKFWithPointCloud[pKF] = pcl::PointCloud<pcl::PointXYZRGB>();
					GetSubMap(pKF->imgLeft, pKF->imgRight, mKFWithPointCloud[pKF]);
					MergeSubMap(mKFWithPointCloud[pKF], pKF);
				}
			}
		}


		if (mStop) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
}
void DenseMapper::Stop()
{
	mStop = true;
}
void DenseMapper::Resume()
{
	mStop = false;
}
void DenseMapper::Update()
{
	mGlobalMap.clear();
	{
		std::lock_guard<std::mutex> lock(mKFMutex);
		for (auto kf_pointcloud: mKFWithPointCloud) {
			if (kf_pointcloud.first->isBad()){
				ROS_INFO_STREAM("DenserMapper: find bad KF, remove and skip");
				mKFWithPointCloud.erase(kf_pointcloud.first);
				return;
			}
			MergeSubMap(kf_pointcloud.second, kf_pointcloud.first);
		}
	}
	PublishMap();
}
void DenseMapper::PublishMap()
{
	sensor_msgs::PointCloud2 dense_map;

	std::lock_guard<std::mutex> lock(mDenseMapMutex);
	pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(mGlobalMap));
	sor.setLeafSize(0.05f, 0.05f, 0.05f);
	sor.filter(filtered_cloud);
	pcl::toROSMsg(filtered_cloud, dense_map);
	dense_map.header.frame_id = "orb_slam";
	mMapPub->publish(dense_map);
//	mMapPub->publish(mGlobalMap);
}
void DenseMapper::Save(string path)
{
	std::lock_guard<std::mutex> lock(mDenseMapMutex);
	string file_name = path+"KF_map.pcd";
	pcl::io::savePCDFileBinary(file_name,mGlobalMap);
}

}
