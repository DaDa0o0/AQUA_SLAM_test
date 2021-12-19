#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>


// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
// Eigen
#include <Eigen/Dense>

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

// store all feature points are being tracked
vector<int> ids, ids_right;

// the number of each feature points has been tracked
vector<int> track_cnt;

//display track image or not
bool SHOW_TRACK = true;

//stereo mode or mono cmode
bool stereo_cam = true;

//the maximun number of feature points allowed to be tracked per image frame
const int MAX_CNT = 150;

// the closest distance between two feature points allowed to be extracted
const int MIN_DIST = 30;

// col and row of image
int row, col;

cv::Mat mask;

cv::Mat imTrack;

vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts, predict_pts, n_pts;

bool FLOW_BACK = true;

int n_id = 0;

double cur_time, prev_time;

cv::Mat prev_img, cur_img;

vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;

map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map, prevLeftPtsMap;

map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;

vector<cv::Point2f> pts_velocity, right_pts_velocity;

boost::shared_ptr<image_transport::Publisher> pTrack_img_pub;

vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
								map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
	vector<cv::Point2f> pts_velocity;
	cur_id_pts.clear();
	for (unsigned int i = 0; i < ids.size(); i++) {
		cur_id_pts.insert(make_pair(ids[i], pts[i]));
	}

	// caculate points velocity
	if (!prev_id_pts.empty()) {
		double dt = cur_time - prev_time;

		for (unsigned int i = 0; i < pts.size(); i++) {
			std::map<int, cv::Point2f>::iterator it;
			it = prev_id_pts.find(ids[i]);
			if (it != prev_id_pts.end()) {
				double v_x = (pts[i].x - it->second.x) / dt;
				double v_y = (pts[i].y - it->second.y) / dt;
				pts_velocity.push_back(cv::Point2f(v_x, v_y));
			}
			else {
				pts_velocity.push_back(cv::Point2f(0, 0));
			}

		}
	}
	else {
		for (unsigned int i = 0; i < cur_pts.size(); i++) {
			pts_velocity.push_back(cv::Point2f(0, 0));
		}
	}
	return pts_velocity;
}

double distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
	//printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	return sqrt(dx * dx + dy * dy);
}

void setMask()
{
	mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

	// prefer to keep features that are tracked for long time
	vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

	for (unsigned int i = 0; i < cur_pts.size(); i++)
		cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

	sort(cnt_pts_id.begin(),
		 cnt_pts_id.end(),
		 [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
		 {
			 return a.first > b.first;
		 });

	cur_pts.clear();
	ids.clear();
	track_cnt.clear();

	for (auto &it: cnt_pts_id) {
		if (mask.at<uchar>(it.second.first) == 255) {
			cur_pts.push_back(it.second.first);
			ids.push_back(it.second.second);
			track_cnt.push_back(it.first);
			cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
		}
	}
}

bool inBorder(const cv::Point2f &pt, int col, int row)
{
	const int BORDER_SIZE = 1;
	int img_x = cvRound(pt.x);
	int img_y = cvRound(pt.y);
	return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
	int j = 0;
	for (int i = 0; i < int(v.size()); i++)
		if (status[i]) {
			v[j++] = v[i];
		}
	v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
	int j = 0;
	for (int i = 0; i < int(v.size()); i++)
		if (status[i]) {
			v[j++] = v[i];
		}
	v.resize(j);
}

void rejectWithF()
{
//	if (cur_pts.size() >= 8)
//	{
//		ROS_DEBUG("FM ransac begins");
//		vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
//		for (unsigned int i = 0; i < cur_pts.size(); i++)
//		{
//			Eigen::Vector3d tmp_p;
//			m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
//			tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
//			tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
//			un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
//
//			m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
//			tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
//			tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
//			un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
//		}
//
//		vector<uchar> status;
//		cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
//		int size_a = cur_pts.size();
//		reduceVector(prev_pts, status);
//		reduceVector(cur_pts, status);
//		reduceVector(cur_un_pts, status);
//		reduceVector(ids, status);
//		reduceVector(track_cnt, status);
//		ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
//		ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
//	}
}

void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
			   vector<int> &curLeftIds,
			   vector<cv::Point2f> &curLeftPts,
			   vector<cv::Point2f> &curRightPts,
			   map<int, cv::Point2f> &prevLeftPtsMap)
{
	//int rows = imLeft.rows;
	int cols = imLeft.cols;
	if (!imRight.empty() && stereo_cam) {
		cv::hconcat(imLeft, imRight, imTrack);
	}
	else {
		imTrack = imLeft.clone();
	}
	cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

	for (size_t j = 0; j < curLeftPts.size(); j++) {
		double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
		cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
	}
	if (!imRight.empty() && stereo_cam) {
		for (size_t i = 0; i < curRightPts.size(); i++) {
			cv::Point2f rightPt = curRightPts[i];
			rightPt.x += cols;
			cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
			//cv::Point2f leftPt = curLeftPtsTrackRight[i];
			//cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
		}
	}

	map<int, cv::Point2f>::iterator mapIt;
	for (size_t i = 0; i < curLeftIds.size(); i++) {
		int id = curLeftIds[i];
		mapIt = prevLeftPtsMap.find(id);
		if (mapIt != prevLeftPtsMap.end()) {
			cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
		}
	}

	//draw prediction
	/*
	for(size_t i = 0; i < predict_pts_debug.size(); i++)
	{
		cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
	}
	*/
	//printf("predict pts size %d \n", (int)predict_pts_debug.size());

	//cv::Mat imCur2Compress;
	//cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time,
																	const cv::Mat &_img,
																	const cv::Mat &_img1)
{
//	TicToc t_r;
	bool hasPrediction = false;
	cur_time = _cur_time;
	cur_img = _img;
	row = cur_img.rows;
	col = cur_img.cols;
	cv::Mat rightImg = _img1;

	/*
	{
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
		clahe->apply(cur_img, cur_img);
		if(!rightImg.empty())
			clahe->apply(rightImg, rightImg);
	}
	*/
	cur_pts.clear();

	if (prev_pts.size() > 0) {
		vector<uchar> status;
		vector<float> err;
		if (hasPrediction) {
			cur_pts = predict_pts;
			cv::calcOpticalFlowPyrLK(prev_img,
									 cur_img,
									 prev_pts,
									 cur_pts,
									 status,
									 err,
									 cv::Size(21, 21),
									 1,
									 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
									 cv::OPTFLOW_USE_INITIAL_FLOW);

			int succ_num = 0;
			for (size_t i = 0; i < status.size(); i++) {
				if (status[i]) {
					succ_num++;
				}
			}
			if (succ_num < 10) {
				cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
			}
		}
		else {
			cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
		}
		// reverse check
		if (FLOW_BACK) {
			vector<uchar> reverse_status;
			vector<cv::Point2f> reverse_pts = prev_pts;
			cv::calcOpticalFlowPyrLK(cur_img,
									 prev_img,
									 cur_pts,
									 reverse_pts,
									 reverse_status,
									 err,
									 cv::Size(21, 21),
									 1,
									 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
									 cv::OPTFLOW_USE_INITIAL_FLOW);
			//cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
			for (size_t i = 0; i < status.size(); i++) {
				if (status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
					status[i] = 1;
				}
				else {
					status[i] = 0;
				}
			}
		}

		for (int i = 0; i < int(cur_pts.size()); i++)
			if (status[i] && !inBorder(cur_pts[i], col, row)) {
				status[i] = 0;
			}
		reduceVector(prev_pts, status);
		reduceVector(cur_pts, status);
		reduceVector(ids, status);
		reduceVector(track_cnt, status);
//		ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
		//printf("track cnt %d\n", (int)ids.size());
	}

	for (auto &n: track_cnt)
		n++;

	if (1) {
		//rejectWithF();
		ROS_DEBUG("set mask begins");
		setMask();

		ROS_DEBUG("detect feature begins");
		int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
		if (n_max_cnt > 0) {
			if (mask.empty()) {
				cout << "mask is empty " << endl;
			}
			if (mask.type() != CV_8UC1) {
				cout << "mask type wrong " << endl;
			}
			cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
		}
		else {
			n_pts.clear();
		}

		for (auto &p: n_pts) {
			cur_pts.push_back(p);
			ids.push_back(n_id++);
			track_cnt.push_back(1);
		}
		//printf("feature cnt after add %d\n", (int)ids.size());
	}

	cur_un_pts = cur_pts;
	pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

	if (!_img1.empty() && stereo_cam) {
		ids_right.clear();
		cur_right_pts.clear();
		cur_un_right_pts.clear();
		right_pts_velocity.clear();
		cur_un_right_pts_map.clear();
		if (!cur_pts.empty()) {
			//printf("stereo image; track feature on right image\n");
			vector<cv::Point2f> reverseLeftPts;
			vector<uchar> status, statusRightLeft;
			vector<float> err;
			// cur left ---- cur right
			cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
			// reverse check cur right ---- cur left
			if (FLOW_BACK) {
				cv::calcOpticalFlowPyrLK(rightImg,
										 cur_img,
										 cur_right_pts,
										 reverseLeftPts,
										 statusRightLeft,
										 err,
										 cv::Size(21, 21),
										 3);
				for (size_t i = 0; i < status.size(); i++) {
					if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i], col, row)
						&& distance(cur_pts[i], reverseLeftPts[i]) <= 0.5) {
						status[i] = 1;
					}
					else {
						status[i] = 0;
					}
				}
			}

			ids_right = ids;
			reduceVector(cur_right_pts, status);
			reduceVector(ids_right, status);
			// only keep left-right pts
			/*
			reduceVector(cur_pts, status);
			reduceVector(ids, status);
			reduceVector(track_cnt, status);
			reduceVector(cur_un_pts, status);
			reduceVector(pts_velocity, status);
			*/
			cur_un_right_pts = cur_right_pts;
			right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
		}
		prev_un_right_pts_map = cur_un_right_pts_map;
	}
	if (SHOW_TRACK) {
		drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
	}

	prev_img = cur_img;
	prev_pts = cur_pts;
	prev_un_pts = cur_un_pts;
	prev_un_pts_map = cur_un_pts_map;
	prev_time = cur_time;
	hasPrediction = false;

	prevLeftPtsMap.clear();
	for (size_t i = 0; i < cur_pts.size(); i++)
		prevLeftPtsMap[ids[i]] = cur_pts[i];

	map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
	for (size_t i = 0; i < ids.size(); i++) {
		int feature_id = ids[i];
		double x, y, z;
		x = cur_un_pts[i].x;
		y = cur_un_pts[i].y;
		z = 1;
		double p_u, p_v;
		p_u = cur_pts[i].x;
		p_v = cur_pts[i].y;
		int camera_id = 0;
		double velocity_x, velocity_y;
		velocity_x = pts_velocity[i].x;
		velocity_y = pts_velocity[i].y;

		Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
		xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
		featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
	}

	if (!_img1.empty() && stereo_cam) {
		for (size_t i = 0; i < ids_right.size(); i++) {
			int feature_id = ids_right[i];
			double x, y, z;
			x = cur_un_right_pts[i].x;
			y = cur_un_right_pts[i].y;
			z = 1;
			double p_u, p_v;
			p_u = cur_right_pts[i].x;
			p_v = cur_right_pts[i].y;
			int camera_id = 1;
			double velocity_x, velocity_y;
			velocity_x = right_pts_velocity[i].x;
			velocity_y = right_pts_velocity[i].y;

			Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
			xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
			featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
		}
	}

	//printf("feature track whole time %f\n", t_r.toc());
	return featureFrame;
}

void img_callback(const sensor_msgs::ImageConstPtr &left, const sensor_msgs::ImageConstPtr &right)
{
	cv_bridge::CvImagePtr cv_ptr_l;
	cv_bridge::CvImagePtr cv_ptr_r;
	try {
		cv_ptr_l = cv_bridge::toCvCopy(left, "bgr8");
		cv_ptr_r = cv_bridge::toCvCopy(right, "bgr8");
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	double cur_img_time = left->header.stamp.toSec();

	cv::Mat gray_img_l, gray_img_r;
	cv::cvtColor(cv_ptr_l->image, gray_img_l, cv::COLOR_BGR2GRAY);
	cv::cvtColor(cv_ptr_r->image, gray_img_r, cv::COLOR_BGR2GRAY);
	trackImage(cur_img_time, gray_img_l, gray_img_r);
	std_msgs::Header header = left->header; // empty header
	cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, imTrack);
	pTrack_img_pub->publish(img_bridge.toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "LKTrcker");
	ros::NodeHandle n("~");
	image_transport::ImageTransport it(n);
//	ros::Publisher track_img_pub = n.advertise<sensor_msgs::Image>("/lk_tracker/track_img", 10);

	image_transport::Publisher track_pub = it.advertise("/lk_tracker/track_img", 10);
	pTrack_img_pub =
		boost::shared_ptr<image_transport::Publisher>(boost::make_shared<image_transport::Publisher>(track_pub));

	image_transport::TransportHints hints("compressed");
	image_transport::SubscriberFilter img_l_sub(it, "/img_l", 50, hints);
	image_transport::SubscriberFilter img_r_sub(it, "/img_r", 50, hints);
	typedef sync_policies::ApproximateTime<Image, Image> Img_sync;
	Synchronizer<Img_sync> img_sync(Img_sync(50), img_l_sub, img_r_sub);
	img_sync.registerCallback(boost::bind(&img_callback, _1, _2));
	ros::spin();
	return 0;
}