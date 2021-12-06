//
// Created by da on 25/01/2021.
//

#include "DVLGroPreIntegration.h"
#include<iostream>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

using namespace std;

DVLGroPreIntegration::DVLGroPreIntegration(const Bias &b_, const Calib &calib, bool bDVL)
	:
	bDVL(bDVL)
{
	Nga = calib.Cov.clone();
	NgaWalk = calib.CovWalk.clone();
	mVelocityThreshold = 0.6f;
	Initialize(b_);
	calib.mT_gyro_dvl.rowRange(0,3).colRange(0,3).copyTo(mR_g_d);
}

DVLGroPreIntegration::DVLGroPreIntegration(const Bias &b_, const Calib &calib, const cv::Point3f &v_di, bool bDVL)
	:
	bDVL(bDVL)
{
	calib.mT_gyro_dvl.rowRange(0,3).colRange(0,3).copyTo(mR_g_d);
	Nga = calib.Cov.clone();
	NgaWalk = calib.CovWalk.clone();
	Initialize(b_);
	SetVelocity(v_di);
	mVelocityThreshold = 0.6f;
}


DVLGroPreIntegration::DVLGroPreIntegration(const Bias &b_, const Calib &calib, const cv::Point3f &v_di, float velocity_threshold, bool bDVL )
:
bDVL(bDVL),mVelocityThreshold(velocity_threshold)
{
	calib.mT_gyro_dvl.rowRange(0,3).colRange(0,3).copyTo(mR_g_d);
	Nga = calib.Cov.clone();
	NgaWalk = calib.CovWalk.clone();
	Initialize(b_);
	SetVelocity(v_di);

}

// Copy constructor
DVLGroPreIntegration::DVLGroPreIntegration(DVLGroPreIntegration *pDVLPre)
	: dT(pDVLPre->dT), C(pDVLPre->C.clone()), Info(pDVLPre->Info.clone()),
	  Nga(pDVLPre->Nga.clone()), NgaWalk(pDVLPre->NgaWalk.clone()), mb(pDVLPre->mb), dR(pDVLPre->dR.clone()),
	  dV(pDVLPre->dV.clone()),
	  dP(pDVLPre->dP.clone()), JRg(pDVLPre->JRg.clone()), JVg(pDVLPre->JVg.clone()), JVa(pDVLPre->JVa.clone()),
	  JPg(pDVLPre->JPg.clone()),
	  JPa(pDVLPre->JPa.clone()), avgA(pDVLPre->avgA.clone()), avgW(pDVLPre->avgW.clone()), bu(pDVLPre->bu),
	  db(pDVLPre->db.clone()), mvMeasurements(pDVLPre->mvMeasurements),
	  bDVL(pDVLPre->bDVL)
{

}

void DVLGroPreIntegration::CopyFrom(DVLGroPreIntegration *pDVLPre)
{
	std::cout << "Preintegrated: start clone" << std::endl;
	bDVL = pDVLPre->bDVL;
	dT = pDVLPre->dT;
	C = pDVLPre->C.clone();
	Info = pDVLPre->Info.clone();
	Nga = pDVLPre->Nga.clone();
	NgaWalk = pDVLPre->NgaWalk.clone();
	std::cout << "Preintegrated: first clone" << std::endl;
	mb.CopyFrom(pDVLPre->mb);
	dR = pDVLPre->dR.clone();
	dV = pDVLPre->dV.clone();
	dP = pDVLPre->dP.clone();
	JRg = pDVLPre->JRg.clone();
	JVg = pDVLPre->JVg.clone();
	JVa = pDVLPre->JVa.clone();
	JPg = pDVLPre->JPg.clone();
	JPa = pDVLPre->JPa.clone();
	avgA = pDVLPre->avgA.clone();
	avgW = pDVLPre->avgW.clone();
	std::cout << "Preintegrated: second clone" << std::endl;
	bu.CopyFrom(pDVLPre->bu);
	db = pDVLPre->db.clone();
	std::cout << "Preintegrated: third clone" << std::endl;
	mvMeasurements = pDVLPre->mvMeasurements;
	std::cout << "Preintegrated: end clone" << std::endl;
}

void DVLGroPreIntegration::Initialize(const Bias &b_)
{
	dR = cv::Mat::eye(3, 3, CV_32F);
	dV = cv::Mat::zeros(3, 1, CV_32F);
	mVelocity = cv::Mat::zeros(3, 1, CV_32F);
//	mR_g_d = cv::Mat::eye(3, 3, CV_32F);
	dP = cv::Mat::zeros(3, 1, CV_32F);
	JRg = cv::Mat::zeros(3, 3, CV_32F);
	JVg = cv::Mat::zeros(3, 3, CV_32F);
	JVa = cv::Mat::zeros(3, 3, CV_32F);
	JPg = cv::Mat::zeros(3, 3, CV_32F);
	JPa = cv::Mat::zeros(3, 3, CV_32F);
	C = cv::Mat::zeros(15, 15, CV_32F);
	Info = cv::Mat();
	db = cv::Mat::zeros(6, 1, CV_32F);
	mb = b_;
	bu = b_;
	avgA = cv::Mat::zeros(3, 1, CV_32F);
	avgW = cv::Mat::zeros(3, 1, CV_32F);
	dT = 0.0f;
	mvMeasurements.clear();
}

void DVLGroPreIntegration::ReintegrateWithVelocity()
{

//	std::unique_lock<std::mutex> lock(mMutex);
	if (mMutex.try_lock()) {
		const std::vector<integrable> aux = mvMeasurements;
		cv::Mat v = dV.clone();
		Initialize(bu);
		//todo_tightly
		// save mVelocity
		SetVelocity(cv::Point3f(v.at<float>(0), v.at<float>(1), v.at<float>(2)));
		for (size_t i = 0; i < aux.size(); i++)
//		IntegrateNewMeasurement(aux[i].a, aux[i].w, aux[i].t);
			IntegrateGroMeasurement(aux[i].w, aux[i].t);
		mMutex.unlock();
	}
	else {
		const std::vector<integrable> aux = mvMeasurements;
		cv::Mat v = dV.clone();
		Initialize(bu);
		SetVelocity(cv::Point3f(v.at<float>(0), v.at<float>(1), v.at<float>(2)));
//		dV = v.clone();
		for (size_t i = 0; i < aux.size(); i++)
//		IntegrateNewMeasurement(aux[i].a, aux[i].w, aux[i].t);
			IntegrateGroMeasurement(aux[i].w, aux[i].t);
	}

}
void DVLGroPreIntegration::ReintegrateWithBiasAndRotation(const Bias &b, const cv::Mat &R_g_d)
{
	std::lock_guard<std::mutex> lock(mMutex);
	const std::vector<integrable> aux = mvMeasurements;
	cv::Mat v=dV.clone();
	Initialize(bu);
//	dV=v.clone();
	SetVelocity(cv::Point3f(v.at<float>(0), v.at<float>(1), v.at<float>(2)));
	mb = b;
	mR_g_d = R_g_d.clone();
	for (size_t i = 0; i < aux.size(); i++)
	{
		if (aux[i].a.x == 0 && aux[i].a.y == 0 && aux[i].a.z == 0)
			IntegrateGroMeasurement(aux[i].w,aux[i].t);
		else if (aux[i].w.x == 0 && aux[i].w.y == 0 && aux[i].w.z == 0)
			IntegrateDVLMeasurement(aux[i].a);
		else
			cout<<"found error in measurement"<<endl;
	}

}

void DVLGroPreIntegration::Reintegrate()
{
	std::unique_lock<std::mutex> lock(mMutex);
	const std::vector<integrable> aux = mvMeasurements;
//	cv::Mat v=dV.clone();
	Initialize(bu);
//	dV=v.clone();
	for (size_t i = 0; i < aux.size(); i++)
		IntegrateNewMeasurement(aux[i].a, aux[i].w, aux[i].t);
//		IntegrateGroMeasurement(aux[i].w,aux[i].t);
}

void DVLGroPreIntegration::IntegrateNewMeasurement(const cv::Point3f &acceleration,
												   const cv::Point3f &angVel,
												   const float &dt)
{
	mvMeasurements.push_back(integrable(acceleration, angVel, dt));

	cv::Mat accW = (cv::Mat_<float>(3, 1) << angVel.x - mb.bwx, angVel.y - mb.bwy, angVel.z - mb.bwz);

	//3*1
	cv::Mat_<float> v(3, 1);
	v << acceleration.x, acceleration.y, acceleration.z;
	dV = v.clone();


	// Total integrated time
	dT += dt;

	dP = dP + dR * dV * dt;
//	if (dP.at<float>(0,0)!=0)
//		cout<<"dP: "<<dP<<endl;

	// Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
	cv::Mat v_k_hat = (cv::Mat_<float>(3, 3) << 0, -dV.at<float>(2), dV.at<float>(1),
		dV.at<float>(2), 0, -dV.at<float>(0),
		-dV.at<float>(1), dV.at<float>(0), 0);

	// Update position and velocity jacobians wrt bias correction
	JPg = JPg - dR * dt * v_k_hat * JRg;



	// Update delta rotation
	IntegratedRotation dRi(angVel, mb, dt);
	dR = NormalizeRotation(dR * dRi.deltaR);


	// Update rotation jacobian wrt bias correction
	JRg = dRi.deltaR.t() * JRg - dRi.rightJ * dt;
}

void DVLGroPreIntegration::IntegrateNewMeasurementWithBiasAndRotation(const cv::Point3f &acceleration,
																	  const cv::Point3f &angVel,
																	  const float &dt,
																	  const Bias &b,
																	  const cv::Mat &R_g_d)
{
//	mb=b;
//	mR_g_d=R_g_d.clone();
	if (angVel.x != 0 || angVel.y != 0 || angVel.z != 0) {
		IntegrateGroMeasurement(angVel, dt);
	}


	if (acceleration.x != 0 || acceleration.y != 0 || acceleration.z != 0) {
		IntegrateDVLMeasurement(acceleration);
	}

}

void DVLGroPreIntegration::IntegrateGroMeasurement(const cv::Point3f &angVel,
												   const float &dt)
{
	mvMeasurements.push_back(integrable(cv::Point3f(0, 0, 0), angVel, dt));

	cv::Mat accW = (cv::Mat_<float>(3, 1) << angVel.x - mb.bwx, angVel.y - mb.bwy, angVel.z - mb.bwz);

	//3*1
//	avgW = (dT * avgW + accW * dt) / (dT + dt);

	// Total integrated time
	dT += dt;

	// assume V_di constant
//	dP = dP + dV*dt;

	// assume V_dk constant
	//R_d_g * R_gi_gk * R_g_d * V_dk
	dV = mR_g_d.t() * dR * mR_g_d * mVelocity;
//	dV =  dR * mVelocity;
	dP = dP + dV * dt;
//	if (dP.at<float>(0,0)!=0)
//		cout<<"dP: "<<dP<<endl;

	// Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
	cv::Mat v_k_hat = (cv::Mat_<float>(3, 3) << 0, -dV.at<float>(2), dV.at<float>(1),
		dV.at<float>(2), 0, -dV.at<float>(0),
		-dV.at<float>(1), dV.at<float>(0), 0);

	//todo_tightly
	//	not sure JPg is correct or not
	//	add velocity jacobians
	//  Update position and velocity jacobians wrt bias correction
	JPg = JPg - dR * dt * v_k_hat * JRg;



	// Update delta rotation
	IntegratedRotation dRi(angVel, mb, dt);
	dR = NormalizeRotation(dR * dRi.deltaR);


	// Update rotation jacobian wrt bias correction
	JRg = dRi.deltaR.t() * JRg - dRi.rightJ * dt;

}

void DVLGroPreIntegration::IntegrateDVLMeasurement(const cv::Point3f &velocity)
{
	// V_dk
	cv::Mat_<float> v(3, 1);
	v << velocity.x, velocity.y, velocity.z;
	if (cv::norm(v)>(double)mVelocityThreshold)
		return;
	mVelocity = v;
	bDVL = true;
//	ReintegrateWithVelocity();
	if (dV.at<float>(0) == 0 && dV.at<float>(1) == 0 && dV.at<float>(2) == 0) {
//		dV = dR * mR_g_d* v;
//		dV = dR * v;
		dV = mR_g_d.t() * dR * mR_g_d * v;
//		cout<<"dR:"<<dR<<endl;
//		cout<<"dV:"<<dV<<endl;
		ReintegrateWithVelocity();
	}
	else {
//		dV = dR * v;
		dV = mR_g_d.t() * dR * mR_g_d * v;
	}

	mvMeasurements.push_back(integrable(velocity, cv::Point3f(0, 0, 0), 0));
}

void DVLGroPreIntegration::SetVelocity(const cv::Point3f &v_di)
{
	// V_di
	cv::Mat_<float> v(3, 1);
	v << v_di.x, v_di.y, v_di.z;



//		dV = dR * v;
	dV = v;
	// R_d_g * R_gj_gi * R_g_d = R_dj_di
	mVelocity = mR_g_d.t() * dR.t() * mR_g_d *v;
	cv::Point3f v_dj(mVelocity.at<float>(0),mVelocity.at<float>(1),mVelocity.at<float>(2));

	mvMeasurements.push_back(integrable(v_dj, cv::Point3f(0, 0, 0), 0));
}


void DVLGroPreIntegration::MergePrevious(DVLGroPreIntegration *pPrev)
{
	if (pPrev == this) {
		return;
	}

	std::unique_lock<std::mutex> lock1(mMutex);
	std::unique_lock<std::mutex> lock2(pPrev->mMutex);
	Bias bav;
	bav.bwx = bu.bwx;
	bav.bwy = bu.bwy;
	bav.bwz = bu.bwz;
	bav.bax = bu.bax;
	bav.bay = bu.bay;
	bav.baz = bu.baz;

	const std::vector<integrable> aux1 = pPrev->mvMeasurements;
	const std::vector<integrable> aux2 = mvMeasurements;

	Initialize(bav);
	for (size_t i = 0; i < aux1.size(); i++)
	{
		if (aux1[i].a.x == 0 && aux1[i].a.y == 0 && aux1[i].a.z == 0)
			IntegrateGroMeasurement(aux1[i].w,aux1[i].t);
		else if (aux1[i].w.x == 0 && aux1[i].w.y == 0 && aux1[i].w.z == 0)
			IntegrateDVLMeasurement(aux1[i].a);
		else
			cout<<"found error in measurement"<<endl;
	}
	for (size_t i = 0; i < aux2.size(); i++)
	{
		if (aux2[i].a.x == 0 && aux2[i].a.y == 0 && aux2[i].a.z == 0)
			IntegrateGroMeasurement(aux2[i].w,aux2[i].t);
		else if (aux2[i].w.x == 0 && aux2[i].w.y == 0 && aux2[i].w.z == 0)
			IntegrateDVLMeasurement(aux2[i].a);
		else
			cout<<"found error in measurement"<<endl;
	}

}

void DVLGroPreIntegration::SetNewBias(const Bias &bu_)
{
	std::unique_lock<std::mutex> lock(mMutex);
	bu = bu_;

	db.at<float>(0) = bu_.bwx - mb.bwx;
	db.at<float>(1) = bu_.bwy - mb.bwy;
	db.at<float>(2) = bu_.bwz - mb.bwz;
	db.at<float>(3) = bu_.bax - mb.bax;
	db.at<float>(4) = bu_.bay - mb.bay;
	db.at<float>(5) = bu_.baz - mb.baz;
}

IMU::Bias DVLGroPreIntegration::GetDeltaBias(const Bias &b_)
{
	std::unique_lock<std::mutex> lock(mMutex);
	return IMU::Bias(b_.bax - mb.bax,
					 b_.bay - mb.bay,
					 b_.baz - mb.baz,
					 b_.bwx - mb.bwx,
					 b_.bwy - mb.bwy,
					 b_.bwz - mb.bwz);
}

cv::Mat DVLGroPreIntegration::GetDeltaRotation(const Bias &b_)
{
	std::unique_lock<std::mutex> lock(mMutex);
	cv::Mat dbg = (cv::Mat_<float>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
	return NormalizeRotation(dR * ExpSO3(JRg * dbg));
}

cv::Mat DVLGroPreIntegration::GetDeltaRotation(const Bias &b, const cv::Mat &R_g_d)
{
//	std::unique_lock<std::mutex> lock(mMutex);
	ReintegrateWithBiasAndRotation(b, R_g_d);
	return NormalizeRotation(dR);
}

cv::Mat DVLGroPreIntegration::GetDeltaVelocity(const Bias &b_)
{
	std::unique_lock<std::mutex> lock(mMutex);
	cv::Mat dbg = (cv::Mat_<float>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
	cv::Mat dba = (cv::Mat_<float>(3, 1) << b_.bax - mb.bax, b_.bay - mb.bay, b_.baz - mb.baz);
	return dV + JVg * dbg + JVa * dba;
}

cv::Mat DVLGroPreIntegration::GetDeltaPosition(const Bias &b_)
{
	std::unique_lock<std::mutex> lock(mMutex);
	cv::Mat dbg = (cv::Mat_<float>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
	cv::Mat dba = (cv::Mat_<float>(3, 1) << b_.bax - mb.bax, b_.bay - mb.bay, b_.baz - mb.baz);
	return dP + JPg * dbg;
}
cv::Mat DVLGroPreIntegration::GetDeltaPosition(const Bias &b, const cv::Mat &R_g_d)
{
//	std::unique_lock<std::mutex> lock(mMutex);
	ReintegrateWithBiasAndRotation(b, R_g_d);
	return dP;
}

cv::Mat DVLGroPreIntegration::GetUpdatedDeltaRotation()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return NormalizeRotation(dR * ExpSO3(JRg * db.rowRange(0, 3)));
}

cv::Mat DVLGroPreIntegration::GetUpdatedDeltaVelocity()
{
	//todo_tightly
	//	complete update of JVg
	//  not used currently, maybe for further use
	std::unique_lock<std::mutex> lock(mMutex);
	return dV + JVg * db.rowRange(0, 3) + JVa * db.rowRange(3, 6);
}

cv::Mat DVLGroPreIntegration::GetUpdatedDeltaPosition()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return dP + JPg * db.rowRange(0, 3) + JPa * db.rowRange(3, 6);
}

cv::Mat DVLGroPreIntegration::GetOriginalDeltaRotation()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return dR.clone();
}

cv::Mat DVLGroPreIntegration::GetOriginalDeltaVelocity()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return dV.clone();
}

cv::Mat DVLGroPreIntegration::GetOriginalDeltaPosition()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return dP.clone();
}

Bias DVLGroPreIntegration::GetOriginalBias()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return mb;
}

Bias DVLGroPreIntegration::GetUpdatedBias()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return bu;
}

cv::Mat DVLGroPreIntegration::GetDeltaBias()
{
	std::unique_lock<std::mutex> lock(mMutex);
	return db.clone();
}

Eigen::Matrix<double, 15, 15> DVLGroPreIntegration::GetInformationMatrix()
{
	std::unique_lock<std::mutex> lock(mMutex);
	if (Info.empty()) {
		Info = cv::Mat::zeros(15, 15, CV_32F);
		Info.rowRange(0, 9).colRange(0, 9) = C.rowRange(0, 9).colRange(0, 9).inv(cv::DECOMP_SVD);
		for (int i = 9; i < 15; i++)
			Info.at<float>(i, i) = 1.0f / C.at<float>(i, i);
	}

	Eigen::Matrix<double, 15, 15> EI;
	for (int i = 0; i < 15; i++)
		for (int j = 0; j < 15; j++)
			EI(i, j) = Info.at<float>(i, j);
	return EI;
}
void DVLGroPreIntegration::output()
{
	Eigen::Matrix3d R_gi_gj;
	cv::cv2eigen(dR,R_gi_gj);
	Eigen::Quaterniond q(R_gi_gj);
//	ROS_INFO_STREAM("integrated oritentation: x:"<<q.x()<<" y:"<<q.y()<<" z:"<<q.z()<<" w:"<<q.w());
}

Eigen::Isometry3d DVLGroPreIntegration::getDVLPose()
{
	cv::Mat R_gi_gj, R_di_dj, t_di_didj;
	dR.copyTo(R_gi_gj);
	dP.copyTo(t_di_didj);
	R_di_dj = mR_g_d.t() * R_gi_gj * mR_g_d;
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	cv::cv2eigen(R_di_dj,R);
	cv::cv2eigen(t_di_didj,t);

	Eigen::Isometry3d T_di_dj = Eigen::Isometry3d::Identity();
	T_di_dj.pretranslate(t);
	T_di_dj.rotate(R);
	return T_di_dj;
}
