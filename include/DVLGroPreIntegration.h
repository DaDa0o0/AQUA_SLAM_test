//
// Created by da on 25/01/2021.
//

#ifndef DVLGROPREINTEGRATION_H
#define DVLGROPREINTEGRATION_H

#include <ImuTypes.h>

using namespace ORB_SLAM3;
using namespace IMU;

class DVLGroPreIntegration
{
public:
	DVLGroPreIntegration(const Bias &b_, const Calib &calib, bool bDVL = false);
	DVLGroPreIntegration(const Bias &b_, const Calib &calib, const cv::Point3d &v_di, bool bDVL = false);
	DVLGroPreIntegration(const Bias &b_,
	                     const Calib &calib,
	                     const cv::Point3d &v_di,
	                     const Eigen::Vector4d &alpha,
	                     const Eigen::Vector4d &beta,
	                     bool bDVL = false);
	DVLGroPreIntegration(const Bias &b_,
	                     const Calib &calib,
	                     const cv::Point3d &v_di,
	                     double velocity_threshold,
	                     bool bDVL = false);
	DVLGroPreIntegration(DVLGroPreIntegration *pDVLPre);
	DVLGroPreIntegration()
	{}
	~DVLGroPreIntegration()
	{}
	void Initialize(const Bias &b_);



	//  first Integrate Gro, then DVL
	void IntegrateGroMeasurement(const cv::Point3d &angVel, const double &dt);

//	void IntegrateGroMeasurement2(const cv::Point3d &angVel, const double &dt);

	/*!
	 *
	 * @param v_dk: V_dk
	 */
	void IntegrateDVLMeasurement(const cv::Point3d &v_dk, const double &dt);

	/***
	 * @param v_di
	 * @param dt
	 */
	void IntegrateVelocity(const cv::Point3d &v_di, const double &dt);

	/***
	 *
	 * @param velocity: velocity of 4 beam
	 */
	void IntegrateDVLMeasurement2(const Eigen::Vector4d &velocity_beam, const double &dt);
	/*!
	 *
	 * @param v_di: V_di
	 */
	void SetVelocity(const cv::Point3d &v_di);
	void SetBeamOrientation(const Eigen::Vector4d &alpha, const Eigen::Vector4d &beta);
	void ReintegrateWithVelocity();
	void ReintegrateWithVelocity(const Eigen::Vector3d &velocity);
	void ReintegrateWithBiasAndRotation(const Bias &b, const cv::Mat &R_g_d);
	void ReintegrateWithBiasRotationBeamOri(const Bias &b, const cv::Mat &R_g_d, const Eigen::Vector4d &alpha, const Eigen::Vector4d &beta);
	void MergePrevious(DVLGroPreIntegration *pPrev);
	void MergePrevious2(DVLGroPreIntegration *pPrev);
	void SetNewBias(const Bias &bu_);
	IMU::Bias GetDeltaBias(const Bias &b_);
	// equation(44)
	cv::Mat GetDeltaRotation(const Bias &b_);
	cv::Mat GetDeltaRotation(const Bias &b, const cv::Mat &R_g_d);
	// equation(44)
	cv::Mat GetDeltaVelocity(const Bias &b_);
	// equation(44)
	cv::Mat GetDeltaPosition(const Bias &b_);
	cv::Mat GetDeltaPosition(const Bias &b, const cv::Mat &R_g_d);
	cv::Mat GetUpdatedDeltaRotation();
	cv::Mat GetUpdatedDeltaVelocity();
	cv::Mat GetUpdatedDeltaPosition();
	cv::Mat GetOriginalDeltaRotation();
	cv::Mat GetOriginalDeltaVelocity();
	cv::Mat GetOriginalDeltaPosition();
	Eigen::Matrix<double, 15, 15> GetInformationMatrix();
	cv::Mat GetDeltaBias();
	Bias GetOriginalBias();
	Bias GetUpdatedBias();
	void output();
	Eigen::Isometry3d getDVLPose();

public:
	double dT;
	cv::Mat C;
	cv::Mat Info;
	cv::Mat Nga, NgaWalk;

	// Values for the original bias (when integration was computed)
	Bias mb;
	/***
	 * dR: R_gi_gj. during integrtion R_gi_gk
	 * dV: V_di
	 * dP: t_di_didj, during integration t_di_didk
	 */
	cv::Mat dR, dV, dP;
	// mVelocity: V_dk velocity reading in DVL frame at time k
	// mAngV: angular_v last angular v, this is used for DVL inegration
	cv::Mat mVelocity, mR_g_d;
	cv::Point3d mAngV;

	// DVL beam orientation
	Eigen::Vector4d mAlpha;
	Eigen::Vector4d mBeta;
	Eigen::Matrix<double, 4, 3> mE;
	Eigen::Matrix<double, 3, 3> mETEInv;

	/*** euqation(44)
	 * JRg: Jacobian of R wrt bias_gro
	 * JVg: Jacobian of v wrt bias_gro
	 * JVq: Jacobian of v wrt bias_acc
	 * JPg: Jacobian of p wrt bias_gro
	 * JPa: Jacobian of p wrt bias_acc
	 */
	cv::Mat JRg, JVg, JVa, JPg, JPa;
	cv::Mat avgA;
	cv::Mat avgW;

	// whether we have DVL reading
	bool bDVL;
	// if velocity reading is more then threshold, filter it out
	double mVelocityThreshold;
	cv::Point3d v_dk_dvl;

	void SetDVLDebugVelocity(const cv::Point3d &v_dk);
	Eigen::Vector3d v_di_dvl, v_dk_visual;
	std::vector<double> mBeams;


private:
	// Updated bias
	Bias bu;
	// Dif between original and updated bias
	// This is used to compute the updated values of the preintegration
	cv::Mat db;

	struct integrable
	{
		integrable(const cv::Point3d &a_, const cv::Point3d &w_, const double &t_)
			: a(a_), w(w_), t(t_)
		{}
		integrable(const cv::Point3d &a_,
		           const cv::Point3d &w_,
		           const cv::Point3d &v_,
		           const Eigen::Vector4d &v_beam,
		           const double &t_)
			: a(a_), w(w_), v(v_), v_beam(v_beam), t(t_)
		{}
		cv::Point3d a;
		cv::Point3d w;
		cv::Point3d v;
		Eigen::Vector4d v_beam;
		double t;
	};

	std::vector<integrable> mvMeasurements;
	std::vector<integrable> mvMeasurements2;

	std::mutex mMutex;
};


#endif //DVLGROPREINTEGRATION_H
