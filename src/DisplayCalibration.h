/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_CALIBRATION_DISPLAY_H
#define CALIB_CALIBRATION_DISPLAY_H

#include <stdio.h>
#include <iostream>

#include <vector>
#include <algorithm> 
#include <stdexcept>

#include "ProjectorCalibration.h"
#include "CameraCalibration.h"
#include "ProjectorSys.h"
#include "Flea3Cam.h"
#include "Pixel3DArray.h"

#include "LmOptimizer.h"
#include "FundamentalMat.h"
#include "BackGroundSubstractor.h"
#include "BlobPattern.h"
#include "BoundingCircle.h"

#include "global.h"

namespace multi_proj_calib
{
	namespace dp_calib
	{
		enum FsmStatus { project = 0, detect = 1, start = 2, calibrate = 3, idle = 4, finish = 5};
		enum CalibMethod { Auto = 0, SemiAuto = 1 };
		void pairCalibOnMouse(int event, int x, int y, int flags, void* userdata);
		void displayOnMouse(int event, int x, int y, int flags, void* userdata);
	}

	/* DisplayCalibration: core class to calibrate the display by computing display pose, 3D pixel coordinates and alpha mask */

	class DisplayCalibration
	{
	public:
		
		///***** Interface ******///
		
		DisplayCalibration( int proj_count) :
			m_blobs(), m_cam_calib(), m_proj_calib(), m_camera(), m_projectors(proj_count, setting::proj_width, setting::proj_height), m_calibMethod(dp_calib::SemiAuto),m_num_proj(proj_count), m_MaxReprojErr(3) {}

		// initialize camera, projector, camcalib and projcalib
		bool setup();

		// clean up camera and projector
		bool cleanup();

		// show a blob on each projector and determine the order of projectors
		bool testWindowSequence();

		// project and detect feature for each projecor+camera pair
		bool calibratePair();

		// estimate extrinsics for each projecor+camera pair
		bool estimatePairExtrinsic();

		// estimate sphere pose: call after all pairs have been recovered
		double estimateSpherePose();

		// ray-sphere intersection to locate pixels on sphere for all projectors
		void computePixel3D();

		// compute alpha mask for all projectors
		void computeAlphaMask();
				
		// display calibration result: it should appear to be a grid pattern from camera viewpoint
		bool displayCorrectedPattern();

		// update refined parameters  
		bool updateParams(bool isFromMatlab, std::vector<double> new_param = std::vector<double>());

		// Nonlinear Optimization using lm methord
		void lmOptimize();

		// save calibration result to dir with file names defined in global.h
		void saveCalibrationResult(std::string dir = std::string());

		// load calibration result to dir with file names defined in global.h
		void loadCalibrationResult(std::string dir = std::string());

		// read blob data from xml files
		bool loadBlobData(const std::string& file_name); //todo may not in use

		// save an initial guess of extrinsic and sphere pose parameters
		bool saveExtrinsics(const std::string& file_name);

		void setFsmMode(dp_calib::FsmStatus mode){
			m_mode = mode; }

		void setBoundPoint(const int& x, const int& y){
			m_blobs.addBoundPoint(x, y); }

		void setCurrProjector(int proj_cnt) {
			m_curr_proj = proj_cnt; }

		void setCalibrationMethod(dp_calib::CalibMethod m){
			m_calibMethod = m; }

		uint& getCurrProjector() { return m_curr_proj; }

		const uint& getTotalProjector() { return m_num_proj; }

		dp_calib::CalibMethod getCalibMethod() { return m_calibMethod; }

		bool isBoundCircleFound() { return m_blobs.isBoundCircleFound(); }

	protected:
		
		///***** Member Methods *****///

		bool triangulatePairFeature(const std::vector<cv::Point2f>& cam_blobs_nml, const cv::Mat& cam_proj_mat, const std::vector<cv::Point2f>& proj_blobs_nml, const cv::Mat& proj_proj_mat, std::vector<cv::Point3f>& obj_pts);

		// linear triangulation from two views
		cv::Mat_<double> linearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1);

		// sphere pose estimation Linear Weighted Least Square
		double linearWLSSpherePose(const std::vector<cv::Point3f>& blobs, const std::vector<float>& weight, cv::Mat_<double>& sphere_pose);

		///***** Member Data *****///
		
		uint m_num_proj;
		uint m_curr_proj;
		double m_MaxReprojErr; // maximum reprojection error allowed
		
		std::string m_cvwindow;
		cv::Mat_<double> m_sphere_pose;

		dp_calib::FsmStatus m_mode;
		dp_calib::CalibMethod m_calibMethod;

		CameraCalibration m_cam_calib; // camera calibration 
		std::vector<ProjectorCalibration> m_proj_calib; // projector calibration for each projector
		ProjectorSys m_projectors; // projector displays
		Flea3Cam m_camera; // camera control
		LmOptimizer m_optimizer;

		BlobPattern m_blobs; // projected pattern
		std::vector<Pixel3DArray> m_pixelarray;  // store and compute geometry data and alpha mask for each projector

		std::vector<std::vector<cv::Point2f>> m_cam_pts; //todo: may not in use
		std::vector<std::vector<cv::Point2f>> m_proj_pts;//todo: may not in use
		std::vector<std::vector<cv::Point3f>> m_obj_pts;
		std::vector<std::vector<float>> m_reproj_err;
	};

}

#endif // !CALIB_CALIBRATION_DISPLAY_H