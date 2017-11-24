/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_CALIBRATION_BASE_H_
#define CALIB_CALIBRATION_BASE_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "global.h"

namespace multi_proj_calib
{

	enum CalibrationPattern { CHECKER_BOARD, CIRCLE_GRID, BLOCKED_CIRCLE_GRID };

	class CalibrationBase
	{
	public:
		CalibrationBase():  m_mincalib_frame(6), m_total_frame(20), m_reproj_err(0), m_square_size(1),
							m_pattern(CHECKER_BOARD), m_cam_mat(cv::Mat::eye(3, 3, CV_32F)),
							m_dist_coeff(cv::Mat::zeros(8, 1, CV_32F)), m_pattern_size(cv::Size(8, 6)),
							m_img_size(cv::Size(setting::cam_width, setting::cam_height)), m_msg("press 's' to start") {}
		CalibrationBase(cv::Size resolution, int total_frame, CalibrationPattern pattern, cv::Size pattern_size, std::string msg)
			: m_img_size(resolution), m_total_frame(total_frame), m_pattern(pattern), m_pattern_size(pattern_size), 
			  m_cam_mat(cv::Mat::eye(3, 3, CV_32F)), m_dist_coeff(cv::Mat::zeros(8, 1, CV_32F)), m_msg(msg) {}
		
		virtual ~CalibrationBase(){}
		
		//! calibration using cv function calibrateCamera()
		bool calibrate(uint frame_count);

		//! erase image that has reproj-err larger than pre-set threshold
		bool clean(float maxReprojectionError = 1.f);

		//! push 2d-3d correspondence
		bool add(std::vector<cv::Point2f> img_pts, std::vector<cv::Point3f> obj_pts);
		
		//! undistort Input&Output img using lens parameter
		void undistortImage(cv::Mat& img);

		//! remove 2d-3d correspondence
		void remove(int index);

		void resetData();

		void printIntrinsics();

		void drawDetectedPattern(cv::Mat& img, const std::vector<cv::Point2f>& pattern_pts, cv::Size pattern_size);

		void setImageParams(uint width, uint height);

		void setFrameCount(uint total_frame = 20, uint min_calib_frame = 6);

		void setPatternParams(CalibrationPattern pattern, int pattern_width = 8, int pattern_height = 6, int square_size = 1);

		uint currFrame() { return m_obj_pts.size(); }
		uint getTotalFrame() const { return m_total_frame; }
		uint getMinCalibFrame() const { return m_mincalib_frame; }

		float getReprojectionError() const { return m_reproj_err; }
		float getReprojectionError(int i) const { return m_perview_err[i]; }

		const std::string & getCalibMsg() const { return m_msg; }
		const std::vector<std::vector<cv::Point2f>> & getImagePoints() const { return m_img_pts; }

		const cv::Mat getCameraMatrix() const { return m_cam_mat; }  
		const cv::Mat getDistortCoeff() const { return m_dist_coeff; } 

		void updateCameraMatrix(const cv::Mat& new_cam_mat)  {  m_cam_mat = new_cam_mat.clone(); }

		void updateDistortCoeff(const cv::Mat& new_dist)  {  m_dist_coeff = new_dist.clone();  }

		void saveCalibParams(const std::string& file_name);
		bool loadCalibParams(const std::string& file_name);

		bool isReady() const{ return m_ready; }
	
	protected:

		std::vector<cv::Mat> m_rot_vecs, m_transl_vecs;
		cv::Mat m_cam_mat;
		cv::Mat m_undist_cam_mat;
		cv::Mat m_dist_coeff;
		
		float m_reproj_err;
		float m_max_reproj_err;
		std::vector<float> m_perview_err;

		CalibrationPattern m_pattern;
		cv::Size m_pattern_size;
		cv::Size m_img_size;

		uint m_total_frame;
		uint m_mincalib_frame;
		uint m_square_size;

		std::vector<std::vector<cv::Point2f>> m_img_pts;
		std::vector<std::vector<cv::Point3f>> m_obj_pts;

		bool m_ready;

		std::string m_msg;

		void updateReprojectionError();
		void updateUndistortion();

	};

}//end of namespace

#endif // !CALIB_CALIBRATION_BASE_H_


