/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_CALIBRATION_CAMERA_H_
#define CALIB_CALIBRATION_CAMERA_H_

#include "opencv2/features2d/features2d.hpp"

#include "CalibrationBase.h"
#include "BlobDetector.hpp"
#include "global.h"

namespace multi_proj_calib
{
	class CameraCalibration : public CalibrationBase 
	{
	
	public:
		CameraCalibration(): CalibrationBase(){}
		
		//! detect pre-set pattern: call in loop
		bool detectPattern(const cv::Mat& img, std::vector<cv::Point2f>& img_pts); 

		//! wrapper of calibrate() and clean(): call in loop of camera calibration
		bool runCalibration();

		//! prepare board pose for projector: call in loop of projector calibration 
		bool computeBoardPose(const std::vector<cv::Point2f> & img_pts, cv::Mat& board_r, cv::Mat& board_t); 

		//! create checkerboard pattern points in 3D
		void createPatternObjectPoints();

		//! use as arg in add()
		const std::vector<cv::Point3f> & getPresetObjectPoints() const { return m_pattern_pts; }  
		
	private:
		std::vector<cv::Point3f> m_pattern_pts;
		BlobDetector m_blob_detector;
	};

}

#endif