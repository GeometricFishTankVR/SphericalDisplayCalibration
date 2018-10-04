/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_CALIBRATION_PROJCTOR_H_
#define CALIB_CALIBRATION_PROJCTOR_H_

#include "calibrationbase.h"
#include "global.h"

namespace multi_proj_calib 
{
	class ProjectorCalibration : public CalibrationBase 
	{
	public:
		ProjectorCalibration():CalibrationBase(cv::Size(setting::proj::res_width, setting::proj::res_height), 20, CIRCLE_GRID, cv::Size(setting::camera::checkerboard_row, setting::camera::checkerboard_col), "click right mouse button to capture") {}
		ProjectorCalibration(cv::Size resolution, CalibrationPattern pattern, cv::Size pattern_size): 
			CalibrationBase(resolution, 20, pattern, pattern_size, "click right mouse button to capture")  {}
		
		//! erase image that has reproj-err larger than pre-set threshold; data are stored in p_calib = p_cam: call in runCalibration
		bool clean(float max_reproj_err, CalibrationBase* p_calib);

		//! wrapper of calibrate() and clean(): call in loop of projector calibration
		bool runCalibration( CalibrationBase* p_cam);
		
		//! ray-plane intersection to compute projected points in 3D
		bool computeObjectPoints(const CalibrationBase* p_cam, const std::vector<cv::Point2f>& cam_img_pts, std::vector<cv::Point3f>& object_pts);
		
		//! compute extrinsics in camera-centered coordinate
		bool computeProjectorExtrinsic(const CalibrationBase* p_cam);

		//! not in use now
		bool computeDynamicProjection(const std::vector<cv::Point3f>& object_point, const cv::Mat& board_r, const cv::Mat& board_t);

		//! compute projected points in 2D
		void createImagePoints();
		
		//! detect pixels only above threshold: not in use now
		void preProcessImage(const cv::Mat& src_img, cv::Mat& proc_img, int thre);

		const std::vector<cv::Point2f> & getImagePoints() { return m_pattern_pts; }

		void setPatternPixel(float dwidth_pixel, float dheight_pixel);

		void setStaticFrame(uint static_frame);

		void setExtrinsic(const cv::Mat& rmat, const cv::Mat& tvec)
		{
			m_r3x3_proj2cam = rmat.clone();
			m_t3x1_proj2cam = tvec.clone();
		}

		std::vector<cv::Point2f>& getPresetImagePoint() { return m_pattern_pts; }

		cv::Mat& getBoardRotation() { return m_r3x3_board;  } 

		cv::Mat& getBoardTranslation() { return m_t3x1_board;  } 

		cv::Mat& getExtrinsicRmat() { return m_r3x3_proj2cam; }

		cv::Mat& getExtrinsicRvec() 
		{
			cv::Rodrigues(m_r3x3_proj2cam, m_r3x1_proj2cam);
			return m_r3x1_proj2cam;
		}

		cv::Mat& getExtrinsicTvec() { return m_t3x1_proj2cam; }

		uint getStaticFrame() { return m_static_frame; }

	private:
		cv::Mat m_r3x3_proj2cam;
		cv::Mat m_r3x1_proj2cam;
		cv::Mat m_t3x1_proj2cam;

		cv::Mat m_r3x3_board;
		cv::Mat m_t3x1_board;

		cv::Size2f m_dpattern_pixel;
		std::vector<cv::Point2f> m_pattern_pts;
		uint m_static_frame;

		std::vector<cv::Point2f> m_dynamic_quad;
	};
}


#endif // !CALIB_CALIBRATION_PROJCTOR_H_