/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_GRID_LINE_PATTERN_H
#define CALIB_GRID_LINE_PATTERN_H

#include <iostream>

#include <vector>
#include <stdexcept>

#include "BoundingCircle.h"
#include "BackGroundSubstractor.h"

#include "global.h"

namespace multi_proj_calib
{

	/* GridPattern: class to detect grid patterns and compute pixel errors (not in use) */
	class GridPattern
	{
	public:
		//!detect and record corners
		bool detectCorner( cv::Mat& img);

		//!call after detectCorner
		bool cleanCorner();

		//!call after cleanCorner
		void drawDetectedCorner(cv::Mat& img);

		void refineCorners(const cv::Mat& img)
		{
			cornerSubPix(img, m_corners, cv::Size(3, 3), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 40, 0.005));
		}

		//!automatic detect bounding circle: update background image once completed
		bool detectBoundingCircle(const cv::Mat& img)
		{
			return m_bound_circle.detectBoundary(img);
		}

		//!call after bounding detection
		void drawBoundingCircle(cv::Mat& img)
		{
			return m_bound_circle.drawBoundaryCircle(img);
		}

		//!call after bounding detection succeed
		void setBackGroundImg(const cv::Mat& img)
		{
			m_background_img.updateBackGroundImg(img);
		}

		void setDetectorParams(const int& max_corners, const double& quality_level, const double& min_d, const int& block_size = 3, const float& k = 0.04)
		{
			m_max_corners = max_corners;
			m_quality_level = quality_level;
			m_min_dist = min_d;

			m_blockSize = block_size;
			m_k = k;
		}
	private:

		// Detector parameters	
		int m_blockSize;
		int m_max_corners;
		double m_quality_level;
		double m_min_dist;
		double m_k;

		// corners
		std::vector<cv::Point2f> m_corners;
		BoundingCircle m_bound_circle;
		BackGroundSubstractor m_background_img;
	};
}

#endif // !CALIB_GRID_LINE_PATTERN_H