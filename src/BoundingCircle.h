/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_BOUND_CIRCLE_H
#define CALIB_BOUND_CIRCLE_H

#include <iostream>
#include <math.h>
#include <vector>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include "global.h"


namespace multi_proj_calib
{
	/* BoundingCircle: class to detect bounding circle */
	class BoundingCircle
	{
	public:
		BoundingCircle() : m_bound_cnt(0), m_bound_radius(0.f), m_bound_center(cv::Point2f(0.f, 0.f)), m_findcircle(false)
		{
			m_bound_pt.clear();
		}

		//!automatic detection of bounding circle
		bool detectBoundary(const cv::Mat& img);

		//!manual detection of bounding circle
		bool detectBoundary();

		void addBoundaryPoint(const int& x, const int& y);

		//! draw the bounding circle; call after boudary detection
		void drawBoundaryCircle(cv::Mat& img);

		void reset()
		{
			m_bound_center = cv::Point2f(0.f, 0.f);
			m_bound_radius = 0.f;
			m_bound_cnt = 0;
			m_bound_pt.clear();
			m_findcircle = false;
		}

		const float& getBoundCircleRadius() const { return m_bound_radius; }
		
		const cv::Point2f& getBoundCircleCenter() const { return m_bound_center; }

		bool isBoundCircleFound() { return m_findcircle; }

		bool m_findcircle;
		int m_bound_cnt;
		float m_bound_radius;
		cv::Point2f m_bound_center;

		// vars for manual way to compute boundary
		std::vector<cv::Point2f> m_bound_pt;
		static const uint m_total_pt = 4;
		void linearLSCircle();

	};

}

#endif // !CALIB_BOUND_CIRCLE_H