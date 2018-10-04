/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_BACKGROUND_SUBSTRACTOR_H
#define CALIB_BACKGROUND_SUBSTRACTOR_H

#include "../src/global.h"

namespace multi_proj_calib
{
	/* BackGroundSubstractor: class to store and substract BG img */
	class BackGroundSubstractor
	{
	public:
		void updateBackGroundImg(const cv::Mat& img)
		{
			m_bg_img.create(img.rows, img.cols, setting::camera::cv_pixel_format);
			m_bg_img = img.clone();
		}
		void substractBackGround(const cv::Mat& src, cv::Mat& dest)
		{
			dest = src - m_bg_img;
		}
	private:
		cv::Mat m_bg_img;
	};
}

#endif // !CALIB_BACKGROUND_SUBSTRACTOR_H