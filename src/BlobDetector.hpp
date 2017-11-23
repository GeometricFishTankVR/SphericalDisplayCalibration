/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca), Kai Wu(imkaywu{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef BlobDetection_hpp
#define BlobDetection_hpp

#include <stdio.h>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

namespace multi_proj_calib
{
	class BlobDetector
	{
	private:

		struct larger_than
		{
			inline bool operator() (const cv::KeyPoint& keypoint_1, const cv::KeyPoint& keypoint_2)
			{
				return(keypoint_1.size > keypoint_2.size);
			}
		};

	public:
		BlobDetector();
		~BlobDetector() { };
		void set_thre(float, float);
		void set_params(std::string, float, float = 1.0);
		void detect(const cv::Mat&, std::vector<cv::KeyPoint>&);
		void refine_keypoints(std::vector<cv::KeyPoint>&, int);
		cv::SimpleBlobDetector::Params params;

	};

	inline BlobDetector::BlobDetector()
	{
		params.filterByArea = false;
		params.filterByCircularity = false;
		params.filterByColor = false;
		params.filterByConvexity = false;
		params.filterByInertia = false;
	}
}

#endif /* BlobDetection_hpp */
