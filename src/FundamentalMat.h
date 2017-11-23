/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_FUNDAMENTAL_MAT_H
#define CALIB_FUNDAMENTAL_MAT_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <algorithm>
#include <math.h>
#include <stdlib.h> 
#include <time.h>  

namespace stereo_recon
{
	enum DistanceType { Sampson = 0, Geometric = 1, Residue = 2};
	
	class FundamentalMat
	{
	public:
		FundamentalMat(): m_thre_ratio(.99f){}
		
		cv::Mat_<float> findFundamentalMat(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, DistanceType type);

		cv::Mat_<float> findEssentialMat(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, DistanceType type);
		
		void setThresholdRatio(const float& thre)
		{
			m_thre_ratio = thre;
		}

	protected:

		float m_thre_ratio;  // modify this param if the error is too large
		
		size_t m_size;
		
		const unsigned int m_num_trials = 500;
		
		cv::Mat_<float> m_F;
		
		static int run8Point(const cv::Mat& _m1, const cv::Mat& _m2, cv::Mat& _fmatrix);

		void generateRandom8Pts(std::vector<int>& idx_rdm8pts);

		static void computeDistance(const std::vector<cv::Point3f>& pts1_h, const std::vector<cv::Point3f>& pts2_h, const cv::Mat_<float>& F, std::vector<float>& dvec, DistanceType type);

		inline float medianf(std::vector<float> vec);
	};
}




#endif
