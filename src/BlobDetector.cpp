#include "BlobDetector.hpp"

namespace multi_proj_calib
{
	void BlobDetector::set_thre(float min_thre, float max_thre)
	{
		params.minThreshold = min_thre;
		params.maxThreshold = max_thre;
	}

	void BlobDetector::set_params(std::string param_key, float min_val, float max_val)
	{
		if (!param_key.compare("area"))
		{
			params.filterByArea = true;
			params.minArea = min_val;
			if (max_val > min_val)
				params.maxArea = max_val;
		}
		else if (!param_key.compare("circularity"))
		{
			params.filterByCircularity = true;
			params.minCircularity = min_val;
			if (max_val > min_val)
				params.maxCircularity = max_val;
		}
		else if (!param_key.compare("convexity"))
		{
			params.filterByConvexity = true;
			params.minConvexity = min_val;
			if (max_val > min_val)
				params.maxConvexity = max_val;
		}
		else if (!param_key.compare("inertia"))
		{
			params.filterByInertia = true;
			params.minInertiaRatio = min_val;
			if (max_val > min_val)
				params.maxInertiaRatio = max_val;
		}
		else if (!param_key.compare("color")) 
		{
			params.filterByColor = true;
			params.blobColor = (uchar) min_val;
		}
	}

	void BlobDetector::detect(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints)
	{
		keypoints.clear();
		cv::SimpleBlobDetector simple_blob_detector(params);
		simple_blob_detector.detect(img, keypoints);
	}

	void BlobDetector::refine_keypoints(std::vector<cv::KeyPoint>& keypoints, int keypoints_num)
	{
		std::sort(keypoints.begin(), keypoints.end(), larger_than());

		keypoints.erase(keypoints.begin() + keypoints_num, keypoints.end());
	}

}



