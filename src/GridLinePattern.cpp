#include "GridLinePattern.h"

namespace multi_proj_calib
{
	using namespace cv;
	using std::cout;
	using std::endl;
	using std::to_string;

	/* GridPattern Class */

	bool GridPattern::detectCorner( cv::Mat& img)
	{
		if (img.empty())
		{
			throw std::runtime_error("ridPattern::detectCorner() fails: image is null. ");
		}
		
		bool found(false);
		
		/// pre-processing 
		Mat processed_img;
		m_background_img.substractBackGround(img, processed_img);
		medianBlur(processed_img, processed_img, 5);
		
		/// Apply Canny to detect edge with automatic threshold
		Mat edges;
		Mat _no_use_array;
		double otsu_thresh_val = cv::threshold(
			processed_img, _no_use_array, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
		);
		double high_thresh_val = otsu_thresh_val,
			lower_thresh_val = otsu_thresh_val * 0.01;
		Canny(processed_img, edges, lower_thresh_val, high_thresh_val);

		m_corners.clear();
		cv::goodFeaturesToTrack(edges, m_corners, m_max_corners, m_quality_level, m_min_dist, cv::Mat(), m_blockSize, true, m_k);
		found = !m_corners.empty();

		img = edges.clone();

		return found;
	}

	bool GridPattern::cleanCorner()
	{
		int removed = 0;
		const float thre = 30.f;
		float bound_cir_r = m_bound_circle.getBoundCircleRadius();
		Point2f bound_cir_o = m_bound_circle.getBoundCircleCenter();

		if (bound_cir_r <= 0)
		{
			cout << endl;
			cout << "GridPattern::cleanCorner(): need to first compute bounding circle" << endl;
			return false;
		}

		for (int i = m_corners.size() - 1; i >= 0; i--)
		{
			Point2f diff = m_corners[i] - bound_cir_o;
			float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
			if (dist + thre > bound_cir_r)
			{
				m_corners.erase(m_corners.begin() + i);
				removed++;
			}
		}
		if (!m_corners.empty())
		{
			if (removed > 0)
				cout << to_string(removed) << " corners have been removed: out of bounding circle" << endl;
			else
				cout << "GridPattern::cleanCorner(): No corner is removed, error in bounding circle detection" << endl;
			return true;
		}
		else
		{
			cout << "GridPattern::cleanCorner(): All corners have been removed, error in bounding circle detection" << endl;
			return false;
		}
	}

	void GridPattern::drawDetectedCorner(cv::Mat& img)
	{
		for (int i = 0; i < m_corners.size(); i++)
		{
			circle(img, m_corners[i], 10, cv::Scalar(255, 255, 255));
		}
	}

}