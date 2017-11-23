#include "BlobPattern.h"

namespace multi_proj_calib
{
	using namespace cv;
	using std::cout;
	using std::endl;
	using std::to_string;

	/* BlobPattern Class */

	Point2f BlobPattern::generateRandomBlob(const Size& proj_img_size, const float blob_radius_pixel)
	{
		float blob_x = m_rng.uniform(blob_radius_pixel, proj_img_size.width - blob_radius_pixel);
		float blob_y = m_rng.uniform(blob_radius_pixel, proj_img_size.height - blob_radius_pixel);

		m_projected_blob = Point2f(blob_x, blob_y);

		return m_projected_blob;
	}
	
	Point2f BlobPattern::generateBlobGrid(const Size& proj_img_size, const float blob_radius_pixel)
	{
		float d_width = (float)(proj_img_size.width - 1) / (float)(m_grid_size.width + 1);
	    float d_height = (float)(proj_img_size.height - 1) / (float)(m_grid_size.height + 1);

		if (!utils::isInRange((int)d_width, 0, proj_img_size.width) || !utils::isInRange((int)d_height, 0, proj_img_size.height))
		{
			throw std::runtime_error("BlobPattern::generateBlobGrid() fails: invalid d_width or d_height. ");
		}
	
		float blob_x = m_curr_cir.width * d_width;
		float blob_y = m_curr_cir.height * d_height;
		
		if (m_curr_cir.width < m_grid_size.width)
		{
			m_curr_cir.width++;
		}
		else
		{
			m_curr_cir.width = 1;
			m_curr_cir.height++;
		}

		m_projected_blob = Point2f(blob_x, blob_y);
		return m_projected_blob;
	}

	void BlobPattern::setupBlobDetector()
	{
		m_blob_detector.set_thre(20.f, 255.f);
		m_blob_detector.set_params("area", 100.f, 2500.f);
		m_blob_detector.set_params("circularity", 0.8f, 1.f);
		m_blob_detector.set_params("convexity", 0.9f, 1.f);
		m_blob_detector.set_params("inertia", .1f, 1.f);
	}

	void BlobPattern::resetBlobs()
	{
		m_cam_blobs.clear();
		m_proj_blobs.clear();
		m_curr_cir = Size(1, 1);
	}

	void BlobPattern::addBlob()
	{
		m_cam_blobs.push_back(m_detected_blob);
		m_proj_blobs.push_back(m_projected_blob);
	}

	bool BlobPattern::cleanBlobs()
	{
		if (m_cam_blobs.size() != m_proj_blobs.size())
		{
			throw std::runtime_error("BlobPattern::cleanBlobs() fails: camera and projector have inequal numbers of blobs. ");
		}
		int removed = 0;
		const float thre = 10.f;
		float bound_cir_r = m_bound_circle.getBoundCircleRadius();
		Point2f bound_cir_o = m_bound_circle.getBoundCircleCenter();

		for (int i = getCurrBlobCnt() - 1; i >= 0; i--)
		{
			Point2f diff = m_cam_blobs[i] - bound_cir_o;
			float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
			if ( dist + thre > bound_cir_r)
			{
				m_cam_blobs.erase(m_cam_blobs.begin() + i);
				m_proj_blobs.erase(m_proj_blobs.begin() + i);
				removed++;
			}
		}

		if (!m_cam_blobs.empty())
		{
			if (removed > 0)
				cout << to_string(removed) << " blobs have been removed: out of bounding circle" << endl;
			else
				cout << "BlobPattern::cleanBlobs(): No blob is removed." << endl;
			return true;
		}
		else
		{
			cout << "BlobPattern::cleanBlobs(): All blobs have been removed, possible error in bounding circle detection" << endl;
			return false;
		}
	}

	bool BlobPattern::detectSingleBlob( Mat& img, float intensityScalar)
	{
		if (img.empty())
		{
			throw std::runtime_error("BlobPattern::detectSingleBlob() fails to detect blob, image is null. ");
		}
		Mat substract_img, binary_img;
		m_background_img.substractBackGround(img, substract_img);
		
		medianBlur(substract_img, substract_img, 15);

		substract_img = substract_img * intensityScalar;
		substract_img = 255 - substract_img;

		double otsu_thresh_val_low = cv::threshold(
			substract_img, binary_img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU); 

		m_blob_detector.set_thre(otsu_thresh_val_low*0.1f, otsu_thresh_val_low*1.2f);

		vector<KeyPoint> key_pts;
		key_pts.clear();
		m_blob_detector.detect(substract_img, key_pts);
		
		cout << "keypoint "<<key_pts.size() << endl;

		if ( key_pts.size() == 1)
		{
			m_detected_blob = key_pts[0].pt;
			putText(img, to_string(m_projected_blob.x) + "," + to_string(m_projected_blob.y), m_detected_blob, 1, 1.f, Scalar(0, 0, 0));
			cout << "blob number:" << getCurrBlobCnt() << endl;
			return true;
		}
		else
			return false;
	}

	void BlobPattern::drawDetectedBlob(Mat& img)
	{
		circle(img, m_cam_blobs.back(), 50, Scalar(255, 255, 255));
	}
	 
	void BlobPattern::thresholdImage(const cv::Mat& src_img, cv::Mat& dest_img, int thre)
	{
		if (src_img.type() != CV_8UC1) {
			cvtColor(src_img, dest_img, CV_RGB2GRAY);
		}
		else {
			dest_img = src_img;
		}

		cv::threshold(dest_img, dest_img, thre, 150, cv::THRESH_BINARY);
	}

	void BlobPattern::saveBlobData(string file_name)
	{
		FileStorage fs(file_name, FileStorage::WRITE);

		if (m_cam_blobs.size() != m_proj_blobs.size())
		{
			throw std::runtime_error("BlobPattern::saveBlobData() fails: camera and projector have inequal numbers of blobs.");
		}
		
		int num_blobs = getCurrBlobCnt();

		Mat imageptmat( num_blobs, 2, CV_32FC1);
		
		for (int i = 0; i < num_blobs; i++)
		{
			imageptmat.at<float>(i, 0) = m_cam_blobs[i].x;
			imageptmat.at<float>(i, 1) = m_cam_blobs[i].y;
		}
		cvWriteComment(*fs, "random blobs in camera coordinate system", 0);
		fs << "cam_pts" << imageptmat;

		Mat projptmat( num_blobs, 2, CV_32FC1);

		for (int i = 0; i < num_blobs; i++)
		{
			projptmat.at<float>(i, 0) = m_proj_blobs[i].x;
			projptmat.at<float>(i, 1) = m_proj_blobs[i].y;
		}
		cvWriteComment(*fs, "blobs in projector coordinate system", 0);
		fs << "proj_pts" << projptmat;
	}

}