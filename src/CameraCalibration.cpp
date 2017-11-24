#include "CameraCalibration.h"

namespace multi_proj_calib
{

	using namespace cv;
	using namespace std;

	bool CameraCalibration::detectPattern(const Mat& img, vector<Point2f>& img_pts)
	{
		bool found(false);
		img_pts.clear();

		switch (m_pattern)
		{
		case CHECKER_BOARD:
			found = findChessboardCorners(img, m_pattern_size, img_pts,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			break;
		case CIRCLE_GRID:
			m_blob_detector.set_thre(10.f, 255.f);
			m_blob_detector.set_params("area", 4.f, m_img_size.width * m_img_size.height);
			m_blob_detector.set_params("circularity", 0.8f, 1.f);
			m_blob_detector.set_params("convexity", 0.8f, 1.f);
			m_blob_detector.set_params("inertia", 0.1f, 1.f);
			
			found = findCirclesGrid(img, m_pattern_size, img_pts,
				CALIB_CB_SYMMETRIC_GRID, new SimpleBlobDetector(m_blob_detector.params));	
			break;
		default:
			found = false;
			break;
		}

		if (found)
		{
			if (m_pattern == CHECKER_BOARD)
			{
				Mat img_gray;
				if (img.type() != CV_8UC1 && img.type() != CV_16UC1)
					cvtColor(img, img_gray, COLOR_BGR2GRAY);
				else
					img_gray = img;

				cornerSubPix(img_gray, img_pts, Size(5, 5), Size(-1, -1),
					TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 40, 0.005));
			}
			if (img_pts.size() != m_pattern_size.width * m_pattern_size.height)
			{
				img_pts.clear();
				m_msg = "Not a complete pattern; remove data";
				found = false;
				return found;
			}
		} 
		else
			m_msg = "Not a complete pattern";
		return found;
	}//end of detect function

	void CameraCalibration::createPatternObjectPoints()
	{
		m_pattern_pts.clear();
		for (int i = 0; i < m_pattern_size.height; ++i)
			for (int j = 0; j < m_pattern_size.width; ++j)
				m_pattern_pts.push_back(Point3f(float(j*m_square_size), float(i*m_square_size), 0.f));
	}

	bool CameraCalibration::computeBoardPose(const vector<Point2f> & img_pts, Mat& board_r, Mat& board_t)
	{
		if (!m_pattern_pts.empty())
		{
			Mat rvec;

			bool ready = cv::solvePnP(m_pattern_pts, img_pts, m_cam_mat, m_dist_coeff, rvec, board_t);
			cv::Rodrigues(rvec, board_r);
			cout << endl;
			cout << "Board Rotation:" <<endl<< board_r << endl << endl;
			cout << "Board Translation" << endl<<board_t << endl << endl;

			return ready;
		}
		else
		{
			cout << endl;
			cout << "CameraCalibration::computeBoardPose() fails: initialize m_pattern_pts first" << endl;
			return false;
		}
	}
	
	bool CameraCalibration::runCalibration()
	{
		bool ready = false;
		m_msg = "Calibrating...";
		ready = calibrate(m_mincalib_frame);
		if (ready)
		{
			ready = clean(.9f);
			m_msg = "Img :" + to_string(currFrame()) + "/" + to_string(m_mincalib_frame) + " Reproj Error: " + to_string(m_reproj_err);
		}
		else
		{
			m_msg = "Calibration Fail";
		}
		return ready;
	}
}