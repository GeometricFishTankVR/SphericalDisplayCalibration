#include "CalibrationBase.h"

namespace multi_proj_calib
{
	
	using namespace cv;
	using std::cout;
	using std::endl;

	bool CalibrationBase::calibrate(uint frame_count)
	{
		if ( currFrame() >= frame_count)
		{
			static int calib_flag = 0;

			if (calib_flag == CV_CALIB_USE_INTRINSIC_GUESS)
			{
				Point2f principalPoint;
				principalPoint.x = m_cam_mat.at<double>(0, 2);
				principalPoint.y = m_cam_mat.at<double>(1, 2);

				// check whether principle points are valid as initial guess to calibrateCamera()
				bool pp_valid_x = checkRange(principalPoint.x, true, 0, 0, m_img_size.width);
				bool pp_valid_y = checkRange(principalPoint.y, true, 0, 0, m_img_size.height);
				if (! (pp_valid_x & pp_valid_y) )
				{
					cout << endl;
					cout << "Adjust principle points to be within the range" << endl;
					if (!pp_valid_x)
						m_cam_mat.at<double>(0, 2) = m_img_size.width / 2 - 1;
					if (!pp_valid_y)
						m_cam_mat.at<double>(1, 2) = m_img_size.height - 1;
				}
			}

			double rms = calibrateCamera(m_obj_pts, m_img_pts, m_img_size, m_cam_mat, m_dist_coeff, m_rot_vecs, m_transl_vecs, calib_flag, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 50, 1e-6));
						
			cout << endl;
			cout << "m_cam_mat" << m_cam_mat << endl << endl;
			cout << "m_img_size" << m_img_size << endl << endl;

			Point2f principalPoint;
			principalPoint.x = m_cam_mat.at<double>(0, 2);
			principalPoint.y = m_cam_mat.at<double>(1, 2);
		
			cout << endl;
			cout << "Calibrated, update reprojection error and distortions"<< endl;

			updateReprojectionError();
			updateUndistortion();
			calib_flag = CV_CALIB_USE_INTRINSIC_GUESS;
		}		

		return true;
	}

	bool CalibrationBase::clean(float max_reproj_err)
	{
		int removed = 0;
		for (int i = currFrame() - 1; i >= 0; i--) {
			if (getReprojectionError(i) > max_reproj_err) 
			{
				remove(i);
				cout << "CalibrationBase::clean() removed Img " << std::to_string(i+1) << endl;
				removed++;
			}
		}
		if (currFrame() > 0) {
			if (removed > 0)
			{
				cout << endl;
				cout << "CalibrationBase::clean() removed " << std::to_string(removed) << " imgs" << endl;
				m_ready = false;
			}
			else 
				m_ready = true;
		}
		else {
			cout << endl;
			cout << "CalibrationBase::clean() removed the last object/image point pair" << endl;
			m_ready = false;
		}
		return m_ready;
	}

	bool CalibrationBase::add(std::vector<cv::Point2f> img_pts, std::vector<cv::Point3f> obj_pts)
	{
		if (!obj_pts.empty() && !img_pts.empty())
		{
			m_img_pts.push_back(img_pts);
			m_obj_pts.push_back(obj_pts);
			m_msg = "Img :" + std::to_string(currFrame()) + "/" + std::to_string(m_mincalib_frame) + "/" + std::to_string(m_total_frame) + " Reproj Error: " + std::to_string(0);
			m_ready = true;
		}
		else
			m_ready = false;
		return m_ready;
	}

	void CalibrationBase::undistortImage(cv::Mat& img)
	{
		cv::Mat undis_img_buf;
		undistort(img, undis_img_buf, m_cam_mat, m_dist_coeff);
		img = undis_img_buf.clone();
	}
	
	void CalibrationBase::resetData()
	{
		m_img_pts.clear();
		m_obj_pts.clear();
		m_rot_vecs.clear();
		m_transl_vecs.clear();
		m_perview_err.clear();
	}

	void CalibrationBase::remove(int index)
	{
		this->m_obj_pts.erase(m_obj_pts.begin() + index);
		this->m_img_pts.erase(m_img_pts.begin() + index);		
	}

	void CalibrationBase::setImageParams(uint width, uint height)
	{
		m_img_size.width = width;
		m_img_size.height = height;
	}

	void CalibrationBase::setPatternParams(CalibrationPattern pattern, int pattern_width, int pattern_height, int square_size)
	{
		m_pattern = pattern;
		m_pattern_size.width = pattern_width;
		m_pattern_size.height = pattern_height;
		m_square_size = square_size;
	}

	void CalibrationBase::setFrameCount(uint total_frame, uint min_calib_frame)
	{
		m_mincalib_frame = min_calib_frame;
		m_total_frame = total_frame;
	}


	void CalibrationBase::updateUndistortion()
	{
		m_undist_cam_mat = getOptimalNewCameraMatrix( m_cam_mat, m_dist_coeff, m_img_size, 0, m_img_size);
	}

	void CalibrationBase::updateReprojectionError() 
	{
		std::vector<Point2f> image_points2;
		int total_points = 0;
		double total_err = 0;

		m_perview_err.clear();
		m_perview_err.resize(currFrame());

		for (int i = 0; i < (int)m_obj_pts.size(); i++) {
			projectPoints(Mat(m_obj_pts[i]), m_rot_vecs[i], m_transl_vecs[i], m_cam_mat, m_dist_coeff, image_points2);
			double err = norm(Mat(m_img_pts[i]), Mat(image_points2), CV_L2);
			int n = m_obj_pts[i].size();
			m_perview_err[i] = sqrt(err * err / n);
			total_err += err * err;
			total_points += n;
			cout << "view" << std::to_string(i) << " has error of " << std::to_string(m_perview_err[i]) << endl;
		}

		m_reproj_err = sqrt(total_err / total_points);
		cout << endl;
		cout << "all views have error of " << std::to_string(m_reproj_err) << endl;
	}

	void CalibrationBase::saveCalibParams(const string& file_name)
	{
		std::string fileToSave;

		std::ifstream filetemp(file_name.c_str());
		if (filetemp.good())
		{
			fileToSave = file_name.substr(0, file_name.find_last_of(".")) + "_1" + file_name.substr(file_name.find_last_of("."));
			cout << "File " << file_name << " exists. Save to " << fileToSave << endl;
		}

		FileStorage fs(fileToSave, FileStorage::WRITE);

		if (!m_rot_vecs.empty())
			fs << "nrofframes" << (int)m_rot_vecs.size();

		fs << "image_width" << m_img_size.width;
		fs << "image_height" << m_img_size.height;
		fs << "board_width" << m_pattern_size.width;
		fs << "board_height" << m_pattern_size.height;
		fs << "square_size" << (int)m_square_size;

		fs << "camera_matrix" << m_cam_mat;
		fs << "distortion_coefficients" << m_dist_coeff;
		fs << "reprojection_error" << m_reproj_err;

		if (!m_rot_vecs.empty() && !m_transl_vecs.empty())
		{
			CV_Assert(m_rot_vecs[0].type() == m_transl_vecs[0].type());
			Mat bigmat((int)m_rot_vecs.size(), 6, m_rot_vecs[0].type());
			for (int i = 0; i < (int)m_rot_vecs.size(); i++)
			{
				Mat r = bigmat(Range(i, i + 1), Range(0, 3));
				Mat t = bigmat(Range(i, i + 1), Range(3, 6));

				CV_Assert(m_rot_vecs[i].rows == 3 && m_rot_vecs[i].cols == 1);
				CV_Assert(m_transl_vecs[i].rows == 3 && m_transl_vecs[i].cols == 1);

				r = m_rot_vecs[i].t();
				t = m_transl_vecs[i].t();
			}
			cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each board", 0);
			fs << "extrinsic_parameters" << bigmat;
			fs.release();
		}

		if (!m_img_pts.empty())
		{
			Mat imageptmat((int)m_img_pts.size(), (int)m_img_pts[0].size(), CV_32FC2);
			for (int i = 0; i < (int)m_img_pts.size(); i++)
			{
				Mat r = imageptmat.row(i).reshape(2, imageptmat.cols);
				Mat imgpti(m_img_pts[i]);
				imgpti.copyTo(r);
			}
			fs << "image_points" << imageptmat;
		}
		m_msg = "Result saved; Press q to quit";
	}

	bool CalibrationBase::loadCalibParams(const std::string& file_name)
	{
		FileStorage fs(file_name, FileStorage::READ);
		
		if (!fs.isOpened())
		{
			cout << endl;
			cout << "Failed to open " << file_name << endl;
			return false;
		}
		
		fs["camera_matrix"] >> m_cam_mat;
		fs["distortion_coefficients"] >> m_dist_coeff;
		fs["reprojection_error"] >> m_reproj_err;

		cout << endl;
		cout << "Load Calibration Result from: " << file_name << endl;
		
		printIntrinsics();

		m_ready = true;
		fs.release();
		return m_ready;
	}

	void CalibrationBase::printIntrinsics()
	{
		std::cout << "Intrinsic mat:" << std::endl <<
			getCameraMatrix() << std::endl << std::endl;
		std::cout << "distortion coefficients:" << getDistortCoeff() << std::endl << std::endl;
		std::cout << "re-proj error:" << getReprojectionError() << std::endl;
	}

	void CalibrationBase::drawDetectedPattern(cv::Mat& img, const std::vector<cv::Point2f>& pattern_pts, cv::Size pattern_size)
	{
		if (img.empty())
			throw std::runtime_error("drawDetectedPattern() fails to draw detected patterns: image can not be empty. ");
		
		m_pattern_size = cv::Size(setting::camera::checkerboard_row, setting::camera::checkerboard_col);
		
		if (!pattern_pts.empty())
		{
			m_pattern_size = pattern_size;
		}

		drawChessboardCorners(img, m_pattern_size, pattern_pts, true);
		circle(img, pattern_pts[0], 10, cv::Scalar(0, 0, 0), -1, 6);
		arrowedLine(img, pattern_pts[0], pattern_pts[1], cv::Scalar(0, 0, 0), 2);
	}
}

