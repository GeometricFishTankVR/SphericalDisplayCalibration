#include "ProjectorCalibration.h"

namespace multi_proj_calib
{
	using std::vector;
	using std::cout;
	using std::endl;
	using std::to_string;

	using cv::Mat;
	using cv::Point2f;
	using cv::Point3f;

	bool ProjectorCalibration::clean(float max_reproj_err, CalibrationBase* p_calib)
	{
		int removed = 0;
		for (int i = currFrame() - 1; i >= 0; i--) {
			if (getReprojectionError(i) > max_reproj_err)
			{
				this->remove(i);
				p_calib->remove(i);
				cout << "ProjectorCalibration::clean() removed Img " << std::to_string(i + 1) << endl;
				removed++;
			}
		}
		if (currFrame() > 0) {
			if (removed > 0)
			{
				cout << endl;
				cout << "ProjectorCalibration::clean() removed " << std::to_string(removed) << " imgs" << endl;
				m_ready = false;
			}
			else
				m_ready = true;
		}
		else {
			cout << endl;
			cout << "ProjectorCalibration::clean() removed the last object/image point pair" << endl;
			m_ready = false;
		}
		return m_ready;
	}
	
	bool ProjectorCalibration::runCalibration( CalibrationBase* p_cam)
	{
		m_ready = false;
		m_msg = "Calibrating...";
		m_ready = calibrate(m_mincalib_frame); //calibration based on mincalib-frames
		if (m_ready)
		{
			m_ready = clean(setting::proj::max_reproj_error, p_cam); //clean both proj and camera points
			m_msg = "Img :" + to_string(currFrame()) + "/" + to_string(m_total_frame) + " Reproj Error: " + to_string(m_reproj_err);
			if (m_ready && (p_cam->currFrame() == this->currFrame()) )
				m_ready = computeProjectorExtrinsic(p_cam);
		}
		else
		{
			m_msg = "Calibration fail: remove all points and start again";
			this->resetData();
			p_cam->resetData();
		}
		return m_ready;
	}

	bool ProjectorCalibration::computeProjectorExtrinsic(const CalibrationBase* p_cam)
	{
		Mat proj_cam_mat = this->m_cam_mat;
		Mat proj_dist_coeff = this->m_dist_coeff;
		Mat cam_cam_mat = p_cam->getCameraMatrix();
		Mat cam_dist_coeff = p_cam->getDistortCoeff();
		Mat fund_mat, essen_mat;

		using cv::TermCriteria;

		double rms = cv::stereoCalibrate(m_obj_pts, p_cam->getImagePoints(), this->m_img_pts, cam_cam_mat, cam_dist_coeff, proj_cam_mat, proj_dist_coeff, m_img_size, m_r3x3_proj2cam, m_t3x1_proj2cam, essen_mat, fund_mat, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6), cv::CALIB_FIX_INTRINSIC);
		cout << endl;
		cout << "Camera-projector extrinsic computed: " << endl;
		cout << "rotation mat: " << endl << m_r3x3_proj2cam << endl << endl;
		cout << "translation vec:" << endl << m_t3x1_proj2cam << endl << endl;
		m_ready = cv::checkRange(m_r3x3_proj2cam) && cv::checkRange(m_t3x1_proj2cam);
		
		return m_ready;
	}

	bool ProjectorCalibration::computeObjectPoints(const CalibrationBase* p_cam, const vector<Point2f>& cam_img_pts, vector<Point3f>& object_pts)
	{
		object_pts.clear();

		//ray-plane intersection
		Mat cam_int_mat = p_cam->getCameraMatrix().clone();
		Mat cam_dist_coef = p_cam->getDistortCoeff().clone();
		Mat cam_ext_mat(3, 3, CV_32FC1);
		
		vector<Point2f> undis_img_pts;
		vector<Point3f> undis_img_pts_h;

		undis_img_pts.clear();
		undis_img_pts_h.clear();

		int num_pts = cam_img_pts.size();
		if (checkRange(m_r3x3_board) && checkRange(m_t3x1_board))
		{
			m_r3x3_board.col(0).copyTo(cam_ext_mat.col(0));
			m_r3x3_board.col(1).copyTo(cam_ext_mat.col(1));
			m_t3x1_board.col(0).copyTo(cam_ext_mat.col(2));
		}
		else
		{
			cout << "No Board pose: compute board pose first" << endl;
			m_ready = false;
			return false;
		}

		//undistort 2d points: cam_img_pts
		cv::undistortPoints(cam_img_pts, undis_img_pts, cam_int_mat, cam_dist_coef);
		cv::convertPointsToHomogeneous(undis_img_pts, undis_img_pts_h);
		
		Mat undis_img_pt_h(3, 1, CV_32FC1);
		Mat mult(3, 1, CV_32FC1);
		Point3f temp_pt;

		for (int i = 0; i < num_pts; i++)
		{
			undis_img_pt_h = Mat(undis_img_pts_h[i]).reshape(1); 

			mult = cam_ext_mat.inv() * undis_img_pt_h;
			
			if (!mult.at<float>(2, 0))
				return false;		
			
			temp_pt.x = mult.at<float>(0, 0) / mult.at<float>(2, 0);
			temp_pt.y = mult.at<float>(1, 0) / mult.at<float>(2, 0);
			temp_pt.z = 0.f;
			object_pts.push_back(temp_pt);		
		}
		cout << endl;
		cout << "Compute" << std::to_string(num_pts) <<" blob 3d coordinates" << endl;
		m_ready = !object_pts.empty();
		return m_ready;
	}

	bool ProjectorCalibration::computeDynamicProjection(const vector<Point3f>& object_point, const Mat& board_r, const Mat& board_t)
	{
		const int num_pts = object_point.size();
		vector<Point3f> cam_point(num_pts);

		//transform obj pts from model coordinate to camera coordinate
		for (int i = 0; i < num_pts; i ++)
		{
			Mat temp = board_r * Mat(object_point[i]) + board_t;
			cam_point[i].x = temp.at<double>(0);
			cam_point[i].y = temp.at<double>(1);
			cam_point[i].z = temp.at<double>(2);
		}

		m_dynamic_quad.clear();

		//project obj pts from camera coordinate to 2d projector coordinate
		projectPoints(cam_point, m_r3x3_proj2cam, m_t3x1_proj2cam, m_cam_mat, m_dist_coeff, m_dynamic_quad);

		m_ready = !m_dynamic_quad.empty();
		return m_ready;
	}



	void ProjectorCalibration::preProcessImage(const cv::Mat& src_img, cv::Mat& proc_img, int thre)
	{
		if (src_img.type() != CV_8UC1) {
			cvtColor(src_img, proc_img, CV_RGB2GRAY);
		}
		else {
			proc_img = src_img;
		}
		cv::threshold(proc_img, proc_img, thre, 255, cv::THRESH_BINARY_INV);
	}

	void ProjectorCalibration::createImagePoints()
	{
		m_pattern_pts.clear();

		for (int i = 1; i <= m_pattern_size.height; ++i)
			for (int j = 1; j <= m_pattern_size.width; ++j)
			{
				m_pattern_pts.push_back(Point2f(float(j * m_dpattern_pixel.width),
					float(i * m_dpattern_pixel.height)));
			}
	}

	void ProjectorCalibration::setPatternPixel(float dwidth_pixel, float dheight_pixel)
	{
		m_dpattern_pixel.width = dwidth_pixel;
		m_dpattern_pixel.height = dheight_pixel;
	}

	void ProjectorCalibration::setStaticFrame(uint static_frame)
	{
		m_static_frame = static_frame;
	}

}