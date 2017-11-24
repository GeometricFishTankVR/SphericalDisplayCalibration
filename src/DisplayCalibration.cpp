#include "DisplayCalibration.h"

namespace multi_proj_calib
{
	using namespace cv;
	using std::cout;
	using std::endl;
	using std::to_string;

	bool DisplayCalibration::setup()
	{
		uint buf_width(0), buf_height(0);

		/* camera setup */
		m_camera.getImgSize(buf_width, buf_height);
		/* tweak these params based on the lighting condition */
		m_camera.setParams("exposure", 0.f);
		m_camera.setParams("brightness", 0.f);
		m_camera.setParams("shutter", 8.0f);
		m_camera.setParams("gain", 0.f);

		if (m_camera.startCap()){ 
			throw std::runtime_error("DisplayCalibration::setup(): Fail to start camera capture. ");
		}

		/* projectors setup */
		m_projectors.initPattern(render::ONE_CIRCLE);

		/* camera calib setup */
		m_cam_calib.resetData();
		m_cam_calib.setImageParams(buf_width, buf_height);
		if (!m_cam_calib.loadCalibParams(file::data_path + file::camcalib_file)) {
			throw std::runtime_error("DisplayCalibration::setup(): Fail to load camera intrinsic file. ");
		}

		/* projector calib setup */
		m_proj_calib.clear();
		m_proj_calib.resize(m_num_proj);
		for (uint i = 0; i < m_num_proj; i++)
		{
			m_proj_calib[i].resetData();
			m_proj_calib[i].setImageParams(setting::proj_width, setting::proj_height);
			if (m_calibMethod == dp_calib::SemiAuto) {
				if (!m_proj_calib[i].loadCalibParams(file::data_path + file::projcalib_file[i]))
					throw std::runtime_error("DisplayCalibration::setup(): Fail to load projector intrinsic file. ");
			}
		}

		/* display calib setup */
     	m_curr_proj = 1;
		m_MaxReprojErr = 3;

		m_cam_pts.clear();
		m_proj_pts.clear();
		m_obj_pts.clear();

		m_reproj_err.clear();
		m_reproj_err.resize(m_num_proj);

		m_pixelarray.clear();
		m_pixelarray.resize(m_num_proj+1);

		for (unsigned int i = 1; i <= m_num_proj; i++)
			m_pixelarray[i].m_alpha_mask = Mat::ones(Size(setting::proj_width, setting::proj_height), CV_32FC1);

		m_cvwindow = "display calibration camera view";
		namedWindow(m_cvwindow);
		return true;
	}

	bool DisplayCalibration::cleanup()
	{
		int err = m_camera.stopCap();
		if (err != 0) { std::cout << "DisplayCalibration::cleanup(): Fail to stop camera." << std::endl; }
		m_projectors.cleanRender();
		std::cout << std::endl;
		std::cout << ">>Calibration ended. [Press any key] to exit." << std::endl;
		if (cv::waitKey(0) != -1)
		{
			return true;
		}
		return false;
	}

	bool DisplayCalibration::testWindowSequence()
	{
		using namespace dp_calib;

		m_projectors.cleanRender();
		m_projectors.initPattern(render::ONE_CIRCLE);

		bool keep_running = true;
		bool found = false;

		uint buf_width(0), buf_height(0);
		m_camera.getImgSize(buf_width, buf_height);

		Mat img_buf(buf_height, buf_width, setting::cv_pixel_format);
		uint buf_size = buf_width * buf_height;

		std::vector<int> projector_order(m_num_proj);

		for (uint i = 0; i < m_num_proj; i++)
		{
			Projector proj_name = (Projector) (i + 1);
			m_projectors.startProj(proj_name);
			
			m_mode = start;
			keep_running = true;

			/* start to project and detect */
			try {
				while (keep_running)
				{
					if (m_camera.grabImg(img_buf.data, buf_size)) {
						throw std::runtime_error("DisplayCalibration::calibratePair(): Fail to copy image buffer. ");
					}

					char ch;
					switch (m_mode)
					{
					case start:
						cout << endl;
						cout << "Start to display a blob on activated projector " << proj_name << endl;
						cout << ">>[Press Space] if the Projector Number is right; Or [Press the Correct Number] if wrong. " << endl;
						cvWaitKey(20);
						m_mode = project;
						break;
					case project:
						m_projectors.setBlobPos(600, 400);
						ch = cvWaitKey(20);
						if (ch == ' ')
						{
							projector_order[i] = (int)proj_name;
							m_mode = finish;
						}
						else if (ch - '0' > 0 && ch - '0' <= (int)m_num_proj)
						{
							projector_order[i] = ch - '0';
							m_mode = finish;
						}
						break;
					case finish:
						cout << "Set projector " << proj_name << "  to  " << projector_order[i] << endl;
						m_projectors.setBlobPos(0.f, 0.f);
						m_projectors.clearProj(proj_name);
						keep_running = false;
						break;
					default:
						break;
					}
					m_projectors.projPattern(proj_name);
					m_projectors.projFlush(proj_name);
					cv::imshow(m_cvwindow, img_buf);
				} /* end of loop */				
			}
			catch (const std::runtime_error& e){
				std::cerr << e.what() << std::endl;
			}
			m_projectors.stopProj(proj_name);
		}
		if (!m_projectors.setProjectorSequence(projector_order)) {
			throw std::runtime_error("DisplayCalibration::calibratePair(): Fail to swap projectors. ");
		}
		return true;		
	}

	bool DisplayCalibration::calibratePair()
	{		
		using namespace dp_calib;

		/// setup mouse callback function for bounding circle detection
		cv::setMouseCallback(m_cvwindow, pairCalibOnMouse, this);

		m_projectors.cleanRender();
		m_projectors.initPattern(render::ONE_CIRCLE);
	
		uint buf_width(0), buf_height(0);
		m_camera.getImgSize(buf_width, buf_height);
		
		Mat img_buf(buf_height, buf_width, setting::cv_pixel_format);
		uint buf_size = buf_width * buf_height;

		m_blobs.setupBlobDetector();
		m_blobs.setTotalBlobCnt(setting::blob_count);
		m_blobs.resetBlobs();

		m_mode = idle;

		bool keep_running = true;
		bool found = false;
		bool ready = false;

		Projector proj_name = (Projector)m_curr_proj;
		m_projectors.startProj(proj_name);

		if (!m_blobs.isBoundCircleFound()) 
		{
			cout << endl;
			cout << ">>[Single Click Left MouseButton] for FOUR POINTS on the boundary circle to start" << endl;
			cout << ">>[Double Click Right MouseButton] to quit" << endl;
		}
		else
		{
			if (m_curr_proj <= m_num_proj)
				cout << ">>[Double Click Left MouseButton] to proceed to projector" << to_string(m_curr_proj)<< endl;
		}

		try {
			/* start to project and detect */
			while (keep_running)
			{
				if (m_camera.grabImg(img_buf.data, buf_size))
				{
					throw std::runtime_error("DisplayCalibration::calibratePair(): Fail to copy image buffer");
				}
				Point2f proj_blob;
				switch (m_mode)
				{
				case start:
					/// set the bounding circle by clicking four points on the circle
					if (!m_blobs.isBoundCircleFound())
					{
						found = m_blobs.detectBoundingCircle();
						if (found)
						{
							m_mode = project;
							m_blobs.drawBoundingCircle(img_buf);
							m_blobs.setBackGroundImg(img_buf);
						}
						else
							m_mode = start;
					}
					else
					{
						m_mode = project;
						m_blobs.drawBoundingCircle(img_buf);
						m_blobs.setBackGroundImg(img_buf);
					}
					break;
				case project:
					proj_blob = m_blobs.generateBlobGrid();
					m_projectors.setBlobPos(proj_blob.x, proj_blob.y);
					cout << proj_blob << endl;
					m_mode = dp_calib::detect;
					break;
				case dp_calib::detect:
					found = m_blobs.detectSingleBlob(img_buf);
					if (found)
					{
						m_blobs.addBlob();
						m_blobs.drawDetectedBlob(img_buf);
					}
					else
						cout << "blob not detected" << endl;

					if (m_blobs.getCurrBlobIdx().width >= m_blobs.getGridBlobSize().width && m_blobs.getCurrBlobIdx().height >= m_blobs.getGridBlobSize().height)
					{
						found = m_blobs.cleanBlobs();
						if (found)
						{
							m_blobs.saveBlobData(file::data_path+file::blob_file[proj_name - 1]);
							cout << to_string(m_blobs.getCurrBlobCnt()) + " blobs saved in xml file" << endl;
							ready = true;
						}
						else
							cout << "DisplayCalibration::calibratePair(): error in cleanBlobs()" << endl;
						m_mode = finish;
					}
					else
						m_mode = project;
					break;
				case finish:
					m_projectors.setBlobPos(0.f, 0.f);
					m_projectors.clearProj(proj_name);
					keep_running = false;
					break;
				case idle:
					break;
				default:
					break;
				}
				cv::imshow(m_cvwindow, img_buf);
				m_camera.showControlDlg();

				m_projectors.projPattern(proj_name);
				m_projectors.projFlush(proj_name);

				cvWaitKey(85); // syc problem arises when delay is small
			} /* end of loop */
		}
		catch (const std::runtime_error& e)
		{
			std::cerr << e.what() << std::endl;
			ready = false;
		}
		m_projectors.stopProj(proj_name);
		cv::setMouseCallback(m_cvwindow, NULL, NULL);

		return ready;
	}

	bool DisplayCalibration::estimatePairExtrinsic()
	{
		cout << endl;
		cout << "===== estimate initial guess of projector "<< to_string(m_curr_proj) << " extrinsics =====" << endl;
		
		if (m_calibMethod != dp_calib::SemiAuto)
		{
			cout << "DisplayCalibration::estimatePairExtrinsic(): not in SemiAuto-Calibration mode. Can't estimate extrinsics." << endl;
			return false;
		}
		const uint proj_idx = m_curr_proj - 1;

		Mat cam_cam_mat = m_cam_calib.getCameraMatrix();
		Mat cam_dist_coeff = m_cam_calib.getDistortCoeff();

		Mat proj_cam_mat = m_proj_calib[proj_idx].getCameraMatrix();
		Mat proj_dist_coeff = m_proj_calib[proj_idx].getDistortCoeff();

		uint num_pts = m_blobs.getCurrBlobCnt();
		
		vector<Point2f> cam_pts = m_blobs.getCamBlobs();
		vector<Point2f> proj_pts = m_blobs.getProjBlobs();
		vector<Point3f> obj_pts(num_pts);

		vector<Point2f> cam_blobs_nml(num_pts);
		vector<Point2f> proj_blobs_nml(num_pts);

		cv::undistortPoints(cam_pts, cam_blobs_nml, cam_cam_mat, cam_dist_coeff);
		cv::undistortPoints(proj_pts, proj_blobs_nml, proj_cam_mat, proj_dist_coeff);

		/* recover extrinsic from fundamental mat */

		stereo_recon::FundamentalMat fmat;
		
		Mat essen_mat;
		Mat S, U, Vt;

		// find fundamental mat
		fmat.setThresholdRatio(.99f);
		essen_mat = fmat.findEssentialMat(cam_blobs_nml, proj_blobs_nml, stereo_recon::Sampson);

		essen_mat.convertTo(essen_mat, CV_32FC1);
		SVD::compute(essen_mat, S, U, Vt);

		Mat_<float> W = (Mat_<float>(3, 3) << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f);

		/* rotation: two alternatives */
		vector<Mat_<float>> est_rmat(2);
		vector<Mat_<float>> est_rvec(2);

		est_rmat[0] = U * W * Vt;
		est_rmat[1] = U * W.t() * Vt;

		if ( fabs( fabs( determinant(est_rmat[0]) ) - 1) > 1e-6)
		{
			cout << endl;
			cout << "DisplayCalibration::estimatePairExtrinsic() fails: determinant of rotation matrix not equal to 1." << endl;
			return false;
		}
	
		/// in case of negative result from essen_mat
		est_rmat[0] = utils::sgn(determinant(est_rmat[0])) * est_rmat[0];
		est_rmat[1] = utils::sgn(determinant(est_rmat[1])) * est_rmat[1];

		cv::Rodrigues(est_rmat[0], est_rvec[0]);
		cv::Rodrigues(est_rmat[1], est_rvec[1]);
		
		cout << endl;
		cout << "estimated rotation mat1 det:" << determinant(est_rmat[0]) << endl << est_rmat[0] << endl << endl;
		cout << "estimated rotation mat2 det:" << determinant(est_rmat[1]) << endl << est_rmat[1] << endl << endl;

		/* translation: two alternatives + or - */
		Mat_<float> est_tvec(3, 1, CV_32FC1);
		est_tvec = U.col(2);

		cout << "estimated translation: + or - sign" << endl << est_tvec << endl << endl;

		/* triangulation to pick up the correct solution out of four alternatives: z > 0 */

		Mat_<float> cam_proj_mat(3, 4, CV_32FC1);
		Mat_<float> cam_tvec = Mat::zeros(3, 1, CV_32FC1);
		Mat_<float> cam_rmat = Mat::eye(Size(3, 3), CV_32FC1);
		cv::hconcat(cam_rmat, cam_tvec, cam_proj_mat);

		const int num_sols = 4; // four solutions in total
		vector<Mat_<float>> proj_proj_mat(num_sols);
		cv::hconcat(est_rmat[0], est_tvec, proj_proj_mat[0]);
		cv::hconcat(est_rmat[0], -est_tvec, proj_proj_mat[1]);
		cv::hconcat(est_rmat[1], est_tvec, proj_proj_mat[2]);
		cv::hconcat(est_rmat[1], -est_tvec, proj_proj_mat[3]);

		int sol_num(5); 
		for (int i = 0; i < num_sols; i++)
		{
			bool sol = triangulatePairFeature(cam_blobs_nml, cam_proj_mat, proj_blobs_nml, proj_proj_mat[i], obj_pts);
			if (sol)
			{
				sol_num = i;
				break;
			}
		}

		// if find solution
		if ( sol_num < num_sols )
		{
			cout << endl;
			cout << "extrinsic solution found: solution #" << to_string(sol_num) << endl;
			
			/* compute reprojection error of triangulation: in two views */
			Mat_<float> cam_rvec;
			cv::Rodrigues(cam_rmat, cam_rvec);

			Mat_<float> proj_rmat = proj_proj_mat[sol_num](Range(0, 3), Range(0, 3));
			Mat_<float> proj_tvec = proj_proj_mat[sol_num].col(3);
			Mat_<float> proj_rvec;
			cv::Rodrigues(proj_rmat, proj_rvec);

			vector<Point2f> cam_pts2, proj_pts2;
			projectPoints(obj_pts, cam_rvec, cam_tvec, cam_cam_mat, cam_dist_coeff, cam_pts2);
			projectPoints(obj_pts, proj_rvec, proj_tvec, proj_cam_mat, proj_dist_coeff, proj_pts2);
			
			double cam_err = norm(cam_pts, cam_pts2, CV_L2);
			double proj_err = norm(proj_pts, proj_pts2, CV_L2);
			double reproj_err = std::sqrt((cam_err*cam_err + proj_err*proj_err) /2/ num_pts);
			
			cout << endl;
			cout << "reprojection error:" << reproj_err << endl;

			// compute per-blob reproj error: use to estimate sphere position
			m_reproj_err[proj_idx].clear();

			for (uint i = 0; i < num_pts; i++)
			{
				float cam_d = norm(cam_pts[i] - cam_pts2[i]);
				float proj_d = norm(proj_pts[i] - proj_pts2[i]);
				m_reproj_err[proj_idx].push_back( (cam_d * cam_d + proj_d * proj_d)/2.f);
			}

			// if reprojection error is below the threshold
			if (reproj_err < m_MaxReprojErr )
			{
				m_cam_pts.push_back(cam_pts);
				m_proj_pts.push_back(proj_pts);
				m_obj_pts.push_back(obj_pts);
				m_proj_calib[proj_idx].setExtrinsic(proj_rmat, proj_tvec);
				return true;
			}
			else
			{
				cout << "DisplayCalibration::estimatePairExtrinsic() fails: reprojection error larger than the threshold of" << m_MaxReprojErr <<" : restart by [double click mouse left button]" << endl;
				return false;
			}
		}
		// if does not find valid solution
		else
		{
			cout << "DisplayCalibration::estimatePairExtrinsic() fails: no extrinsic solution found, all points are behind two cameras." << endl;
			return false;
		}
	}

	bool DisplayCalibration::updateParams()
	{
		FileStorage fs(file::data_path+file::optimparam_file, FileStorage::READ);

		if (!fs.isOpened())
		{
			throw std::runtime_error("Failed to open "+file::optimparam_file);
		}

		cout << endl;
		cout << "Update optimized parameters" << endl;

		for (unsigned int i = 0; i < m_num_proj; i++)
		{
			string param_name = "proj" + std::to_string(i) + "_param";
			Mat_<float> proj_param;
			fs[param_name] >> proj_param;
			
			/* projector intrinsics */
			Mat_<float> proj_KK = (Mat_<float>(3, 3) << proj_param(0), 0.f, proj_param(2),
				0.f, proj_param(1), proj_param(3),
				0.f, 0.f, 1.f);
			Mat_<float> proj_dist = (Mat_<float>(5, 1) << 0.f, 0.f, 0.f, 0.f, 0.f);
			m_proj_calib[i].updateCameraMatrix(proj_KK);
			m_proj_calib[i].updateDistortCoeff(proj_dist);

			/* projector extrinsics */
			Mat_<float> est_rvec = (Mat_<float>(3, 1) << proj_param(4), proj_param(5), proj_param(6));
			cv::Rodrigues(est_rvec, m_proj_calib[i].getExtrinsicRmat());
			m_proj_calib[i].getExtrinsicTvec() = (Mat_<float>(3, 1) << proj_param(7), proj_param(8), proj_param(9));
		}
		
		/* sphere pose */
		Mat_<float> sphere_param;		
		fs["sphere_pose"] >> sphere_param;

		m_sphere_pose = (Mat_<float>(4, 1) << sphere_param(0), sphere_param(1), sphere_param(2), sphere_param(3));

		/* camera */
		Mat_<float> cam_param;
		fs["cam_param"] >> cam_param;

		Mat_<float> cam_KK = (Mat_<float>(3, 3) << cam_param(0), 0.f, cam_param(2),
			0.f, cam_param(1), cam_param(3),
			0.f, 0.f, 1.f);
		Mat_<float> cam_dist = (Mat_<float>(5, 1) << cam_param(4), cam_param(5), cam_param(7), cam_param(8), cam_param(6));
		m_cam_calib.updateCameraMatrix(cam_KK);
		m_cam_calib.updateDistortCoeff(cam_dist);

		fs.release();
		return true;
	}

	void DisplayCalibration::computePixel3D()
	{		
		uint i(0);

		while (i <= m_num_proj)
		{
			CalibrationBase* pCalib;
			Point2i start_pixel = Point2i(1, 1);
			Point2i end_pixel;
			
			cout << endl;

			if (i == 0) { /// camera
				pCalib = &m_cam_calib;
				end_pixel = Point2i(setting::cam_width, setting::cam_height);
				cout << "===== computing pixel position of camera " << "... =====" << endl;
			}
			else { /// projectors
				pCalib = &m_proj_calib[i - 1];
				end_pixel = Point2i(setting::proj_width, setting::proj_height);
				cout << "===== computing pixel position of projector " << to_string(i) << "... =====" << endl;
			}

			m_pixelarray[i].m_deviceID = i;

			m_pixelarray[i].raySphereIntersection(pCalib, m_sphere_pose, start_pixel, end_pixel);

			i++;
		}
	}

	bool DisplayCalibration::displayCorrectedPattern()
	{
		using namespace dp_calib;

		cv::setMouseCallback(m_cvwindow, displayOnMouse, this);

		m_projectors.cleanRender();
		m_projectors.initPattern(render::TEXTURE);

		for (uint i = 1; i <= m_num_proj; i++)
		{
			// convert calibrated projector data to texture data
			m_projectors.loadCalibResult(m_pixelarray[i].m_pixel_pts, m_pixelarray[i].m_alpha_mask, (Projector)i);
			// start projecting
			m_projectors.startProj((Projector)i);
		}

		// prepare camera data ready to shader
		Mat KK, dist_coeff;
		KK = m_cam_calib.getCameraMatrix().clone();
		dist_coeff = m_cam_calib.getDistortCoeff().clone();
		
		KK.convertTo(KK, CV_32FC1);
		dist_coeff.convertTo(dist_coeff, CV_32FC1);
		
		m_projectors.setCameraIntrinsic(KK, dist_coeff);
		
		// camera capture setup 
		uint buf_width(0), buf_height(0);
		m_camera.getImgSize(buf_width, buf_height);

		Mat img_buf(buf_height, buf_width, setting::cv_pixel_format);
		uint buf_size = buf_width * buf_height;

		// fsm flags
		bool keep_running = true;
		m_mode = start;
		m_curr_proj = 0;

		cout << endl << "Display a calibrated grid pattern. " << endl << ">>[Double Right Click] to exit. " << endl;

		while (keep_running)
		{
			
			if (m_camera.grabImg(img_buf.data, buf_size)) { 
				throw std::runtime_error("DisplayCalibration::displayCorrectedPattern(): Fail to copy image buffer");
			}

			switch (m_mode)
			{
			case start:
				for (uint i = 1; i <= m_num_proj; i++)
					m_projectors.projPattern((Projector)i);
				break;
			case finish:
				keep_running = false;
				break;
			}

			cv::imshow(m_cvwindow, img_buf);
			m_camera.showControlDlg();
			m_projectors.projFlush();
		}
		for (uint i = 1; i <= m_num_proj; i++)
			m_projectors.stopProj((Projector)i);

		cv::setMouseCallback(m_cvwindow, NULL, NULL);
		return true;
	}

	bool DisplayCalibration::saveExtrinsics(const std::string& file_name)
	{
	
		FileStorage fs(file_name, FileStorage::WRITE);

		for (unsigned int proj_idx = 0; proj_idx < m_num_proj; proj_idx++)
		{
			Mat_<float> Rmat = m_proj_calib[proj_idx].getExtrinsicRmat().clone();
			Mat_<float> Tvec = m_proj_calib[proj_idx].getExtrinsicTvec().clone();

			fs << "proj_" + to_string(proj_idx) + "_Rmat" << Rmat;
			fs << "proj_" + to_string(proj_idx) + "_Tvec" << Tvec;
		}
		fs << "sphere_pose" << m_sphere_pose;

		cout << endl;
		cout << "save extrinsic of projectors and spehre pose to: " << file_name << endl;
		return true;
	}

	double DisplayCalibration::estimateSpherePose()
	{
		/// check if all data are ready
		if (m_obj_pts.size() != m_num_proj || m_reproj_err.size() != m_num_proj)
			throw std::runtime_error("DisplayCalibration::estimateSpherePose() fail: m_obj_pts or m_reproj_err does not contain enough data.");
		
		cout << endl;
		cout << "===== estimating sphere pose =====" << endl;
		vector<Mat_<double>> sphere_pose(m_num_proj);
		double residue = linearWLSSpherePose(m_obj_pts[0], m_reproj_err[0], sphere_pose[0]);
		cout << "proj1 sphere fit residue: " << residue << endl;
		if (m_num_proj > 1)
		{
			uint proj_idx(0);

			vector<Point3f> all_blobs;
			vector<float> all_err;

			while (proj_idx < m_num_proj)
			{
				if (!proj_idx) 
				{
					all_blobs.insert(all_blobs.begin(), m_obj_pts[proj_idx].begin(), m_obj_pts[proj_idx].end());
					all_err.insert(all_err.begin(), m_reproj_err[proj_idx].begin(), m_reproj_err[proj_idx].end());
				}
				else
				{
					residue = linearWLSSpherePose(m_obj_pts[proj_idx], m_reproj_err[proj_idx], sphere_pose[proj_idx]);
					cout << "proj" << proj_idx+1 << " sphere fit residue: " << residue << endl;

					Mat scale; 	// initial guess of scale between projectors
					solve(sphere_pose[proj_idx], sphere_pose[0], scale, DECOMP_SVD);
					scale.convertTo(scale, CV_32FC1);
					
					m_proj_calib[proj_idx].getExtrinsicTvec() = m_proj_calib[proj_idx].getExtrinsicTvec() *  scale.at<float>(0);

					vector<Point3f> scaled_blobs(m_obj_pts[proj_idx].size());
					vector<float> scaled_err(m_reproj_err[proj_idx].size());

					transform(m_obj_pts[proj_idx].begin(), m_obj_pts[proj_idx].end(), scaled_blobs.begin(), bind2nd(utils::mult_scalar<Point3f>(), scale.at<float>(0)));
					transform(m_reproj_err[proj_idx].begin(), m_reproj_err[proj_idx].end(), scaled_err.begin(), bind2nd(utils::mult_scalar<float>(), scale.at<float>(0)));

					all_blobs.insert(all_blobs.end(), scaled_blobs.begin(), scaled_blobs.end());
					all_err.insert(all_err.end(), scaled_err.begin(), scaled_err.end());
				}
				proj_idx++;
			}
			residue = linearWLSSpherePose(all_blobs, all_err, m_sphere_pose);
		}
		else
			m_sphere_pose = sphere_pose[0];

		cout << endl;
		cout << "sphere center: " << m_sphere_pose << endl << endl;
		cout << "sphere pose residue:" << residue << endl;
		cout << endl;
		if (residue > .1f) 
			cout << "Warning: sphere fit residue too large; bad blobs have been included; recalibrate it." << endl << endl;;
		return residue; 	//residue should be around than 0.05, no more than 0.1
	}

	void DisplayCalibration::computeAlphaMask()
	{
		vector<vector<Pixel3DArray>::iterator> pArrayIter;	
		pArrayIter.clear();
		/// construct vector of iterators for all projectors
		for (vector<Pixel3DArray>::iterator it = m_pixelarray.begin()+1; it < m_pixelarray.end(); it++)
		{
			pArrayIter.push_back(it);
		}
		/// compute alpha mask using all projector calibration and iteractors
		for (unsigned int p_idx = 1; p_idx <= m_num_proj; p_idx++)
		{
			m_pixelarray[p_idx].computeAlphaMask(m_proj_calib, pArrayIter);
		}
	}

	void DisplayCalibration::saveCalibrationResult(std::string dir)
	{
		string file_dir = dir.empty() ? file::data_path : dir;

		for (uint idx = 0; idx <= m_num_proj; idx++)
		{
			vector<Point3f> pixel_nml;

			m_pixelarray[idx].normalizePixelPointToSphereCenter(m_sphere_pose, pixel_nml);

			if (idx == 0) /// camera
			{
				m_pixelarray[idx].savePixel3DArray(file_dir + file::camgeom_file, "geom", pixel_nml);
			}
			else /// prjector
			{
				m_pixelarray[idx].savePixel3DArray(file_dir + file::geom_file[idx - 1], "geom", pixel_nml);
				m_pixelarray[idx].gammaCorrection(2.2f);
				m_pixelarray[idx].savePixel3DArray(file_dir + file::blen_file[idx - 1], "alpha");
			}			
		}
	}
	void DisplayCalibration::loadCalibrationResult(std::string dir)
	{
		string file_dir = dir.empty() ? file::data_path : dir;
		
		for (uint idx = 0; idx <= m_num_proj; idx++)
		{
			if (idx == 0) /// camera
			{
				m_pixelarray[idx].loadPixel3DArray(file_dir + file::camgeom_file, "geom");
			}
			else /// prjector
			{
				m_pixelarray[idx].loadPixel3DArray(file_dir + file::geom_file[idx - 1], "geom");
				m_pixelarray[idx].loadPixel3DArray(file_dir + file::blen_file[idx - 1], "alpha");
			}
		}
	}

	double DisplayCalibration::linearWLSSpherePose(const vector<Point3f>& blobs, const vector<float>& weight, Mat_<double>& sphere_pose )
	{
		CV_Assert(blobs.size() == weight.size());
		const uint num_pts = (int) blobs.size();

		Mat_<float> W = Mat::diag(Mat(weight)); // weight matrix

		Mat A;
		Mat XYZ = Mat(blobs).reshape(1);
		cv::hconcat(XYZ, Mat::ones(num_pts, 1, CV_32FC1), A);


		Mat B(num_pts, 1, CV_32FC1);
		for (uint i = 0; i < num_pts; i++)
		{
			B.at<float>(i) = -XYZ.at<float>(i, 0) * XYZ.at<float>(i, 0) - XYZ.at<float>(i, 1) * XYZ.at<float>(i, 1) - XYZ.at<float>(i, 2) * XYZ.at<float>(i, 2);
		}
		Mat_<float> X;
		
		Mat_<float> At = A.t();
		Mat_<float> inv_W = W.inv();

		X = (At * inv_W * A).inv() * At * inv_W * B; // linear weighted least square

		sphere_pose = Mat_<float>(4,1); 
		sphere_pose(0) = -X(0) / 2;
		sphere_pose(1) = -X(1) / 2;
		sphere_pose(2) = -X(2) / 2;
		sphere_pose(3) = std::sqrt(X(0)*X(0) + X(1)*X(1) + X(2)*X(2) - 4 * X(3)) / 2;

		// compute residue
		Point3f sphere_center = Point3f(sphere_pose(0), sphere_pose(1), sphere_pose(2));
		double residue(0);
		for (uint i = 0; i < num_pts; i++)
		{
			double dis = norm(blobs[i] - sphere_center) - sphere_pose(3);
			residue += dis * dis;
		}

		return std::sqrt(residue / num_pts);
	}

	bool DisplayCalibration::loadBlobData(const std::string& file_name)
	{
		Mat_<float> cam_blob, proj_blob;
		FileStorage fs(file_name, FileStorage::READ);

		if (!fs.isOpened())
		{
			throw std::runtime_error("DisplayCalibration::loadBlobData() Failed to open " + file_name);
		}

		fs["cam_pts"] >> cam_blob;
		fs["proj_pts"] >> proj_blob;

		cam_blob.reshape(2).copyTo(m_blobs.getCamBlobs());
		proj_blob.reshape(2).copyTo(m_blobs.getProjBlobs());

		CV_Assert(m_blobs.getCamBlobs().size() == m_blobs.getProjBlobs().size());

		fs.release();
		return !m_blobs.getCamBlobs().empty();
	}

	bool DisplayCalibration::triangulatePairFeature(const vector<Point2f>& cam_blobs_nml, const Mat& cam_proj_mat, const vector<Point2f>& proj_blobs_nml, const Mat& proj_proj_mat, vector<Point3f>& obj_pts)
	{
		CV_Assert(cam_blobs_nml.size() == proj_blobs_nml.size());
		const int num_pts = (int) cam_blobs_nml.size();
		Matx34d extrin_mat = proj_proj_mat;

		obj_pts.clear();
		for (int i = 0; i < num_pts; i++)
		{
			Point3d cam_pt_h(cam_blobs_nml[i].x, cam_blobs_nml[i].y, 1.);
			Point3d proj_pt_h(proj_blobs_nml[i].x, proj_blobs_nml[i].y, 1.);
			Mat_<double> X = linearLSTriangulation(cam_pt_h, cam_proj_mat, proj_pt_h, proj_proj_mat);
			Mat_<double> X1 = Mat(extrin_mat) * Mat(Matx41d(X(0), X(1), X(2), 1.));

			if (X(2) > 0 && X1(2) > 0)
				obj_pts.push_back(Point3f(X(0), X(1), X(2)));
			else
				return false;
		}
		return true;
	}

	Mat_<double> DisplayCalibration::linearLSTriangulation(Point3d u, Matx34d P, Point3d u1, Matx34d P1)
	{
		//u: homogenous image point (u,v,1)
		//P: camera 1 matrix
		//u1: homogenous image point in 2nd camera
		//P1: camera 2 matrix
		Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
			u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
			u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
			u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
		);
		Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
			-(u.y*P(2, 3) - P(1, 3)),
			-(u1.x*P1(2, 3) - P1(0, 3)),
			-(u1.y*P1(2, 3) - P1(1, 3)));

		Mat_<double> X;
		solve(A, B, X, DECOMP_SVD);

		return X;
	}

	/* callback functions */

	void dp_calib::displayOnMouse(int event, int x, int y, int flags, void* ptr)
	{
		DisplayCalibration* p_disp_calib = (DisplayCalibration*)ptr;
		if (event == EVENT_RBUTTONDBLCLK)
		{
			p_disp_calib->setFsmMode(finish);
		}
	}
	void dp_calib::pairCalibOnMouse(int event, int x, int y, int flags, void* ptr)
	{
		DisplayCalibration* p_disp_calib = (DisplayCalibration*)ptr;
		
		if (event == EVENT_LBUTTONDBLCLK)
		{
			cout << endl;
			cout << "===== detect and project patterns of projector" << to_string(p_disp_calib->getCurrProjector()) << "=====" << endl;
			p_disp_calib->setFsmMode(start);
		}
		else if (event == EVENT_RBUTTONDBLCLK)
		{
			cout << endl;
			cout << "===== end of detecting and projecting patterns of projector" << to_string(p_disp_calib->getCurrProjector()) << "=====" << endl;
			p_disp_calib->setFsmMode(finish);
			p_disp_calib->setCurrProjector(p_disp_calib->getTotalProjector() + 1);
		}
		else if (event == EVENT_LBUTTONDOWN)
		{
			if (!p_disp_calib->isBoundCircleFound())
			{
				p_disp_calib->setBoundPoint(x, y);
				p_disp_calib->setFsmMode(start);
			}
		}
	}
}