#include "LmOptimizer.h"

namespace multi_proj_calib
{	
	std::vector<std::vector<cv::Point2f>> LmOptimizer::m_campts = std::vector<std::vector<cv::Point2f>>();
	std::vector<std::vector<cv::Point2f>> LmOptimizer::m_projpts = std::vector<std::vector<cv::Point2f>>();

	int LmOptimizer::m_Nproj = 0;
	
	void LmOptimizer::runNonLinearOptimize(const std::vector<std::vector<cv::Point2f>>& cam_pts, const std::vector<std::vector<cv::Point2f>>& proj_pts, std::vector<double>& params)
	{
		m_Nproj = cam_pts.size();
		if (m_Nproj != proj_pts.size() || m_Nproj <= 0 || proj_pts.size() <=0)
		{
			throw std::runtime_error("LmOptimizer does not have correct points vector: cam_pts.size()!=proj_pts.size() or size() <= 0");
		}
		
		unsigned int Nparams = params.size();
		if (Nparams != 10 * m_Nproj + 13)
		{
			throw std::runtime_error("LmOptimizer does not have correct params input: params.size() != 10 * NumOfProj + 13");
		}

		m_campts.assign(cam_pts.begin(), cam_pts.end());
		m_projpts.assign(proj_pts.begin(), proj_pts.end());
		
		using namespace alglib;
		real_1d_array p_vec;
		p_vec.setcontent(Nparams, params.data());

		bool restartOptim = false;
		do {

			/// use the following code to set boundary of optimization if needed
			//real_1d_array bndl, bndu;
			//const double RATIO = 5;
			//bndl.setlength(m_Nparams);
			//bndu.setlength(m_Nparams);
			//for (int i = 0; i < m_Nparams; i++)
			//{
			//	bndl[i] = params[i] - std::abs(params[i]) * RATIO;
			//	bndu[i] = params[i] + std::abs(params[i]) * RATIO;
			//}

			const double epsx = 0.000000001;
			const ae_int_t maxits = 0;

			minlmstate state;
			minlmreport rep;

			unsigned int Nblobs = 0;
			for (int i = 0; i < m_Nproj; i++)
			{
				if (m_campts[i].size() > 0 && m_projpts[i].size() > 0)
					Nblobs += m_campts[i].size();
				else
					throw std::runtime_error("LmOptimizer does not have correct points input: cam_pts[i].size() or proj_pts[i].size() <=0");
			}

			try {
				minlmcreatevj(Nblobs * 2, p_vec, state);
				//minlmsetbc(state, bndl, bndu); // set boundary condition
				minlmsetacctype(state, 1);
				minlmsetcond(state, epsx, maxits); // set stop condition: If MaxIts=0, the number of iterations is unlimited.
				try {
					minlmoptimize(state, func_err, func_jac); // Running algorithm
				}
				catch (blob_not_intersect& e) // if blob not intersected with sphere: remove blobs and restart optim
				{
					std::cout << e.what() << std::endl << "restart optim" << std::endl;
					restartOptim = true;
					continue;
				}
				minlmresults(state, p_vec, rep); // obtain results
			}
			catch (alglib::ap_error& e) 
			{ 
				std::cerr << e.msg << std::endl; 
				throw std::runtime_error("LmOptimizer Fail to Optim. ");
			}
			
			std::cout << std::endl << "Optimization Complete. New param: " << std::endl << p_vec.tostring(6).c_str() << std::endl;
			restartOptim = false;

		} while (restartOptim);

		/// write back optimized params
		params.clear();
		for (int i = 0; i < p_vec.length(); i++)
			params.push_back(p_vec[i]);
	}
	/* 
	% params = [ fx_p1, fy_p1, cx_p1, cy_p1, //1-4 proj1 intrinsic
	%		 om11, om12, om13, //5-7 proj1 rotation
	%       t11, t12, t13, //8-10 proj1 translation
	%       fx_p2, fy_2, cx_p2, cy_p2, //11-14 proj2 intrinsic
	%       om21, om22, om23, //15-17 proj2 rotation
	%       t21, t22, t23, //18-20 proj2 translation
	%       ... //etc proj3...
	%       a, b, c, r, ///?1-?4 sphere pose
	%       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //?5-?3 camera intrinsic and lens distortion
	*/
	void LmOptimizer::func_err(const alglib::real_1d_array &params, alglib::real_1d_array &fi, void *ptr)
	{
		std::vector<double> all_err;
		all_err.clear();

		for (int pi = 0; pi < m_Nproj; pi++)
		{
			/// step 1: prepare params
			std::vector<float> param_pi;
			// get projector pi's 10 params
			for (int vi = pi * 10; vi < pi * 10 + 10; vi++)
			{
				param_pi.push_back(params[vi]);
			}
			// get camera and sphere 13 params 
			for (int vi = m_Nproj * 10; vi < m_Nproj * 10 + 13; vi++)
			{
				param_pi.push_back(params[vi]);
			}
			/// step 2: compute the error
			std::vector<double> err_pi;
			function_mincost(param_pi, m_campts[pi], m_projpts[pi], err_pi, false);
			/// step 3: attach to err array
			all_err.insert(all_err.end(), err_pi.begin(), err_pi.end());
		}
		/// step 4: transfer to local data type real_1d_array
		for (int i = 0; i < all_err.size(); i++)
		{
			fi[i] = all_err[i];
		}
		
		static unsigned int iter = 0;

		double fx_err = cv::norm(all_err);
		iter += 1;
		std::cout << "iter: "<<  iter << " norm_error:  " << fx_err << std::endl;
	}

	void LmOptimizer::func_jac(const alglib::real_1d_array &params, alglib::real_1d_array &fi, alglib::real_2d_array &jac, void *ptr)
	{
		std::vector<double> all_err;
		all_err.clear();
		cv::Mat_<double> Jac;
		for (int pi = 0; pi < m_Nproj; pi++)
		{
			/// step 1: prepare params
			std::vector<float> param_pi;
			// get projector pi's 10 params
			for (int vi = pi * 10; vi < pi * 10 + 10; vi++)
			{
				param_pi.push_back(params[vi]);
			}
			// get camera and sphere 13 params 
			for (int vi = m_Nproj * 10; vi < m_Nproj * 10 + 13; vi++)
			{
				param_pi.push_back(params[vi]);
			}
			/// step 2: compute the error and sub-jacobian matrix
			std::vector<double> err_pi;
			cv::Mat_<double> A, B, C;
			function_mincost(param_pi, m_campts[pi], m_projpts[pi], err_pi, true, A, B, C);
			/// step 3: attach to err array
			all_err.insert(all_err.end(), err_pi.begin(), err_pi.end());
			/// step 4: attach to overall jacobian
			cv::Mat_<double> J_A_zero;
			J_A_zero = cv::Mat::zeros(A.rows, m_Nproj * 10, CV_64FC1);
			cv::Mat_<double> subJ_A = J_A_zero(cv::Range(0, J_A_zero.rows), cv::Range(pi * 10, pi * 10 + 10));
			A.copyTo(subJ_A);

			cv::Mat_<double> J_pi;
			cv::hconcat(J_A_zero, B, J_pi);
			cv::hconcat(J_pi, C, J_pi);
			if (Jac.empty())
			{
				Jac = J_pi.clone();
			}
			else
				cv::vconcat(Jac, J_pi, Jac);
		}
		/// step 5: transfer to local data type real_1d_array: fi
		for (int i = 0; i < all_err.size(); i++)
		{
			fi[i] = all_err[i];
		}
		/// step 6: transfer to local data type real_1d_array: Jac
		for (int i = 0; i < Jac.rows; i++)
		{
			for (int j = 0; j < Jac.cols; j++)
			{
				jac[i][j] = Jac.at<double>(i, j);
			}
		}
	}


	/* Cost Function Per Projector: computes pixel errors in camera 2D image coordinates
	% p = [fx_p, fy_p, cx_p, cy_p, //1-4 projector intrinsic
	%       om1, om2, om3, //5-7 rotation
	%       t1, t2, t3, //8-10 translation
	%       a, b, c, r, //11-14 sphere pose 3 position a + 1 radius r
	%       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //15-23 camera intrinsics + lens distortion
	% Return:
	%       A : NumberOfBlobs x 10->Jacobian matrix for projector intrinsic and extrinsic
	%       B : NumberOfBlobs x 4->Jacobian matrix for sphere pose
	%       C : NumberOfBlobs x 9->Jacobian matrix for camera intrinsics(with lens distortions)
	%     err : 2 x NumberOfBlobs ->Jacobian matrix for pixel error
	*/

	void LmOptimizer::function_mincost(const std::vector<float>& params, std::vector<cv::Point2f>& cam_pts, std::vector<cv::Point2f>& proj_pts, std::vector<double>& err, bool needJac, cv::Mat_<double>& jac_A , cv::Mat_<double>& jac_B , cv::Mat_<double>& jac_C)
	{
		using namespace cv;

		if (params.size() != 23)
		{
			throw std::invalid_argument("cost function input param does not have size of 23. ");
		}
		float fp[2] = { params[0], params[1] }; // projector intrinsic focal length
		float cp[2] = { params[2], params[3] }; // projector intrinsic central point
		std::vector<float> om(params.begin() + 4, params.begin() + 7); // projector extrinsic rotation

		Mat_<float> R, temp;
		Mat_<double> J_dRdom;
		Rodrigues(om, R, temp);
		temp.convertTo(J_dRdom, CV_64FC1);
		transpose(J_dRdom, J_dRdom);
		temp = J_dRdom;
		// re-order J_dRdom due to OpenCV Rodrigues
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < 3; k++)
					J_dRdom.at<double>(3 * i + j, k) = temp.at<float>(3 * j + i, k);

		Mat_<float> T = (Mat_<float>(3, 1, CV_32FC1) << params[7], params[8], params[9]); // projector extrinsic position
		Mat_<float> S = (Mat_<float>(3, 1, CV_32FC1) << params[10], params[11], params[12]); // sphere pose position
		float r = params[13]; // sphere radius

		float fc[2] = { params[14], params[15] };
		float cc[2] = { params[16], params[17] };
		float kc[3] = { params[18], params[19], params[20] };
		float pc[2] = { params[21], params[22] };

		int N = cam_pts.size(); // total number of blobs

								/// %%% ray-sphere intersection %%%

		Mat_<float> KK_p = (Mat_<float>(3, 3, CV_32FC1) << fp[0], 0.f, cp[0],
			0.f, fp[1], cp[1],
			0.f, 0.f, 1.f);  // projector intrinsic matrix

		Mat_<float> R_t; // 3x3
		transpose(R, R_t);

		Mat_<float> C = -R_t * T; // center position of projector: 3x1
		Mat_<float> C_S = C - S; // 3x1

		Mat_<float> tempMat = KK_p * R;
		Mat_<float> temph_mat;

		vector<Point3f> proj_pts_h;
		convertPointsToHomogeneous(proj_pts, proj_pts_h);

		transpose(Mat(proj_pts_h).reshape(1, N), temph_mat);
		Mat_<float> V = tempMat.inv() * temph_mat; // 3 by N
		temph_mat.release();
		tempMat.release();

		Mat_<float> a = Mat::zeros(1, N, CV_32FC1); // 1 by N

		for (int i = 0; i < V.rows; i++)
		{
			cv::pow(V.row(i), 2, tempMat);
			a += tempMat;
		}
		tempMat.release();

		transpose(C_S, tempMat);
		Mat_<float> b = 2 * tempMat * V; // 1 by N
		tempMat.release();

		float c = std::pow(norm(C_S), 2) - std::pow(r, 2); // 1 by 1

		Mat_<float> b2_4ac(1, N, CV_32FC1); // 1 by N
		cv::pow(b, 2, tempMat);
		b2_4ac = tempMat - 4 * a * c;
		tempMat.release();

		for (int i = b2_4ac.total() - 1; i >= 0; i--)
		{
			if (b2_4ac.at<float>(i) < 0)
			{
				std::cout << "blob not intersected. blob #" << i << ". blob removed in camera and projector pts. " << std::endl;
				cam_pts.erase(cam_pts.begin() + i);
				proj_pts.erase(proj_pts.begin() + i);
			}
		}

		if (cam_pts.size() != N)
			throw blob_not_intersect();

		Mat_<float> lambda(1, N, CV_32FC1); // 1 by N
		cv::sqrt(b2_4ac, tempMat);
		cv::divide(tempMat - b, a, lambda);
		lambda = lambda / 2;
		tempMat.release();

		std::vector<Point3d> X(N, Point3d()); // 3 by N
		for (int i = 0; i < N; i++)
		{
			X[i].x = V.at<float>(0, i) * lambda.at<float>(i) + C.at<float>(0);
			X[i].y = V.at<float>(1, i) * lambda.at<float>(i) + C.at<float>(1);
			X[i].z = V.at<float>(2, i) * lambda.at<float>(i) + C.at<float>(2);
		}

		/// %%% camera projection %%%
		std::vector<Point2f> x_cam_est(N, Point2f());
		for (int i = 0; i < N; i++)
		{
			float x1 = X[i].x / X[i].z;
			float y1 = X[i].y / X[i].z;
			float r2 = std::pow(x1, 2) + std::pow(y1, 2);
			float r4 = std::pow(r2, 2);
			float r6 = r2 * r4;
			float x2 = x1 * (1 + kc[0] * r2 + kc[1] * r4 + kc[2] * r6) +
				2 * pc[0] * x1 * y1 +
				pc[1] * (r2 + 2 * std::pow(x1, 2));
			float y2 = y1 * (1 + kc[0] * r2 + kc[1] * r4 + kc[2] * r6) +
				2 * pc[1] * x1 * y1 +
				pc[0] * (r2 + 2 * std::pow(y1, 2));
			x_cam_est[i].x = fc[0] * x2 + cc[0];
			x_cam_est[i].y = fc[1] * y2 + cc[1];
		}

		/// %%% compute error %%%
		err.resize(N * 2);
		for (int i = 0; i < err.size(); i++)
		{
			err[i] = (i % 2 == 0) ? x_cam_est[i / 2].x - cam_pts[i / 2].x : x_cam_est[i / 2].y - cam_pts[i / 2].y;
		}
		/// %%% compute jacobian %%%
		if (needJac)
		{
			jac_A = Mat::zeros(2 * N, 10, CV_64FC1);
			jac_B = Mat::zeros(2 * N, 4, CV_64FC1);
			jac_C = Mat::zeros(2 * N, 9, CV_64FC1);

			for (int i = 0; i < N; i++)
			{
				Mat_<double> J_dxdX(3, 2, CV_64FC1),
					J_dxdfc(2, 2, CV_64FC1),
					J_dxdcc(2, 2, CV_64FC1),
					J_dxdkc(5, 2, CV_64FC1),
					J_dXdfp(2, 3, CV_64FC1),
					J_dXdcp(2, 3, CV_64FC1),
					J_dXdR(9, 3, CV_64FC1),
					J_dXdT(3, 3, CV_64FC1),
					J_dXdS(4, 3, CV_64FC1);
				J_dxdcc = Mat::eye(2, 2, CV_64FC1);

				float r11 = R(0, 0);
				float r12 = R(0, 1);
				float r13 = R(0, 2);
				float r21 = R(1, 0);
				float r22 = R(1, 1);
				float r23 = R(1, 2);
				float r31 = R(2, 0);
				float r32 = R(2, 1);
				float r33 = R(2, 2);

				lmoptim::JdxdX(X[i].x, X[i].y, X[i].z, fc[0], fc[1], cc[0], cc[1], kc[0], kc[1], kc[2], pc[0], pc[1], (double*)J_dxdX.data);
				lmoptim::Jdxdfc(X[i].x, X[i].y, X[i].z, fc[0], fc[1], cc[0], cc[1], kc[0], kc[1], kc[2], pc[0], pc[1], (double*)J_dxdfc.data);
				lmoptim::Jdxdkc(X[i].x, X[i].y, X[i].z, fc[0], fc[1], cc[0], cc[1], kc[0], kc[1], kc[2], pc[0], pc[1], (double*)J_dxdkc.data);

				lmoptim::JdXdfp(fp[0], fp[1], cp[0], cp[1], r11, r21, r31, r12, r22, r32, r13, r23, r33,
					T(0), T(1), T(2), S(0), S(1), S(2), r, proj_pts[i].x, proj_pts[i].y, (double*)J_dXdfp.data);
				lmoptim::JdXdcp(fp[0], fp[1], cp[0], cp[1], r11, r21, r31, r12, r22, r32, r13, r23, r33,
					T(0), T(1), T(2), S(0), S(1), S(2), r, proj_pts[i].x, proj_pts[i].y, (double*)J_dXdcp.data);
				lmoptim::JdXdR(fp[0], fp[1], cp[0], cp[1], r11, r21, r31, r12, r22, r32, r13, r23, r33,
					T(0), T(1), T(2), S(0), S(1), S(2), r, proj_pts[i].x, proj_pts[i].y, (double*)J_dXdR.data);
				lmoptim::JdXdT(fp[0], fp[1], cp[0], cp[1], r11, r21, r31, r12, r22, r32, r13, r23, r33,
					T(0), T(1), T(2), S(0), S(1), S(2), r, proj_pts[i].x, proj_pts[i].y, (double*)J_dXdT.data);
				lmoptim::JdXdS(fp[0], fp[1], cp[0], cp[1], r11, r21, r31, r12, r22, r32, r13, r23, r33,
					T(0), T(1), T(2), S(0), S(1), S(2), r, proj_pts[i].x, proj_pts[i].y, (double*)J_dXdS.data);

				transpose(J_dxdX, J_dxdX);
				transpose(J_dxdfc, J_dxdfc);
				transpose(J_dxdkc, J_dxdkc);
				transpose(J_dXdfp, J_dXdfp);
				transpose(J_dXdcp, J_dXdcp);
				transpose(J_dXdR, J_dXdR);
				transpose(J_dXdT, J_dXdT);
				transpose(J_dXdS, J_dXdS);

				Mat_<double> hcatMat;
				hconcat(J_dxdX*J_dXdfp, J_dxdX*J_dXdcp, hcatMat);

				hconcat(hcatMat, J_dxdX*J_dXdR*J_dRdom, hcatMat);
				hconcat(hcatMat, J_dxdX*J_dXdT, jac_A(Range(2 * i, 2 * i + 2), Range(0, jac_A.cols)));
				jac_B(Range(2 * i, 2 * i + 2), Range(0, jac_B.cols)) = J_dxdX*J_dXdS;

				hconcat(J_dxdfc, J_dxdcc, hcatMat);
				hconcat(hcatMat, J_dxdkc, jac_C(Range(2 * i, 2 * i + 2), Range(0, jac_C.cols)));
			}
		}
	}
	
}