#include <opencv2/opencv.hpp>
#include "lm_optim_jacobians.h"

/* todo explain input output*/
/*
% p = [fx_p, fy_p, cx_p, cy_p, //1-4
%       om1, om2, om3, //5-7
%       t1, t2, t3, //8-10
%       a, b, c, r, //11-14
%       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //15-23
% p has to be a column vector
% Return:
%       A : NumberOfBlobs x 10->projector intrinsic and extrinsic
%       B : NumberOfBlobs x 4->sphere pose
%       C : NumberOfBlobs x 9->camera intrinsics(with lens distortions)
% X : 3 x NumberOfBlobs ->3d pts
*/

extern void function_mincost( const std::vector<float>& params, std::vector<cv::Point2f>& cam_pts, std::vector<cv::Point2f>& proj_pts, std::vector<double>& err, bool needJac, cv::Mat_<double>& jac_A = cv::Mat_<double>(), cv::Mat_<double>& jac_B = cv::Mat_<double>(), cv::Mat_<double>& jac_C = cv::Mat_<double>())
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
				J_dRdom.at<double>(3*i+j, k) = temp.at<float>(3*j + i, k);

	Mat_<float> T =( Mat_<float>(3, 1, CV_32FC1) << params[7], params[8], params[9]); // projector extrinsic position
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
	
	Mat_<float> R_t; 
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

	Mat_<float> a = Mat::zeros(1,N, CV_32FC1); // 1 by N

	for (int i = 0; i < V.rows; i++)
	{
		cv::pow(V.row(i), 2, tempMat);
		a += tempMat;
	}
	tempMat.release();

	transpose(C_S, tempMat);
	Mat_<float> b = 2 * tempMat * V; // 1 by N
	tempMat.release();

	float c = std::pow(norm(C_S), 2) - std::pow(r,2);

	Mat_<float> b2_4ac(1, N, CV_32FC1); // 1 by N
	cv::pow(b, 2, tempMat);
	b2_4ac = tempMat - 4 * a * c; 
	tempMat.release();

	// TODO: remove the points if not intersect
	//double minP, maxP;
	//Point minIdx, maxIdx;
	//cv::minMaxLoc(b2_4ac, &minP, &maxP, &minIdx, &maxIdx);
	//while ( minP < 0)
	//{
	//	cam_pts.erase(cam_pts.begin()+ minIdx.x);
	//	proj_pts.erase(proj_pts.begin() + minIdx.x);
	//	
	//	N--;
	//	cv::minMaxLoc(b2_4ac, &minP, &maxP, &minIdx, &maxIdx);
	//}

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
		float r2 = std::pow(x1,2) + std::pow(y1,2);
		float r4 = std::pow(r2, 2);
		float r6 = r2 * r4;
		float x2 = x1 * (1 + kc[0] * r2 + kc[1]*r4 + kc[2]*r6) + 
					2 * pc[0] * x1 * y1 + 
					pc[1] * (r2 + 2 * std::pow(x1, 2)); 
		float y2 = y1 * (1 + kc[0] * r2 + kc[1]*r4 + kc[2]*r6) +
					2 * pc[1] * x1 * y1 + 
					pc[0] * (r2 + 2 * std::pow(y1, 2)); 
		x_cam_est[i].x = fc[0]*x2 + cc[0]; 
		x_cam_est[i].y = fc[1]*y2 + cc[1]; 
	}

	/// %%% compute error %%%
	err.resize(N * 2);
	for (int i = 0; i < err.size(); i++)
	{
		err[i] = (i%2==0) ? x_cam_est[i/2].x - cam_pts[i / 2].x : x_cam_est[i/2].y - cam_pts[i / 2].y;
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