#include "Pixel3DArray.h"

namespace multi_proj_calib
{
	using cv::Mat;
	using cv::Mat_;
	using cv::Size;
	using cv::Point2f;
	using cv::Point3f;

	using std::cout;
	using std::endl;
	using std::to_string;
	using std::vector;

	void Pixel3DArray::raySphereIntersection( CalibrationBase* pcalib, const cv::Mat_<double>& sphere_pose, const cv::Point2i& start_pixel, const cv::Point2i& end_pixel)
	{
		/// sphere center S
		if (sphere_pose.empty())
			throw std::runtime_error("Pixel3DArray::raySphereIntersection(): sphere_pose is null. ");
		Mat_<float> S = (Mat_<float>(3, 1) << sphere_pose(0), sphere_pose(1), sphere_pose(2));
		/// sphere radius r
		float r = sphere_pose(3);
		/// device intrinsic matrix
		Mat_<float> KK = pcalib->getCameraMatrix().clone();
		/// device lens distortion coefficient
		Mat_<float> dist_coeff = pcalib->getDistortCoeff().clone();
		/// device inversed Rotation Matrix
		Mat_<float> Rmat_inv;
		/// device projection center C
		Mat C(3, 1, CV_32FC1);
		/// device ray direction V
		Mat V(3, 1, CV_32FC1);

		if (m_deviceID == 0) /// camera
		{
			Rmat_inv = cv::Mat::eye(3, 3, CV_32FC1);
			C = cv::Mat::zeros(3, 1, CV_32FC1);
		}
		else /// projectors
		{
			if (ProjectorCalibration* pProjCalib = dynamic_cast<ProjectorCalibration*>(pcalib))
			{
				dist_coeff = pProjCalib->getDistortCoeff().clone();
				Rmat_inv = pProjCalib->getExtrinsicRmat().clone().inv();
				C = -Rmat_inv * pProjCalib->getExtrinsicTvec().clone();
			}
			else
				throw std::runtime_error("Pixel3DArray::raySphereIntersection() fails: can not cast ProjectorCalibration type. ");
		}


		Mat_<float> C_S = C - S;

		float a, b, c;
		c = C_S.dot(C_S) - r * r;

		int i_start(start_pixel.x), i_end(end_pixel.x);
		int j_start(start_pixel.y), j_end(end_pixel.y);

		if (i_start >= i_end && j_start >= j_end)
		{
			std::swap(i_start, i_end);
			std::swap(j_start, j_end);
		}

		m_pixel_pts.clear();

		for (int j = j_start - 1; j < j_end; j++)
		{
			for (int i = i_start - 1; i < i_end; i++)
			{
				vector<Point2f> ij(1);
				ij[0].x = i;
				ij[0].y = j;

				Mat ij_nml;
				cv::undistortPoints(Mat(ij), ij_nml, KK, dist_coeff);

				V = Rmat_inv * (Mat_<float>(3, 1) << ij_nml.reshape(1).at<float>(0), ij_nml.reshape(1).at<float>(1), 1);

				a = V.dot(V);
				b = 2 * V.dot(C_S);
				double b2_4ac = b*b - 4 * a*c;

				//intersect?
				if (b2_4ac >= 0)
				{
					double lambda = (-b + std::sqrt(b2_4ac)) / 2 / a;
					Mat_<float> X = C + lambda * V;
					m_pixel_pts.push_back(Point3f(X(0), X(1), X(2)));
				}
				else
				{
					m_pixel_pts.push_back(Point3f(0.f, 0.f, 0.f));
				}
			}
		}
		if (m_deviceID) // projectors only
		{
			createPixelMask();
			findContourFromMask();
		}
	}

	void Pixel3DArray::rayPlaneIntersection(CalibrationBase* pcalib, const cv::Mat_<double>& plane_pose, const cv::Point2i& start_pixel, const cv::Point2i& end_pixel)
	{
		/// sphere center S
		if (plane_pose.empty())
			throw std::runtime_error("Pixel3DArray::rayPlaneIntersection(): sphere_pose is null. ");	
		/// plane param P 4x1
		Mat_<float> P = (Mat_<float>(4, 1) << plane_pose(0), plane_pose(1), plane_pose(2), plane_pose(3));
		
		/// device intrinsic matrix 3x3
		Mat_<float> KK = pcalib->getCameraMatrix().clone();
		/// device lens distortion coefficient 5x1
		Mat_<float> dist_coeff = pcalib->getDistortCoeff().clone();
		/// device inversed Rotation Matrix 3x3
		Mat_<float> Rmat_inv;
		/// device projection center C 3x1
		Mat C(3, 1, CV_32FC1);
		/// device ray direction V 3x1
		Mat V(3, 1, CV_32FC1);

		if (m_deviceID == 0) /// camera
		{
			Rmat_inv = cv::Mat::eye(3, 3, CV_32FC1);
			C = cv::Mat::zeros(3, 1, CV_32FC1);
		}
		else /// projectors
		{
			if (ProjectorCalibration* pProjCalib = dynamic_cast<ProjectorCalibration*>(pcalib))
			{
				dist_coeff = pProjCalib->getDistortCoeff().clone();
				Rmat_inv = pProjCalib->getExtrinsicRmat().clone().inv();
				C = -Rmat_inv * pProjCalib->getExtrinsicTvec().clone();
			}
			else
				throw std::runtime_error("Pixel3DArray::raySphereIntersection() fails: can not cast ProjectorCalibration type. ");
		}

		float dot_PC =
			P.at<float>(0) * C.at<float>(0) +
			P.at<float>(1) * C.at<float>(1) +
			P.at<float>(2) * C.at<float>(2) +
			P.at<float>(3);

		int i_start(start_pixel.x), i_end(end_pixel.x);
		int j_start(start_pixel.y), j_end(end_pixel.y);

		if (i_start >= i_end && j_start >= j_end)
		{
			std::swap(i_start, i_end);
			std::swap(j_start, j_end);
		}
		
		m_pixel_pts.clear();

		for (int j = j_start - 1; j < j_end; j++)
		{
			for (int i = i_start - 1; i < i_end; i++)
			{
				vector<Point2f> ij(1);
				ij[0].x = i;
				ij[0].y = j;

				Mat ij_nml;
				cv::undistortPoints(Mat(ij), ij_nml, KK, dist_coeff);

				V = Rmat_inv * (Mat_<float>(3, 1) << ij_nml.reshape(1).at<float>(0), ij_nml.reshape(1).at<float>(1), 1); 

				float dot_PV = P.at<float>(0) * V.at<float>(0) + P.at<float>(1) * V.at<float>(1) + P.at<float>(2) * V.at<float>(2);
				float lambda = -dot_PC / dot_PV;

				Mat_<float> X = C + lambda * V;
				m_pixel_pts.push_back(Point3f(X(0), X(1), X(2)));
			}
		}
		if (m_deviceID) // projectors
		{
			createPixelMask();
			findContourFromMask();
		}
	}

	void Pixel3DArray::computeAlphaMask( std::vector<ProjectorCalibration>& projCalibArray, std::vector<std::vector<Pixel3DArray>::iterator>& pixelArrayIter)
	{

		uint nProj = projCalibArray.size();
		if (nProj == 0)
			throw std::runtime_error("Pixel3DArray::computeAlphaMask() fails: projector calibration array is null. ");
		if (pixelArrayIter.size() != nProj)
			throw std::runtime_error("Pixel3DArray::computeAlphaMask() fails: projCalibArray and pixelArrayList have inequal elements. ");
		if (m_deviceID == 0)
			throw std::runtime_error("Pixel3DArray::computeAlphaMask() fails: cannot compute alpha mask for camera. ");
		if (m_pixel_pts.empty() )
			throw std::runtime_error("Pixel3DArray::computeAlphaMask() fails: need m_pixel_pts to compute alpha mask. ");
		if (m_mask.empty() || m_contours2d.empty())
			throw std::runtime_error("Pixel3DArray::computeAlphaMask() fails: need and m_contours2d m_mask to compute alpha mask. ");
		
		uint pi = m_deviceID - 1; // current projector

		cout << endl;
		cout << "===== computing alpha mask of projector " << to_string(m_deviceID) << "... =====" << endl;

		vector<Mat> distance_mat(nProj); // distance mat: N projector, each projector has N distance mat (including its own)
		distance_mat[pi] = Mat::zeros(Size(setting::proj::res_width, setting::proj::res_height), CV_32FC1); // intialize distance mat of projector i on its own contour
		float *ptr_dmat_pi = (float*)distance_mat[pi].data; //for quick access

		m_alpha_mask = Mat::ones(Size(setting::proj::res_width, setting::proj::res_height), CV_32FC1);

		/* find inliners and compute shortest distance to each contour */
		for (uint pj = 0; pj < nProj; pj++) // for all other projectors
		{
			cout << "computing projector " << pi << " to projector " << pj << endl;
			if (pi != pj) // compute alpha mask of projector i
			{
				
				Mat_<float> KKj = projCalibArray[pj].getCameraMatrix().clone(); // load intrinsics of projector j

				Mat_<float> dist_coeffj = projCalibArray[pj].getDistortCoeff().clone();  // load intrinsics of projector j

				Mat_<float> Rvecj = projCalibArray[pj].getExtrinsicRvec().clone(); // load extrinsics of projector j

				Mat_<float> Tvecj = projCalibArray[pj].getExtrinsicTvec().clone(); // load extrinsics of projector j

				//cout << KKj << endl << endl;
				//cout << dist_coeffj << endl << endl;
				//cout << Rvecj << endl << endl;
				//cout << Tvecj << endl << endl;

				
				distance_mat[pj] = Mat::zeros(Size(setting::proj::res_width, setting::proj::res_height), CV_32FC1); // intialize distance mat of projector i on projector j's contour

				float *ptr_dmat_pj = (float*)distance_mat[pj].data; //for quick access 
				unsigned char* ptr_mask_pj = (*pixelArrayIter[pj]).m_mask.data; //for quick access 
															
				vector<Point2f> img_pti;
				projectPoints(m_pixel_pts, Rvecj, Tvecj, KKj, dist_coeffj, img_pti); // project pixels from projector i to projector j (i!=j)

				for (uint k = 0; k < img_pti.size(); k++)
				{
					Point2f proj_pt = img_pti[k];
					// is inliner
					if (proj_pt.x >= 0 && proj_pt.x < setting::proj::res_width && proj_pt.y >= 0 && proj_pt.y < setting::proj::res_height)
					{
						// within pj's mask
						if (ptr_mask_pj[cvFloor(proj_pt.y) * (*pixelArrayIter[pj]).m_mask.step1() + cvFloor(proj_pt.x)]) 
						{
							// 2d coordinates in pi
							uint idx_i = k / setting::proj::res_width;
							uint idx_j = k % setting::proj::res_width;

							uint idx_dmat_k = idx_i*distance_mat[pj].step1() + idx_j;

							double dj = computeShortestDistancetoContour((*pixelArrayIter[pj]).m_contours2d, proj_pt);
							ptr_dmat_pj[idx_dmat_k] = std::pow(dj, 3);

							if (ptr_dmat_pi[idx_dmat_k] == 0) // if not computed yet
							{
								double di = computeShortestDistancetoContour(m_contours2d, Point2f(idx_j, idx_i));
								ptr_dmat_pi[idx_dmat_k] = std::pow(di, 3);
							}
						}
					}
				} // pixel loop
			}
		} //pj loop
		computeAlphaFromDistance(distance_mat, pi, m_alpha_mask);
	}

	void Pixel3DArray::normalizePixelPointToSphereCenter( const cv::Mat_<double>& sphere_pose, vector<Point3f>& pixel_nml)
	{
		if (sphere_pose.empty())
			throw std::runtime_error("Pixel3DArray::normalizePixelPointToSphereCenter() fail: sphere_pose is null. ");

		Point3f translation = Point3f(sphere_pose(0), sphere_pose(1), sphere_pose(2));
		float scaling = 1.f / sphere_pose(3);
		
		pixel_nml.clear();

		pixel_nml.resize((m_pixel_pts.size()));
		transform(m_pixel_pts.begin(), m_pixel_pts.end(), pixel_nml.begin(), bind2nd(std::plus<Point3f>(), -translation));
		transform(pixel_nml.begin(), pixel_nml.end(), pixel_nml.begin(), bind2nd(utils::mult_scalar<Point3f>(), scaling));
		
	}

	void Pixel3DArray::normalizePixelPointToPlaneCenter(const cv::Mat_<double>& plane_pose, std::vector<cv::Point3f>& pixel_nml)
	{
		if (plane_pose.empty())
			throw std::runtime_error("Pixel3DArray::normalizePixelPointToSphereCenter() fail: plane_pose is null. ");

		const float CAMTOSCREEN_METER = 0.35f; // hardcoding the distance in meters from camera to the planar screen to correct the scaling

		Point3f translation = Point3f(0.f, 0.f, -plane_pose(3) / plane_pose(2));
		float scaling = CAMTOSCREEN_METER / (-plane_pose(3) / plane_pose(2));

		pixel_nml.clear();

		pixel_nml.resize((m_pixel_pts.size()));
		transform(m_pixel_pts.begin(), m_pixel_pts.end(), pixel_nml.begin(), bind2nd(std::plus<Point3f>(), -translation));
		transform(pixel_nml.begin(), pixel_nml.end(), pixel_nml.begin(), bind2nd(utils::mult_scalar<Point3f>(), scaling));	
	}

	void Pixel3DArray::gammaCorrection(const float gamma)
	{
		if (m_alpha_mask.rows != setting::proj::res_height || m_alpha_mask.cols!= setting::proj::res_width)
			throw std::runtime_error("Pixel3DArray::gammaCorrection() fails: alpha mask has wrong size. ");

		float* p_alpha;
		for (int idx_x = 0; idx_x < setting::proj::res_height; idx_x++)
		{
			for (int idx_y = 0; idx_y < setting::proj::res_width; idx_y++)
			{
				p_alpha = &m_alpha_mask.at<float>(idx_x, idx_y);
				float alpha = *p_alpha;
				*p_alpha = std::powf(alpha, 1 / gamma);
			}
		}
	}


	void Pixel3DArray::computeAlphaFromDistance(const std::vector<cv::Mat>& distance_mat, const int& proj_idx, cv::Mat& out_alpha_mask)
	{
		Size img_size = distance_mat[proj_idx].size();
		Mat sum_d_mat = Mat::zeros(img_size, CV_32FC1);
		for (const auto& d : distance_mat)
		{
			sum_d_mat += d;
		}
		cv::divide(sum_d_mat - distance_mat[proj_idx], sum_d_mat, out_alpha_mask);
		out_alpha_mask = Mat::ones(img_size, CV_32FC1) - out_alpha_mask;
	}

	void Pixel3DArray::createPixelMask()
	{
		if (m_pixel_pts.size() != setting::proj::res_height * setting::proj::res_width)
			throw std::runtime_error("Pixel3DArray::createPixelMask() fails: m_pixel_pts does not have the correct size. ");
		
		m_mask = Mat::zeros(Size(setting::proj::res_width, setting::proj::res_height), CV_8UC1);
		for (uint i = 0; i < m_pixel_pts.size(); i++)
		{
			int mask_j = i % setting::proj::res_width;
			int mask_i = i / setting::proj::res_width;

			Point3f pixel3_pt = m_pixel_pts.at(i);
			
			if (!utils::isInRange(mask_i, 0, setting::proj::res_height) || !utils::isInRange(mask_j, 0, setting::proj::res_width))
				throw std::runtime_error("Pixel3DArray::createPixelMask() fails: index not in range. ");

			if (cv::norm(pixel3_pt) > 0)
				m_mask.at<uchar>(mask_i, mask_j) = 255;
		}
	}

	void Pixel3DArray::findContourFromMask()
	{
		vector<vector<cv::Point>> contours;
		vector<cv::Vec4i> hierarchy;
		
		Mat img_pad;
		img_pad = m_mask.clone();

		findContours(img_pad, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		if (!contours.empty())
			m_contours2d = contours[0];
	}

	double Pixel3DArray::computeShortestDistancetoContour(const std::vector<cv::Point3f>& contour3d, const cv::Point3f& obj_pt)
	{
		size_t size_contr = contour3d.size();
		vector<double> arc_distance(size_contr);
		for (uint i = 0; i < size_contr; i++)
		{
			arc_distance[i] = cv::norm(contour3d[i] - obj_pt);
		}

		double min_val = *std::min_element(arc_distance.begin(), arc_distance.end());
		return min_val;
	}

	double Pixel3DArray::computeShortestDistancetoContour(const std::vector<cv::Point2i>& contour2d, const cv::Point2f& img_pt)
	{
		return abs(pointPolygonTest(contour2d, img_pt, true));
	}

	void Pixel3DArray::savePixel3DArray(const std::string& file, const std::string& dataType, std::vector<cv::Point3f> dataToSave)
	{
		float* p_float;
		uint size;
	
		if (dataType.compare("geom") == 0)
		{
			Point3f* p_point3f;
		
			if (dataToSave.empty())
				p_point3f = m_pixel_pts.data();
			else
				p_point3f = dataToSave.data();

			p_float = &(p_point3f->x);
			
			cout << endl;
			if (m_deviceID == 0)
			{
				size = setting::camera::res_width * setting::camera::res_height * 3; /// camera
				cout << "save geometry data of camera " << endl;
			}
			else
			{
				size = setting::proj::res_width * setting::proj::res_height * 3; /// projectors
				cout << "save geometry data of projector " << to_string(m_deviceID) << endl;
			}


		}
		else if (dataType.compare("alpha") == 0)
		{
			p_float = (float *)m_alpha_mask.data;
			size = setting::proj::res_width * setting::proj::res_height;
			cout << endl;
			cout << "save alpha mask of projector " << to_string(m_deviceID) << endl;
		}
		else
		{
			throw std::runtime_error("Pixel3DArray::savePixel3DArray() fails: dataType should be geom or alpha. ");
		}
		saveFloatArrayToBinFile(file, p_float, size);
	}

	void Pixel3DArray::loadPixel3DArray(const std::string& file, const std::string& dataType)
	{
		FILE *pfile; 
		fopen_s(&pfile, file.data(), "rb");
		if (pfile == NULL)
		{
			throw std::runtime_error("Pixel3DArray::loadPixel3DArray() fails: cannot find file " + file);
		}
		fseek(pfile, 0, SEEK_END);
		long lsize = ftell(pfile);
		rewind(pfile);

		if (dataType.compare("geom") == 0)
		{
			cout << endl;
			cout << "load pixel geometry of " << file << endl;

			m_pixel_pts.clear();
			while (ftell(pfile) < lsize)
			{
				float pt[3];
				fread(&pt, sizeof(float), 3, pfile);
				m_pixel_pts.push_back(Point3f(pt[0], pt[1], pt[2]));
			}
		}
		else if (dataType.compare("alpha") == 0)
		{
			cout << endl;
			cout << "load alpha mask of " << file << endl;
			
			m_alpha_mask = Mat::zeros(Size(setting::proj::res_width, setting::proj::res_height), CV_32FC1);
			int i = 0;
			while (ftell(pfile) < lsize)
			{
				float pt;
				fread(&pt, sizeof(float), 1, pfile);
				m_alpha_mask.at<float>(i) = pt;
				i++;
			}
		}
		else 
		{
			throw std::runtime_error("Pixel3DArray::loadPixel3DArray() fails: dataType should be geom or alpha. ");
		}
		cout << "load completed" << endl;
		fclose(pfile);
	}

	void Pixel3DArray::saveFloatArrayToBinFile(std::string file, float* pData, uint size)
	{
		FILE* pfile;
		fopen_s(&pfile, file.data(), "wb");
		if (pfile == NULL)
			throw std::runtime_error("Pixel3DArray::saveFloatArrayToBinFile() fails: cannot save file " + file);

		fwrite(pData, sizeof(float), size, pfile);
		cout << "save completed." << endl;
		fclose(pfile);
	}

}