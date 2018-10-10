#include "FundamentalMat.h"

namespace stereo_recon
{
	using std::vector;
	using std::cout;
	using std::endl;

	using cv::Mat;
	using cv::Mat_;
	using cv::Matx;
	using cv::Vec;
	using cv::Point3f;
	using cv::Point2f;
	using cv::Point2d;

	cv::Mat_<float> FundamentalMat::findFundamentalMat(const vector<Point2f>& pts1, const vector<Point2f>& pts2, DistanceType type)
	{
		CV_Assert(pts1.size() == pts2.size());
		m_size = pts1.size();


		const float confidence = .99;
		const float outline_frac = .2;
		const int num_corresp = m_size * (1 - outline_frac);

		vector<Point3f> pts1_h;
		vector<Point3f> pts2_h;

		cv::convertPointsToHomogeneous(pts1, pts1_h);
		cv::convertPointsToHomogeneous(pts2, pts2_h);

		Mat_<float> best_F = Mat::zeros(3, 3, CV_32FC1);
		Mat_<float> curr_F = Mat::zeros(3, 3, CV_32FC1);

		float best_d(FLT_MAX);
		float curr_d(FLT_MAX);

		vector<float> dvec(m_size);

		vector<Point2f> p1_inliner;
		vector<Point2f> p2_inliner;


		for (unsigned int i = 0; i < m_num_trials; i++)
		{

			vector<int> pts8_index(8);
			generateRandom8Pts(pts8_index);

			vector<Point2f> rdm_8pts1(8);
			vector<Point2f> rdm_8pts2(8);
			for (int i = 0; i < 8; i++)
			{
				rdm_8pts1[i].x = pts1_h[pts8_index[i]].x;
				rdm_8pts1[i].y = pts1_h[pts8_index[i]].y;

				rdm_8pts2[i].x = pts2_h[pts8_index[i]].x;
				rdm_8pts2[i].y = pts2_h[pts8_index[i]].y;

			}
			run8Point(Mat(rdm_8pts1), Mat(rdm_8pts2), curr_F);
			computeDistance(pts1_h, pts2_h, curr_F, dvec, type);

			curr_d = medianf(dvec);

			if (best_d > curr_d)
			{
				best_d = curr_d;
				best_F = curr_F;
			}
		}
		computeDistance(pts1_h, pts2_h, best_F, dvec, type);

		p1_inliner.clear();
		p2_inliner.clear();
		
		for (int i = 0; i < m_size; i++)
		{
			if( dvec[i] <= best_d)
			{
				p1_inliner.push_back(pts1[i]);
				p2_inliner.push_back(pts2[i]);
			}
		}

		run8Point(Mat(p1_inliner), Mat(p2_inliner), m_F);
	
		return m_F;
	}
	cv::Mat_<float> FundamentalMat::findEssentialMat(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, DistanceType type)
	
	{
		const int max_cnt = 100;
		const float step = 0.005f;

		Mat_<float> S, U, Vt;
		float s_ratio(0.f);
		float s_thre(m_thre_ratio);
		int cnt(0);
dowhile:
		do 
		{
			findFundamentalMat(pts1, pts2, type);
			cv::SVD::compute(m_F, S, U, Vt);
			s_ratio = S(1) / S(0);
			cout << "S ratio=" << s_ratio << endl;
			cnt++;
		} while ( s_ratio < s_thre && cnt < max_cnt);
		
		if (cnt >= max_cnt)
		{
			s_thre = s_thre - step;
			cnt = 0;
			cout << endl;
			cout << "Adjust sRatio_threhold to " << s_thre << endl;
			goto dowhile;
		}
		return m_F;
	}
	
	int FundamentalMat::run8Point(const Mat& _m1, const Mat& _m2, Mat& _fmatrix)
	{
		Point2d m1c(0, 0), m2c(0, 0);
		double t, scale1 = 0, scale2 = 0;

		const Point2f* m1 = _m1.ptr<Point2f>();
		const Point2f* m2 = _m2.ptr<Point2f>();
		CV_Assert((_m1.cols == 1 || _m1.rows == 1) && _m1.size() == _m2.size());
		int i, count = _m1.checkVector(2);

		// compute centers and average distances for each of the two point sets
		for (i = 0; i < count; i++)
		{
			m1c += Point2d(m1[i]);
			m2c += Point2d(m2[i]);
		}

		// calculate the normalizing transformations for each of the point sets:
		// after the transformation each set will have the mass center at the coordinate origin
		// and the average distance from the origin will be ~sqrt(2).
		t = 1. / count;
		m1c *= t;
		m2c *= t;

		for (i = 0; i < count; i++)
		{
			scale1 += cv::norm(Point2d(m1[i].x - m1c.x, m1[i].y - m1c.y));
			scale2 += cv::norm(Point2d(m2[i].x - m2c.x, m2[i].y - m2c.y));
		}

		scale1 *= t;
		scale2 *= t;

		if (scale1 < FLT_EPSILON || scale2 < FLT_EPSILON)
			return 0;

		scale1 = std::sqrt(2.) / scale1;
		scale2 = std::sqrt(2.) / scale2;

		Matx<double, 9, 9> A;

		// form a linear system Ax=0: for each selected pair of points m1 & m2,
		// the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
		// to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
		for (i = 0; i < count; i++)
		{
			double x1 = (m1[i].x - m1c.x)*scale1;
			double y1 = (m1[i].y - m1c.y)*scale1;
			double x2 = (m2[i].x - m2c.x)*scale2;
			double y2 = (m2[i].y - m2c.y)*scale2;
			Vec<double, 9> r(x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1);
			A += r*r.t();
		}

		Vec<double, 9> W;
		Matx<double, 9, 9> V;

		eigen(A, W, V);

		for (i = 0; i < 9; i++)
		{
			if (fabs(W[i]) < DBL_EPSILON)
				break;
		}

		if (i < 8)
			return 0;
		
		using cv::Matx33d;

		Matx33d F0(V.val + 9 * 8); // take the last column of v as a solution of Af = 0

								   // make F0 singular (of rank 2) by decomposing it with SVD,
								   // zeroing the last diagonal element of W and then composing the matrices back.
		cv::Vec3d w;
		Matx33d U;
		Matx33d Vt;

		cv::SVD::compute(F0, w, U, Vt);
		w[2] = 0.;

		F0 = U*Matx33d::diag(w)*Vt;

		// apply the transformation that is inverse
		// to what we used to normalize the point coordinates
		Matx33d T1(scale1, 0, -scale1*m1c.x, 0, scale1, -scale1*m1c.y, 0, 0, 1);
		Matx33d T2(scale2, 0, -scale2*m2c.x, 0, scale2, -scale2*m2c.y, 0, 0, 1);

		F0 = T2.t()*F0*T1;
		
		// make F(3,3) = 1
		if (fabs(F0(2, 2)) > FLT_EPSILON)
			F0 *= 1. / F0(2, 2);

		Mat(F0).copyTo(_fmatrix);
		_fmatrix.convertTo(_fmatrix, CV_32FC1);
		return 1;
	}

	void FundamentalMat::generateRandom8Pts(vector<int>& idx_rdm8pts)
	{
		idx_rdm8pts.clear();
		idx_rdm8pts.resize(8, 0);

		size_t num_uniq(0);

		//srand(clock());
		srand(clock()+time(NULL));
		while (num_uniq < 8)
		{			
			int p = rand() % m_size;
			if (find(idx_rdm8pts.begin(), idx_rdm8pts.end(), p) == idx_rdm8pts.end())
			{
				idx_rdm8pts[num_uniq] = p;
				num_uniq++;
			}
		}
	}


	//cv::Mat_<float> FundamentalMat::norm8Points(const std::vector<cv::Point3f>& pts1_h, const std::vector<cv::Point3f>& pts2_h)
	//{
	//	size_t size = pts1_h.size();

	//	Mat_<float> T1, T2;
	//	vector<Point3f> pts1_h_nml(size), pts2_h_nml(size);
	//	normalizePoints(pts1_h, pts1_h_nml, T1);
	//	normalizePoints(pts2_h, pts2_h_nml, T2);

	//	Mat_<float> A(size, 9);

	//	for (int i = 0; i < size; i++)
	//	{
	//		A.row(i) << pts2_h[i].x * pts1_h[i].x, pts2_h[i].x * pts1_h[i].y, pts2_h[i].x,
	//					pts2_h[i].y * pts1_h[i].x, pts2_h[i].y * pts1_h[i].y, pts2_h[i].y,
	//								  pts1_h[i].x,               pts1_h[i].y,			1;
	//	}
	//	Mat U, S, Vt;
	//	SVD::compute(A, S, U, Vt);
	//	Mat V = Vt.t();

	//}

	//void FundamentalMat::normalizePoints(const std::vector<cv::Point3f>& src_h, std::vector<cv::Point3f>& dest_h, Mat_<float>& T)
	//{
	//	size_t size = src_h.size();

	//}


	void FundamentalMat::computeDistance(const std::vector<cv::Point3f>& pts1_h, const std::vector<cv::Point3f>& pts2_h, const cv::Mat_<float>& F, std::vector<float>& dvec, DistanceType type)
	{

		size_t size = pts1_h.size();
		dvec.clear();
		dvec.resize(size);
		switch (type)
		{
		case Sampson:
			for (int i = 0; i < size; i++)
			{
				
				float x1 = pts1_h[i].x;
				float y1 = pts1_h[i].y;
				float z1 = pts1_h[i].z;

				float x2 = pts2_h[i].x;
				float y2 = pts2_h[i].y;
				float z2 = pts2_h[i].z;

				float elx1 = F(0, 0) * x1 + F(0, 1) * y1 + F(0, 2) * z1;
				float ely1 = F(1, 0) * x1 + F(1, 1) * y1 + F(1, 2) * z1;
				float elz1 = F(2, 0) * x1 + F(2, 1) * y1 + F(2, 2) * z1;

				float elx2 = F(0, 0) * x2 + F(1, 0) * y2 + F(2, 0) * z2;
				float ely2 = F(0, 1) * x2 + F(1, 1) * y2 + F(2, 1) * z2;
				float elz2 = F(0, 2) * x2 + F(1, 2) * y2 + F(2, 2) * z2;

				float ezero = elx2 * x1 + ely2 * y1 + elz2 * z1;
				dvec[i] = pow(ezero, 2) / ( pow(elx1, 2) + pow(ely1, 2) + pow(elx2, 2) + pow(ely2, 2) );
			}
			break;
		case Geometric:
			break;
		default:
			break;
		}
	}

	inline float FundamentalMat::medianf(vector<float> vec)
	{
		float median;

		sort(vec.begin(), vec.end());

		if (m_size % 2 == 0)
			median = (vec[m_size / 2 - 1] + vec[m_size / 2]) / 2;
		else
			median = vec[(m_size - 1) / 2];
		return median;
	}

}

