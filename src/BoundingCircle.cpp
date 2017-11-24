#include "BoundingCircle.h"

namespace multi_proj_calib
{
	using namespace cv;
	using std::cout;
	using std::endl;
	using std::to_string;

	bool BoundingCircle::detectBoundary()
	{
		/// if current boundary points are enough
		if (m_bound_pt.size() < m_total_pt)
		{
			m_findcircle = false;
		}
		else
		{
			m_findcircle = true;
			linearLSCircle();
			cout << "Bounding Circle Found:" << m_bound_center << endl;
		}
		return m_findcircle;
	}

	void BoundingCircle::addBoundaryPoint(const int& x, const int& y)
	{
		m_bound_pt.push_back(Point2f(x, y));
		cout << "Add boundary point " << m_bound_pt.size() << "/" << m_total_pt << endl;
	}

	void BoundingCircle::drawBoundaryCircle(Mat& img)
	{
		circle(img, (cv::Point2i)m_bound_center, m_bound_radius, Scalar(255, 255, 255), 5);
	}

	void BoundingCircle::linearLSCircle()
		/*
		Modified from Nikolai Chernov Circle Fitting Code (September 2012):
		
		Circle fit to a given set of data points (in 2D)

		This is an algebraic fit, disovered and rediscovered by many people.
		One of the earliest publications is due to Kasa:

		I. Kasa, "A curve fitting procedure and its error analysis",
		IEEE Trans. Inst. Meas., Vol. 25, pages 8-14, (1976)

		The method is based on the minimization of the function

		F = sum [(x-a)^2 + (y-b)^2 - R^2]^2

		This is perhaps the simplest and fastest circle fit.

		It works well when data points are sampled along an entire circle
		or a large part of it (at least half circle).

		It does not work well when data points are sampled along a small arc
		of a circle. In that case the method is heavily biased, it returns
		circles that are too often too small.
		*/
	{

		size_t size = m_bound_pt.size();

		float Xi(0.f), Yi(0.f), Zi(0.f);
		float Mxy(0.f), Mxx(0.f), Myy(0.f), Mxz(0.f), Myz(0.f);
		float B(0.f), C(0.f), G11(0.f), G12(0.f), G22(0.f), D1(0.f), D2(0.f);

		float meanX = cv::mean(m_bound_pt)[0];
		float meanY = cv::mean(m_bound_pt)[1];

		// computing moments 
		for (uint i = 0; i< size; i++)
		{
			Xi = m_bound_pt[i].x - meanX;   //  centered x-coordinates
			Yi = m_bound_pt[i].y - meanY;   //  centered y-coordinates
			Zi = Xi*Xi + Yi*Yi;

			Mxx += Xi*Xi;
			Myy += Yi*Yi;
			Mxy += Xi*Yi;
			Mxz += Xi*Zi;
			Myz += Yi*Zi;
		}
		Mxx /= size;
		Myy /= size;
		Mxy /= size;
		Mxz /= size;
		Myz /= size;

		// solving system of equations by Cholesky factorization
		G11 = std::sqrtf(Mxx);
		G12 = Mxy / G11;
		G22 = std::sqrtf(Myy - G12*G12);

		D1 = Mxz / G11;
		D2 = (Myz - D1*G12) / G22;

		// computing paramters of the fitting circle
		C = D2 / G22 / 2.f;
		B = (D1 - G12*C) / G11 / 2.f;

		//  assembling the output
		m_bound_center.x = B + meanX;
		m_bound_center.y = C + meanY;
		m_bound_radius = std::sqrtf(B*B + C*C + Mxx + Myy);
	}

	bool BoundingCircle::detectBoundary(const Mat& img)
	{
		static const int total_cnt = 10;

		/// blur the image first
		Mat blur_img;
		medianBlur(img, blur_img, 17);
		if (blur_img.type() != CV_8UC1 && blur_img.type() != CV_16UC1)
			cvtColor(blur_img, blur_img, CV_BGR2GRAY);

		/// Apply Canny to detect edge with automatic threshold
		Mat edges;
		Mat _no_use_array;
		double otsu_thresh_val = cv::threshold(
			blur_img, _no_use_array, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
		);
		double high_thresh_val = otsu_thresh_val,
			lower_thresh_val = otsu_thresh_val * 0.5;
		Canny(blur_img, edges, lower_thresh_val, high_thresh_val);

		/// Apply the dilation and erosion operation: make edges stronger and merge the gap
		Mat element_dil = getStructuringElement(MORPH_ELLIPSE,
			Size(5, 5));
		Mat element_ero = getStructuringElement(MORPH_ELLIPSE,
			Size(3, 3));
		dilate(edges, edges, element_dil);
		erode(edges, edges, element_ero);

		/// Now detect the bounding circle: use minradius and maxradius to filter false detections
		vector<Vec3f> circles;
		int num_pts = cv::sum(edges)[0];
		/// modify the coefficients if no bounding circle is detected
		double param2 = 2e-5 * num_pts + 10;

		const int minradius = 400;
		const int maxradius = 600;
		HoughCircles(edges, circles, CV_HOUGH_GRADIENT, 2, blur_img.rows / 8, high_thresh_val, param2, minradius, maxradius);

		//dbg
		//static int cnt = 0;
		//imwrite("edges"+to_string(cnt) +".jpg", edges);
		//cnt++;

		// in case radius is not accurate: detect several times to get an average of the radius
		if (m_bound_cnt < total_cnt)
		{
			if (!circles.empty())
			{
				m_bound_center += Point2f(circles[0][0], circles[0][1]);
				m_bound_radius += circles[0][2];
				m_bound_cnt++;
			}
			else
			{
				cout << endl;
				cout << "no bounding circle found" << endl;
			}
			cout << "m_bound_cnt" << m_bound_cnt << endl;
			m_findcircle = false;
		}
		else
		{
			m_bound_center.x = m_bound_center.x / m_bound_cnt;
			m_bound_center.y = m_bound_center.y / m_bound_cnt;
			m_bound_radius = m_bound_radius / m_bound_cnt;
			m_findcircle = true;
		}
		return m_findcircle;
	}


}