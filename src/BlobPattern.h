/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_BLOB_PATTERN_H
#define CALIB_BLOB_PATTERN_H

#include <iostream>
#include <vector>
#include <string> 
#include <stdexcept>

#include "BlobDetector.hpp"
#include "BackGroundSubstractor.h"
#include "BoundingCircle.h"

#include "global.h"

namespace multi_proj_calib
{

	/* BlobPattern: class to generate and detect blob features */

	class BlobPattern
	{
	public:
		BlobPattern() :m_blob_detector(), m_blob_total_count(60), m_curr_cir(cv::Size(1,1)), m_grid_size(cv::Size(setting::blob_row, setting::blob_col)) {}
		
		// return position of a random blob (not in use)
		cv::Point2f generateRandomBlob(const cv::Size& proj_img_size = cv::Size(setting::proj_width, setting::proj_height), const float blob_radius_pixel = setting::blob_radius);
		
		// return position of a single blob within a grid of blobs
		cv::Point2f generateBlobGrid(const cv::Size& proj_img_size = cv::Size(setting::proj_width, setting::proj_height));

		//setup default blob detector params:  tweak these params based on the light condition and the actual display size
		void setupBlobDetector();
		
		//reset blob data; call for each projector
		void resetBlobs();
		
		//add blob to the container; call if a blob detected
		void addBlob();
		
		//erase blobs outside bounding circle
		bool cleanBlobs();

		//detect and record the first blob detected
		bool detectSingleBlob( cv::Mat& img, float intensityScalar = 5);

		//call after single blob detection
		void drawDetectedBlob(cv::Mat& img);

		//automatic detect bounding circle, not in use as lighting condition varies in cases
		bool detectBoundingCircle(const cv::Mat& img)
		{
			return m_bound_circle.detectBoundary(img);
		}

		//detect bounding circle by collecting the boudary points (user input)
		bool detectBoundingCircle()
		{
			return m_bound_circle.detectBoundary();
		}

		//draw the counding circle; call after bounding detection
		void drawBoundingCircle(cv::Mat& img)
		{
			return m_bound_circle.drawBoundaryCircle(img);
		}

		//record a background imgage; call after bounding detection succeed
		void setBackGroundImg(const cv::Mat& img)
		{
			m_background_img.updateBackGroundImg(img);
		}

		//add a bounding point to the container
		void addBoundPoint(const int& x, const int& y)
		{
			m_bound_circle.addBoundaryPoint(x, y);
		}

		//convert an intensity image to binary image based on threshold (not in use now)
		void thresholdImage(const cv::Mat& src_img, cv::Mat& dest_img, int thre);

		void saveBlobData(std::string file_name);

		bool isBoundCircleFound() { return m_bound_circle.isBoundCircleFound(); }

		const uint getCurrBlobCnt() const { return m_cam_blobs.size(); }

		const uint& getTotalBlobCnt() const { return m_blob_total_count; }

		const cv::Size& getCurrBlobIdx() const { return m_curr_cir; }

		const cv::Size& getGridBlobSize() const { return m_grid_size; }

		std::vector<cv::Point2f>& getCamBlobs()  { return m_cam_blobs; }

		std::vector<cv::Point2f>& getProjBlobs()  { return m_proj_blobs; }

		void setTotalBlobCnt(int blob_num)
		{
			m_blob_total_count = blob_num;
		}

	private:
				
		cv::Point2f m_projected_blob;
		cv::Point2f m_detected_blob;
		std::vector<cv::Point2f> m_cam_blobs;
		std::vector<cv::Point2f> m_proj_blobs;
		
		cv::RNG m_rng;
		uint m_blob_total_count;
		cv::Size m_curr_cir; //index of current projected blob, at (i,j) within M x N blob grid
		cv::Size m_grid_size; //grid size of blob patterns, contains M x N blobs

		BlobDetector m_blob_detector;

		BoundingCircle m_bound_circle;
		BackGroundSubstractor m_background_img;
	};
	
}

#endif // !CALIB_BLOB_PATTERN_H