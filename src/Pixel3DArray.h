/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_PIXEL_3DARRAY_H
#define CALIB_PIXEL_3DARRAY_H

#include <iostream>
#include <math.h>
#include <vector>
#include <functional>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include "CameraCalibration.h"
#include "ProjectorCalibration.h"
#include "global.h"


namespace multi_proj_calib
{
	/* Pixel3DArray: store and compute the array of 3D position and alpha mask of each pixel for a projector or camera */
	
	class Pixel3DArray
	{
	public:

		///***** interface *****////

		// ray sphere intersection that computes 3D points from start_pixel to end_pixel
		void raySphereIntersection( CalibrationBase* pcalib, const cv::Mat_<double>& sphere_pose, const cv::Point2i& start_pixel, const cv::Point2i& end_pixel);

		// ray plane intersection that computes 3D points from start_pixel to end_pixel
		void rayPlaneIntersection(CalibrationBase* pcalib, const cv::Mat_<double>& plane_pose, const cv::Point2i& start_pixel, const cv::Point2i& end_pixel);

		// compute alpha mask using geometry data
		void computeAlphaMask(std::vector<ProjectorCalibration>& projCalibArray, std::vector<std::vector<Pixel3DArray>::iterator>& pixelArrayIter);
		
		// normalize geometry data in sphere-centered coordinate frame with raius of one
		void normalizePixelPointToSphereCenter(const cv::Mat_<double>& sphere_pose, std::vector<cv::Point3f>& pixel_nml);

		// normalize geometry data in a plane-centered coordinate frame with plane passing the origin
		void normalizePixelPointToPlaneCenter(const cv::Mat_<double>& plane_pose, std::vector<cv::Point3f>& pixel_nml);

		// apply gamma correction to the alpha mask
		void gammaCorrection(const float gamma);

		// save geometry data and alpha mask to file; dataType is "geom" or "alpha"; 
		void savePixel3DArray(const std::string& file, const std::string& dataType, std::vector<cv::Point3f> dataToSave = std::vector<cv::Point3f>());
		
		// load geometry data and alpha mask to file; dataType is "geom" or "alpha"; 
		void loadPixel3DArray(const std::string& file, const std::string& dataType);

		std::vector<cv::Point3f> m_pixel_pts; // geometry data
		cv::Mat_<float> m_alpha_mask; // alpha mask
		
		cv::Mat m_mask; // mask of pixels on the display for this device
		std::vector<cv::Point2i> m_contours2d; // contour of pixels on the display for this device
		uint m_deviceID; // device index (0: cam, 1 : proj1, 2 : proj2, ...)
		
		///***** Member Methods and Data *****////
	
	protected:

		// compute projector mask based on m_pixel_pts
		void createPixelMask();

		// find contour of mask
		void findContourFromMask();

		// compute 3D Euclidean distance between each inner pixel to the contour: slow, not in use
		double computeShortestDistancetoContour(const std::vector<cv::Point3f>& contour3d, const cv::Point3f& obj_pt);

		// (approximation) compute 2D Euclidean distance between each inner pixel to the contour
		double computeShortestDistancetoContour(const std::vector<cv::Point2i>& contour2d, const cv::Point2f& img_pt);

		// compute alpha mask for each projector based on distances
		void computeAlphaFromDistance(const std::vector<cv::Mat>& distance_mat, const int& proj_idx, cv::Mat& out_alpha_mask);

		// save float data with size to bin file 
		void saveFloatArrayToBinFile(std::string file, float* pData, uint size);
	};

}

#endif // !CALIB_PIXEL_3DARRAY_H