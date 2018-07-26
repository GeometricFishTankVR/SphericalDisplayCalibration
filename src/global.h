/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <opencv2/opencv.hpp>
#include <string>

namespace multi_proj_calib
{
	typedef unsigned int uint;
	
	namespace setting
	{
		extern const int proj_width;
		extern const int proj_height;
		extern const int cam_width;
		extern const int cam_height;
		extern const uint cv_pixel_format;
		extern const float blob_radius;
		extern const int blob_col;
		extern const int blob_row;
	}

	namespace file
	{
		extern const std::string camcalib_file;
		extern const std::string projcalib_file[4];
		extern const std::string blob_file[4];
		extern const std::string extrinsic_file;
		extern const std::string gridtexture_file;
		extern const std::string optimparam_file;
		extern const std::string geom_file[4];
		extern const std::string camgeom_file;
		extern const std::string blen_file[4];
		extern const std::string data_path;
		extern const std::string src_path;
	}

	namespace utils
	{
		template <typename T> inline int sgn(T val)
		{
			return (T(0) < val) - (val < T(0));
		};

		template <class T> struct mult_scalar : public std::binary_function <T, float, T>
		{
			T operator() (const T& p, const float& m) const {
				return p * m;
			}
		};

		template <typename T>
		bool isInRange(const T& value, const T& low, const T& high) {
			return (value >= low) && (value < high);
		};
	}
}

#endif //! GLOBAL_H