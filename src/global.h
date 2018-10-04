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
		namespace camera
		{
			extern int res_width; //resolution
			extern int res_height;
			extern uint cv_pixel_format;
			extern int checkerboard_col; //pattern size
			extern int checkerboard_row;
			extern double max_reproj_error; 
		}

		namespace proj
		{
			extern int res_width; //resolution
			extern int res_height;
			extern int blobs_col; //pattern size
			extern int blobs_row;
			extern double max_reproj_error;
		}

		namespace display
		{
			extern int num_proj;
			extern int blobs_col;
			extern int blobs_row;
			extern float blob_radius;
			extern double max_reproj_error;
			extern double max_spherepose_error;
		}
	}

	namespace file
	{
		extern std::string camcalib_file;
		extern std::string projcalib_file[4];
		extern std::string blob_file[4];
		extern std::string extrinsic_file;
		extern std::string gridtexture_file;
		extern std::string optimparam_file;
		extern std::string geom_file[4];
		extern std::string blen_file[4];
		extern std::string camgeom_file;
		extern std::string data_path;
		extern std::string src_path;
	}

	namespace utils
	{
		// todo: load json file
		
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

		template <typename T> bool isInRange(const T& value, const T& low, const T& high) {
			return (value >= low) && (value < high);
		};
	}
}

#endif //! GLOBAL_H