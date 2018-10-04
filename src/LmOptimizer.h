/*
* Automatic Calibration Approach For Spherical Multi - projector Display
* Author: Qian Zhou(qzhou{ at }ece.ubc.ca)
* (c)University of British Columbia 2017.
* Usage subject to the accompanying file "License.txt"
*/

#ifndef LM_OPTIM_H
#define LM_OPTIM_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <opencv2/opencv.hpp>

#include "optimization.h"
#include "lm_optim_jacobians.h"

namespace multi_proj_calib
{
	class LmOptimizer
	{
	public:
		static void runNonLinearOptimize(const std::vector<std::vector<cv::Point2f>>& cam_pts, const std::vector<std::vector<cv::Point2f>>& proj_pts,  std::vector<double>& params);
	protected:
		/* 
		* function vector for alglib optimization: 
		* computes pixel error for all projectors by calling cost function function_mincost() for each projector
		* 
		% Input definition:
		% params = [ fx_p1, fy_p1, cx_p1, cy_p1, //1-4 projector1 intrinsic
		%		 om11, om12, om13, //5-7 proj1 rotation
		%       t11, t12, t13, //8-10 proj1 translation
		%       fx_p2, fy_2, cx_p2, cy_p2, //11-14 proj2 intrinsic
		%       om21, om22, om23, //15-17 proj2 rotation
		%       t21, t22, t23, //18-20 proj2 translation
		%       ... //etc proj3...
		%       a, b, c, r, ///?1-?4 sphere pose
		%       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //?5-?3 camera intrinsic and lens distortion
		% 
		% Returns:
		%		fi: pixel error for all points of all projectors. [ P11x P11y P12x P12y ...  P21x P21y P22x P22y ...   ]
		%														  |<- projector 1 points ->|<- projector 2 points ->| ...
		*/
		static void func_err(const alglib::real_1d_array &params, alglib::real_1d_array &fi, void *ptr);
		/*
		* function jacobian for alglib optimization:
		* computes jacobian matrix for all projectors by calling cost function function_mincost() for each projector
		*
		% Input definition:
		% params = [ fx_p1, fy_p1, cx_p1, cy_p1, //1-4 projector1 intrinsic
		%		 om11, om12, om13, //5-7 proj1 rotation
		%       t11, t12, t13, //8-10 proj1 translation
		%       fx_p2, fy_2, cx_p2, cy_p2, //11-14 proj2 intrinsic
		%       om21, om22, om23, //15-17 proj2 rotation
		%       t21, t22, t23, //18-20 proj2 translation
		%       ... //etc proj3...
		%       a, b, c, r, ///?1-?4 sphere pose
		%       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //?5-?3 camera intrinsic and lens distortion
		%
		% Returns:
		%		fi: pixel error for all points of all projectors.
		%		  [ P11x P11y P12x P12y ...  P21x P21y P22x P22y ...   ]
		%		  |<- projector 1 points ->|<- projector 2 points ->| ...
		%		jac: a large jacobian matrix for all points and params.
		%            An example of jac with 4 projectors:
		%		  |proj1 param|proj2 param|proj3 param|proj4 param|sphere param|camera param|
		% proj1pts[    A1          0            0           0			B1           C1     ]
		% proj2pts[    0           A2           0           0			B2           C2     ]
		% proj3pts[    0           0            A3          0			B3           C3     ]
		% proj4pts[    0           0            0           A4			B4           C4     ]
		%
		*/
		
		static void func_jac(const alglib::real_1d_array &params, alglib::real_1d_array &fi, alglib::real_2d_array &jac, void *ptr);
		
		/* 
		 * Cost Function Per Projector: computes pixel errors in camera 2D image coordinates by:
		 * 1. Ray-sphere intersection between projector ray and sphere
		 * 2. Camera projection that projects intersected 3D points onto camera image plane
		 * Each projector will have one cost function for all blob features. 
		 * It also has an option to compute jacobian matrices
		 *
		 % Input definition:
		 % params = [fx_p, fy_p, cx_p, cy_p, //1-4
		 %       om1, om2, om3, //5-7
		 %       t1, t2, t3, //8-10
		 %       a, b, c, r, //11-14
		 %       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //15-23
		 % cam_pts: camera blob 2D position in camera image frame
		 % proj_pts: projector blob 2D position in projector image frame. It requires the SAME number of points as cam_pts.
		 %
		 % Return:
		 %       A : NumberOfBlobs x 10->Jacobian matrix for projector intrinsic and extrinsic
		 %       B : NumberOfBlobs x 4->Jacobian matrix for sphere pose
		 %       C : NumberOfBlobs x 9->Jacobian matrix for camera intrinsics(with lens distortions)
		 %     err : 2 x NumberOfBlobs ->Jacobian matrix for pixel error
		 */
		static void function_mincost(const std::vector<float>& params, std::vector<cv::Point2f>& cam_pts, std::vector<cv::Point2f>& proj_pts, std::vector<double>& err, bool needJac, cv::Mat_<double>& jac_A = cv::Mat_<double>(), cv::Mat_<double>& jac_B = cv::Mat_<double>(), cv::Mat_<double>& jac_C = cv::Mat_<double>());
		
		static std::vector<std::vector<cv::Point2f>> m_campts;
		static std::vector<std::vector<cv::Point2f>> m_projpts;

		static int m_Nproj;

		struct blob_not_intersect : public std::exception {
			const char * what() const throw () {
				return "blob_not_intersect exception";
			}
		};
	};
}

#endif //! LM_OPTIM_H