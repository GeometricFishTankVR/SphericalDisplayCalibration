//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: JdxdX.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Sep-2018 15:15:36
//
#ifndef __JDXDX_H__
#define __JDXDX_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "rt_nonfinite.h"
#include "rtwtypes.h"
namespace lmoptim {

	// Function Declarations
	extern void JdxdX(double X1, double X2, double X3, double fx_c, double fy_c,
		double cx_c, double cy_c, double kc1, double kc2, double kc3,
		double pc1, double pc2, double b_JdxdX[6]);
	
	extern void JdXdcp(double fx_p, double fy_p, double cx_p, double cy_p, double
		r11, double r21, double r31, double r12, double r22, double
		r32, double r13, double r23, double r33, double t1, double t2,
		double t3, double s1, double s2, double s3, double r, double
		x1_p, double x2_p, double b_JdXdcp[6]);

	extern void Jdxdfc(double X1, double X2, double X3, double fx_c, double fy_c,
		double cx_c, double cy_c, double kc1, double kc2, double kc3,
		double pc1, double pc2, double b_Jdxdfc[4]);

	extern void JdXdfp(double fx_p, double fy_p, double cx_p, double cy_p, double
		r11, double r21, double r31, double r12, double r22, double
		r32, double r13, double r23, double r33, double t1, double t2,
		double t3, double s1, double s2, double s3, double r, double
		x1_p, double x2_p, double b_JdXdfp[6]);

	extern void Jdxdkc(double X1, double X2, double X3, double fx_c, double fy_c,
		double cx_c, double cy_c, double kc1, double kc2, double kc3,
		double pc1, double pc2, double b_Jdxdkc[10]);

	extern void JdXdR(double fx_p, double fy_p, double cx_p, double cy_p, double r11,
		double r21, double r31, double r12, double r22, double r32,
		double r13, double r23, double r33, double t1, double t2,
		double t3, double s1, double s2, double s3, double r, double
		x1_p, double x2_p, double b_JdXdR[27]);

	extern void JdXdS(double fx_p, double fy_p, double cx_p, double cy_p, double r11,
		double r21, double r31, double r12, double r22, double r32,
		double r13, double r23, double r33, double t1, double t2,
		double t3, double s1, double s2, double s3, double r, double
		x1_p, double x2_p, double b_JdXdS[12]);

	extern void JdXdT(double fx_p, double fy_p, double cx_p, double cy_p, double r11,
		double r21, double r31, double r12, double r22, double r32,
		double r13, double r23, double r33, double t1, double t2,
		double t3, double s1, double s2, double s3, double r, double
		x1_p, double x2_p, double b_JdXdT[9]);
}
#endif

//
// File trailer for JdxdX.h
//
// [EOF]
//
