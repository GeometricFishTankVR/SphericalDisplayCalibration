/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_FLEA3CAM_H_
#define CALIB_FLEA3CAM_H_

#include "stdio.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

#include "global.h"

#include "FlyCapture2.h"
#include "FlyCapture2GUI.h"

class Flea3Cam
{
public:
	//! the img size should match cam_width and cam_height in global.h
	void getImgSize( unsigned int& width, unsigned int& height);
	//! start capture: call before loop
	int startCap();
	//! start capture: call after loop
	int stopCap();
	//! copy img data: call in loop
	int grabImg( unsigned char* dest_mat, const size_t & num);
	//! show control dialog: call in loop
	void showControlDlg(); 

	void setParams(std::string param_name, float value);

	Flea3Cam();
	Flea3Cam( const FlyCapture2::Mode & fm7_mode, 
		const FlyCapture2::PixelFormat & fmt7_pix);
	~Flea3Cam();
private:
	int camInit();
	
	int setFormat7();

	void printBuildInfo();
	int printCamInfo();
	void printFormat7(FlyCapture2::Format7Info* pfmt7_info);
	void printError();

	FlyCapture2::Camera m_cam;
	FlyCapture2::CameraControlDlg m_cam_control;
	FlyCapture2::Image m_raw_img;
	FlyCapture2::Property m_prop;


	FlyCapture2::Error m_err;
	FlyCapture2::BusManager m_bus_mgr;
	FlyCapture2::PGRGuid m_guid;
	FlyCapture2::Format7ImageSettings m_fmt7_img;

	unsigned int m_num_cam;
	FlyCapture2::Mode m_fmt7_mode;
	FlyCapture2::PixelFormat m_fmt7_pix;
};

template<class T>
bool check_range(T value, T min, T max) {
	return (value >= min) && (value <= max);
}

#endif // !CALIB_FLEA3CAM_H_
