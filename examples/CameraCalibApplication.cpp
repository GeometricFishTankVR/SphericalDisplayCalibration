#include <iostream>

#include "../src/global.h"
#include "../src/CameraCalibration.h"
#include "../src/Flea3Cam.h"

using namespace multi_proj_calib;
using namespace cv;
using std::endl;
using std::cout;

enum CameraFsmStatus { detect = 0, wait = 1, calibrate = 2, idle = 3, finish = 4 };
void cvKeyBoardFsm(int delay_ms, CameraFsmStatus& mode);
bool timeOut(int delay_s, clock_t& prev_time_stamp);

/* CameraCalibApplication: calibrate camera intrinsics. Capture image with chessboard pattern automatically each a few seconds. */

int main()
{	
	/* flea3 camera setup */

	Flea3Cam flea3_cam;

	uint buf_width(0), buf_height(0);
	uint err;

	flea3_cam.getImgSize(buf_width, buf_height);
	Mat img_buf(buf_height, buf_width, CV_8UC1);

	err = flea3_cam.startCap();
	if (err != 0){	cout << "Fail to start capture" << endl; return -1;}

	/* camera calibration setup */

	CameraCalibration* p_cam_calib = new CameraCalibration;
	if (p_cam_calib == NULL) {	cout << "Fail to initial p_cam_calib" << endl; return -1; }
	
	p_cam_calib->resetData();
	p_cam_calib->setImageParams(buf_width, buf_height);
	p_cam_calib->setFrameCount(18, 15);
	p_cam_calib->createPatternObjectPoints();

	/* calibration implementation */
	
	CameraFsmStatus mode = idle;
	bool keep_running = true;
	bool found = false;
	
	clock_t prev_time_stamp = 0;
	const int DELAY_SEC = 3; // use this to control the delay of capture between images
	
	vector<Point2f> feature_pts;

	while (keep_running)
	{
		err = flea3_cam.grabImg(img_buf.data, buf_width * buf_height);
		if (err != 0){	cout << "Fail to copy data" << endl; break; }

		switch (mode)
		{
		case detect:
			p_cam_calib->setPatternParams(CHECKER_BOARD);
			found = p_cam_calib->detectPattern(img_buf, feature_pts);
			if (found)
			{
				p_cam_calib->add(feature_pts, p_cam_calib->getPresetObjectPoints());
				p_cam_calib->drawDetectedPattern(img_buf, feature_pts);
				//cv::imwrite("Gray_Image" + std::to_string(p_cam_calib->currFrame()) + ".jpg", img_buf); 
			}
			if ( p_cam_calib->currFrame() >= p_cam_calib->getMinCalibFrame() )
				mode = calibrate;
			else
				mode = wait;
			break;
		case wait:
			if (timeOut(DELAY_SEC, prev_time_stamp))
				mode = detect;
			p_cam_calib->drawDetectedPattern(img_buf, feature_pts);
			break;
		case calibrate:
			p_cam_calib->runCalibration();
			if (p_cam_calib->currFrame() < p_cam_calib->getTotalFrame())
			{
				prev_time_stamp = clock();
				mode = wait;
			}
			else
			{
				p_cam_calib->printIntrinsics();
				p_cam_calib->saveCalibParams(file::data_path + file::camcalib_file);
				mode = idle;
			}
			break;
		case finish:
			keep_running = false;
			break;
		case idle:
			prev_time_stamp = clock();
			if (p_cam_calib->currFrame() >= p_cam_calib->getTotalFrame())
				p_cam_calib->undistortImage(img_buf);
			break;
		default:
			break;
		}
		flea3_cam.showControlDlg();
		
		cv::putText(img_buf, p_cam_calib->getCalibMsg(), cv::Point((int)img_buf.rows*.6, (int)img_buf.cols*.7), 1, 2, cv::Scalar(255, 0, 0));
		cv::imshow("wind", img_buf);

		cvKeyBoardFsm(20, mode);
	} // end of loop

	delete p_cam_calib;
	err = flea3_cam.stopCap();
	if (err != 0) {	cout << "Fail to stop capture" << endl; return -1; }
	
	return 0;

}

void cvKeyBoardFsm(int delay_ms, CameraFsmStatus& mode)
{
	char key = cv::waitKey(delay_ms);
	switch (key)
	{
	case 'q':
	case 'Q':
		mode = finish;
		break;
	case 's':
	case 'S':
		mode = detect;
		break;
	default:
		break;
	}
}

bool timeOut(int delay_s, clock_t& prev_time_stamp)
{
	bool time_out = (float)(clock() - prev_time_stamp) / CLOCKS_PER_SEC > delay_s;
	if (time_out)
		prev_time_stamp = clock();
	return time_out;
}
