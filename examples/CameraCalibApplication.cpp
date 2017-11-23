#ifdef CAM_CALIB
#include "../src/global.h"
#include "../src/CameraCalibration.h"
#include "../src/Flea3Cam.h"
//#include "utility.h"

using namespace multi_proj_calib;
using namespace std;
using namespace cv;

enum CameraFsmStatus { detect = 0, wait = 1, calibrate = 2, idle = 3, finish = 4 };
void cvKeyBoardFsm(int delay_ms, CameraFsmStatus& mode);

int main()
{	

	unsigned int buf_width(0), buf_height(0);
	unsigned int err; 

	Flea3Cam flea3_cam;

	flea3_cam.getImgSize(buf_width, buf_height);
	Mat img_buf(buf_height, buf_width, CV_8UC1);

	err = flea3_cam.startCap();
	if (err != 0){	cout << "Fail to start capture" << endl; return -1;}

	CameraCalibration* p_cam_calib = new CameraCalibration;
	if (p_cam_calib == NULL) {	cout << "Fail to initial p_cam_calib" << endl; return -1; }
	
	p_cam_calib->resetData();
	p_cam_calib->setImageParams(buf_width, buf_height);
	p_cam_calib->setFrameCount(18, 15);
	p_cam_calib->createPatternObjectPoints();

	CameraFsmStatus mode = idle;

	bool keep_running = true;
	bool found = 0;
	clock_t prev_time_stamp = 0;

	vector<Point2f> feature_pts;

	string file_name = file::camcalib_file;
	while (keep_running)
	{
		err = flea3_cam.grabImg(img_buf.data, buf_width * buf_height);
		if (err != 0){	cout << "Fail to copy data" << endl; break; }

		switch (mode)
		{
		case detect:
			p_cam_calib->setPatternParams(CHECKER_BOARD, 8, 6, 1);
			found = p_cam_calib->detectPattern(img_buf, feature_pts);
			if (found)
			{
				p_cam_calib->add(feature_pts, p_cam_calib->getPresetObjectPoints());
				utils::drawDetectedPattern(img_buf, feature_pts, Size(8,6),found);
				cv::imwrite("Gray_Image" + to_string(p_cam_calib->currFrame()) + ".jpg", img_buf); //dbg
			}
			if ( p_cam_calib->currFrame() >= p_cam_calib->getMinCalibFrame() )
				mode = calibrate;
			else
				mode = wait;
			break;
		case wait:
			if (utils::timeOut(2, prev_time_stamp))
				mode = detect;
			utils::drawDetectedPattern(img_buf, feature_pts, Size(8, 6), found);
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
				utils::printCalibResult(p_cam_calib);
				p_cam_calib->saveCalibParams(file_name);
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
		utils::showCameraImg("wind", img_buf, p_cam_calib->getCalibMsg());
			
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

#endif // CAM_CALIB
