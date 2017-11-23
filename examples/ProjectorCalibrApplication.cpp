
#ifdef PROJ_CALIB
#include "../src/global.h"
#include "../src/CameraCalibration.h"
#include "../src/Flea3Cam.h"
//#include "utility.h"
#include "../src/ProjectorSys.h"
#include "../src/ProjectorCalibration.h"

using namespace multi_proj_calib;
using namespace std;
using namespace cv;

enum FsmStatus { detect_print = 0, detect_proj = 1 ,wait = 2, calibrate = 3, idle = 4, finish = 5 } mode;
void cvKeyBoardFsm(int delay_ms, FsmStatus& mode);
void onMouse(int event, int x, int y, int flags, void* userdata);

int main()
{
	/* modify this number before calibrating! File will be overwritten! */
	const int proj_idx = 1; //starts from 1

	unsigned int buf_width(0), buf_height(0);
	float dwidth(0), dheight(0);
	unsigned int err;

	Flea3Cam flea3_cam;
	err = flea3_cam.startCap();
	if (err != 0) { cout << "Fail to start capture" << endl; return -1; }
	flea3_cam.getImgSize(buf_width, buf_height);

	ProjectorSys projectors(1, setting::proj_width, setting::proj_height);
	projectors.initPattern(render::CIRCLE_GRID);
	projectors.startProj(PROJECTOR_1);
	projectors.getPatternDistancePixel(dwidth, dheight);
	
	Size blob_pattern, checkerbrd_pattern(8, 6);
	projectors.getPatternSize(blob_pattern.width, blob_pattern.height);

	CameraCalibration* p_cam_calib = new CameraCalibration;
	if (p_cam_calib == NULL) { cout << "Fail to initial p_cam_calib" << endl; return -1; }

	p_cam_calib->resetData();
	p_cam_calib->setImageParams(buf_width, buf_height);
	p_cam_calib->setPatternParams(CHECKER_BOARD, checkerbrd_pattern.width, checkerbrd_pattern.height, 1);
	p_cam_calib->createPatternObjectPoints();

	ProjectorCalibration* p_proj_calib = new ProjectorCalibration;
	if (p_proj_calib == NULL) { cout << "Fail to initial p_proj_calib" << endl; return -1; }

	p_proj_calib->resetData();
	p_proj_calib->setImageParams(setting::proj_width, setting::proj_height);
	p_proj_calib->setPatternParams(CIRCLE_GRID, blob_pattern.width, blob_pattern.height, 1);
	p_proj_calib->setPatternPixel(dwidth, dheight);
	p_proj_calib->setFrameCount(15, 12);
	//p_proj_calib->setStaticFrame(20);
	p_proj_calib->createImagePoints();

	mode = idle;
	string camera_window = "flea3 camera image";
	namedWindow(camera_window);
	setMouseCallback(camera_window, onMouse);
	bool keep_running = true;
	bool found = 0;
	clock_t prev_time_stamp = 0;

	Mat img_buf(buf_height, buf_width, CV_8UC1);
	vector<Point2f> chebrd_pts;
	vector<Point2f> cirgrd_pts;

	vector<Point3f> obj_pts;

	p_cam_calib->loadCalibParams(file::camcalib_file);

	while (keep_running)
	{
		err = flea3_cam.grabImg(img_buf.data, buf_width * buf_height);
		if (err != 0) { cout << "Fail to copy data" << endl; break; }

		switch (mode)
		{
		case detect_print:
			p_cam_calib->setPatternParams(CHECKER_BOARD, checkerbrd_pattern.width, checkerbrd_pattern.height, 1);
			found = p_cam_calib->detectPattern(img_buf, chebrd_pts);
			if (found)
			{
				p_cam_calib->computeBoardPose(chebrd_pts, p_proj_calib->getBoardRotation(), p_proj_calib->getBoardTranslation());
				mode = detect_proj;
				//if (p_proj_calib->currFrame() >= p_proj_calib->getStaticFrame())
				//	break;
			} 
			else
			{
				mode = idle;
				break;
			}
		case detect_proj:
			p_cam_calib->setPatternParams(CIRCLE_GRID, blob_pattern.width, blob_pattern.height, 1);
			found = p_cam_calib->detectPattern(img_buf, cirgrd_pts);
			if (found)
			{
				p_proj_calib->computeObjectPoints(p_cam_calib, cirgrd_pts, obj_pts);
				p_proj_calib->add(p_proj_calib->getPresetImagePoint(), obj_pts);
				p_cam_calib->add(cirgrd_pts, obj_pts);
			}		
			if (p_proj_calib->currFrame() >= p_proj_calib->getMinCalibFrame())
				mode = calibrate;
			else
				mode = idle;
			break;
		case calibrate:
			p_proj_calib->runCalibration(p_cam_calib);
			if (p_proj_calib->isReady())
				utils::printCalibResult(p_proj_calib);
			if (p_proj_calib->currFrame() >= p_proj_calib->getTotalFrame())
				p_proj_calib->saveProjCalibParams(file::projcalib_file[proj_idx - 1]);
			mode = idle;
			break;
		case finish:
			keep_running = false;
			break;
		case idle:
			break;
		default:
			break;
		}
		
		projectors.projPattern(PROJECTOR_1);

		projectors.projFlush();

		flea3_cam.showControlDlg(); //use this to adjust camera if room is too dark

		utils::drawDetectedPattern(img_buf, chebrd_pts, checkerbrd_pattern, found);
		utils::drawDetectedPattern(img_buf, cirgrd_pts, blob_pattern, found);
		
		p_cam_calib->undistortImage(img_buf);
		
		utils::showCameraImg(camera_window, img_buf, p_proj_calib->getCalibMsg()); 
		cvKeyBoardFsm(20, mode);
	} 

	delete p_cam_calib;
	delete p_proj_calib;

	err = flea3_cam.stopCap();
	if (err != 0) { cout << "Fail to stop capture" << endl; return -1; }

	projectors.stopProj(PROJECTOR_1);

	return 0;

}

void cvKeyBoardFsm(int delay_ms, FsmStatus& mode)
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
		mode = detect_print;
		cout << endl;
		cout << " /* start detection */" << endl;
		break;
	default:
		break;
	}
}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_RBUTTONDOWN)
	{
		cout << endl;
		mode = detect_print;
	}
}
#endif // PROJ_CALIB
