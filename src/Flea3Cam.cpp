// file: Flea3Cam.cpp
//
// date: 2016-06-15

#include "Flea3Cam.h"

using namespace FlyCapture2;
using namespace std;

Flea3Cam::Flea3Cam()
{
	m_fmt7_mode = MODE_0;
	m_fmt7_pix = PIXEL_FORMAT_MONO8;
	//m_err = PGRERROR_OK;
	if (camInit() != 0)
		cout << "Fail to initialize the flea3 camera" << endl;
}

Flea3Cam::Flea3Cam( const Mode & fm7_mode, const PixelFormat & fmt7_pix )
{
	m_fmt7_mode = fm7_mode;
	m_fmt7_pix = fmt7_pix;
	//m_err = PGRERROR_OK;
	if (camInit() != 0)
		cout << "Fail to initialize the flea3 camera" << endl;
}

Flea3Cam::~Flea3Cam()
{
	// Disconnect the camera
	m_err = m_cam.Disconnect();
	if ( m_err != PGRERROR_OK )
		printError();
}

void Flea3Cam::printError()
{
	m_err.PrintErrorTrace();
}

void Flea3Cam::printBuildInfo()
{
	FC2Version fc2Version;
	Utilities::GetLibraryVersion( &fc2Version );

	ostringstream version;
	version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
	cout << version.str() << endl;

	ostringstream time_stamp;
	time_stamp << "Application build date: " << __DATE__ << " " << __TIME__;
	cout << time_stamp.str() << endl << endl;
}

int Flea3Cam::printCamInfo()
{
	CameraInfo cam_info;
	m_err = m_cam.GetCameraInfo( &cam_info );
	if ( m_err != PGRERROR_OK )
	{
		printError();
		return -1;
	}

	cout << endl;
	cout << "*** CAMERA INFORMATION ***" << endl;
	cout << "Serial number -" << cam_info.serialNumber << endl;
	cout << "Camera model - " << cam_info.modelName << endl;
	cout << "Camera vendor - " << cam_info.vendorName << endl;
	cout << "Sensor - " << cam_info.sensorInfo << endl;
	cout << "Resolution - " << cam_info.sensorResolution << endl;
	cout << "Firmware version - " << cam_info.firmwareVersion << endl;
	cout << "Firmware build time - " << cam_info.firmwareBuildTime << endl << endl;
	return 0;
}

void Flea3Cam::printFormat7( Format7Info* pfmt7_info )
{
	cout << "Max image pixels: (" << pfmt7_info->maxWidth << ", " << pfmt7_info->maxHeight << ")" << endl;
	cout << "Image Unit size: (" << pfmt7_info->imageHStepSize << ", " << pfmt7_info->imageVStepSize << ")" << endl;
	cout << "Offset Unit size: (" << pfmt7_info->offsetHStepSize << ", " << pfmt7_info->offsetVStepSize << ")" << endl;
	cout << "Pixel format bitfield: 0x" << pfmt7_info->pixelFormatBitField << endl;
}

int Flea3Cam::setFormat7()
{
	Format7Info fmt7Info;
	bool supported;
	fmt7Info.mode = m_fmt7_mode;
	m_err = m_cam.GetFormat7Info( &fmt7Info, &supported );
	if ( m_err != PGRERROR_OK )
	{
		printError();
		return -1;
	}
	printFormat7(&fmt7Info);

	if ((m_fmt7_pix & fmt7Info.pixelFormatBitField) == 0)
	{
		cout << "Pixel format is not supported" << endl;
		return -1;
	}

	m_fmt7_img.mode = m_fmt7_mode;
	m_fmt7_img.offsetX = 0;
	m_fmt7_img.offsetY = 0;
	m_fmt7_img.width = fmt7Info.maxWidth;  // user defined 
	m_fmt7_img.height = fmt7Info.maxHeight;  //user defined
	m_fmt7_img.pixelFormat = m_fmt7_pix;

	bool valid;
	Format7PacketInfo fmt7PacketInfo;

	// Validate the settings to make sure that they are valid
	m_err = m_cam.ValidateFormat7Settings(
		&m_fmt7_img,
		&valid,
		&fmt7PacketInfo);
	if ( m_err != PGRERROR_OK )
	{
		printError();
		return -1;
	}

	if ( !valid )
	{
		// Settings are not valid
		cout << "Format7 settings are not valid" << endl;
		return -1;
	}

	// Set the settings to the camera
	m_err = m_cam.SetFormat7Configuration(
		&m_fmt7_img,
		fmt7PacketInfo.recommendedBytesPerPacket);
	if (m_err != PGRERROR_OK)
	{
		printError();
		return -1;
	}
	cout << "Format7 settings are successful" << endl;
	return 0;
}

int Flea3Cam::camInit()
{
	printBuildInfo();
	
	m_err = m_bus_mgr.GetNumOfCameras( &m_num_cam );
	
	if (m_err != PGRERROR_OK)
	{
		printError();
		return -1;
	}
	
	cout << "Number of cameras detected: " << m_num_cam << endl;
	
	if ( m_num_cam < 1)
	{
		cout << "Insufficient number of cameras... exiting" << endl;
		return -1;
	}

	m_err = m_bus_mgr.GetCameraFromIndex( 0, &m_guid );
	if ( m_err != PGRERROR_OK )
	{
		printError();
		return -1;
	}

	m_err = m_cam.Connect( &m_guid );
	if ( m_err != PGRERROR_OK)
	{
		printError();
		return -1;
	}

	// Get the camera information
	CameraInfo cam_info;
	m_err = m_cam.GetCameraInfo( &cam_info);
	if ( m_err != PGRERROR_OK)
	{
		printError();
		return -1;
	}

	if ( printCamInfo() != 0)
	{
		cout << "Fail to get Camera info" << endl;
		return -1;
	}

	if ( setFormat7() != 0)
	{
		cout << "Fail to set format 7" << endl;
		return -1;
	}

	m_prop.type = FRAME_RATE;
	m_err = m_cam.GetProperty( &m_prop );
	if ( m_err != PGRERROR_OK)
	{
		printError();
		return -1;
	}
	cout << "Frame rate is " << fixed << setprecision(2) << m_prop.absValue << " fps" << endl;

	m_cam_control.Connect(&m_cam);
	m_cam_control.Hide();
	return 0;
}

void Flea3Cam::getImgSize(  unsigned int & width, unsigned int & height )
{
	width = m_fmt7_img.width;
	height = m_fmt7_img.height;
}

int Flea3Cam::startCap()
{
	m_err = m_cam.StartCapture();
	if ( m_err != PGRERROR_OK )
	{
		printError();
		return -1;
	}
	return 0;
}

int Flea3Cam::stopCap()
{
	m_err = m_cam.StopCapture();
	if ( m_err != PGRERROR_OK )
	{
		printError();
		return -1;
	}
	m_cam_control.Hide();
	return 0;
}

int Flea3Cam::grabImg( unsigned char* dest_mat, const size_t & num )
{
	m_err = m_cam.RetrieveBuffer( &m_raw_img);
	if ( m_err != PGRERROR_OK)
	{
		printError();
		return -1;
	}
	memcpy( dest_mat, m_raw_img.GetData(), num );
	return 0;
}

void Flea3Cam::setParams(std::string param, float val)
{
	if (!param.compare("exposure"))
	{
		m_prop.type = AUTO_EXPOSURE;
		if (!check_range<float>(val, -7.5f, 2.4f)) // -7.5EV ~ 2.4EV
		{
			std::cout << "Flea3Cam::setParams(): exposure value out of range, setting value fails. " << std::endl;
			return;
		}
	}
	else if (!param.compare("shutter"))
	{
		m_prop.type = SHUTTER;
		if (!check_range<float>(val, 0.f, 10.f)) // 0 ~ 10 ms
		{
			std::cout << "Flea3Cam::setParams(): shutter value out of range, setting value fails. " << std::endl;
			return;
		}
	}
	else if (!param.compare("gain"))
	{
		m_prop.type = GAIN;
		if (!check_range<float>(val, 0.f, 18.f)) // 0 ~ 18 dB
		{
			std::cout << "Flea3Cam::setParams(): gain value out of range, setting value fails. " << std::endl;
			return;
		}
	}
	else if (!param.compare("brightness"))
	{
		m_prop.type = BRIGHTNESS;
		if (!check_range<float>(val, 0.f, 25.f)) // 0 ~ 25 %
		{
			std::cout << "Flea3Cam::setParams(): brightness value out of range, setting value fails. " << std::endl;
			return;
		}
	}
	else if (!param.compare("sharpness"))
	{
		m_prop.type = SHARPNESS;
		if (!check_range<float>(val, 0.f, 4095.f)) // 0 ~4095
		{
			std::cout << "Flea3Cam::setParams(): sharpness value out of range, setting value fails. " << std::endl;
			return;
		}
	}
	else
	{
		std::cout << "Flea3Cam::setParams(): No Param found" << std::endl;
		return;
	}
	m_prop.onOff = true;
	m_prop.autoManualMode = false;
	m_prop.absControl = true;
	m_prop.absValue = val;
	m_err = m_cam.SetProperty(&m_prop);
}

void Flea3Cam::showControlDlg()
{
	if ( m_cam_control.IsVisible() == false )
		m_cam_control.Show();
}
