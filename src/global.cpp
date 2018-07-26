#include "global.h"

namespace multi_proj_calib
{
	namespace setting
	{
		extern const int proj_width = 1024;
		extern const int proj_height = 768;
		extern const int cam_width = 1280;
		extern const int cam_height = 1024;
		extern const uint cv_pixel_format = CV_8UC1;
		extern const float blob_radius = 20.f;
		extern const int blob_col = 15;
		extern const int blob_row = 18;
	}

	namespace file
	{
		extern const std::string camcalib_file = "cam_calib.xml";
		extern const std::string projcalib_file[4] = { "proj_calib1.xml", "proj_calib2.xml", "proj_calib3.xml", "proj_calib4.xml" };
		extern const std::string blob_file[4] = { "Proj1PairBlobData.xml", "Proj2PairBlobData.xml", "Proj3PairBlobData.xml", "Proj4PairBlobData.xml" };
		extern const std::string extrinsic_file = "extrinsics.xml";
		extern const std::string gridtexture_file = "grid1p.DDS"; 
		extern const std::string optimparam_file = "new_params.xml";
		extern const std::string geom_file[4] = { "pro1pixel_.bin", "pro2pixel_.bin", "pro3pixel_.bin", "pro4pixel_.bin" };
		extern const std::string blen_file[4] = { "pro1pixela_.bin", "pro2pixela_.bin", "pro3pixela_.bin", "pro4pixela_.bin" };
		extern const std::string camgeom_file = "campixel_.bin";
		extern const std::string data_path = "..\\data\\";
		extern const std::string src_path = "..\\src\\";

	}
}