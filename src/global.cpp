#include "global.h"

namespace multi_proj_calib
{
	namespace setting
	{
		namespace camera
		{
			extern int res_width = 1280; //resolution
			extern int res_height = 1024;
			extern uint cv_pixel_format = CV_8UC1;
			extern int checkerboard_col = 6; //pattern size
			extern int checkerboard_row = 8;
			extern double max_reproj_error = 1;
		}

		namespace proj
		{
			extern int res_width = 1024; //resolution
			extern int res_height = 768;
			extern int blobs_col = 8; //pattern size
			extern int blobs_row = 10;
			extern double max_reproj_error = 1;
		}

		namespace display
		{
			extern int num_proj = 4;
			extern int blobs_col = 15;
			extern int blobs_row = 18;
			extern float blob_radius = 20.f;
			extern double max_reproj_error = 3;
			extern double max_spherepose_error = 0.1;
		}
	}

	namespace file
	{
		extern std::string camcalib_file = "cam_calib.xml";
		extern std::string projcalib_file[4] = { "proj_calib1.xml", "proj_calib2.xml", "proj_calib3.xml", "proj_calib4.xml" };
		extern std::string blob_file[4] = { "Proj1PairBlobData.xml", "Proj2PairBlobData.xml", "Proj3PairBlobData.xml", "Proj4PairBlobData.xml" };
		extern std::string extrinsic_file = "extrinsics.xml";
		extern std::string gridtexture_file = "grid1p.DDS";
		extern std::string optimparam_file = "new_params.xml";
		extern std::string geom_file[4] = { "pro1pixel_.bin", "pro2pixel_.bin", "pro3pixel_.bin", "pro4pixel_.bin" };
		extern std::string blen_file[4] = { "pro1pixela_.bin", "pro2pixela_.bin", "pro3pixela_.bin", "pro4pixela_.bin" };
		extern std::string camgeom_file = "campixel_.bin";
		extern std::string data_path = "..\\data\\";
		extern std::string src_path = "..\\src\\";
	}
}