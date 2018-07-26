
#include "../src/global.h"
#include "../src/DisplayCalibration.h"

using namespace multi_proj_calib;

int main()
{
	const int num_proj = 4;
	bool isOptimized = false; // change this flag to be true if the params have been optimized by matlab

	DisplayCalibration disp_calib(num_proj);
	disp_calib.setCalibrationMethod(dp_calib::SemiAuto);
	try {
		disp_calib.setup();
		disp_calib.testWindowSequence();
	}
	catch (const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		return -1; 
	}

	if (!isOptimized){
		try {
			while (disp_calib.getCurrProjector() <= num_proj)
			{
				/// Project and capture blob pattern
				if (disp_calib.calibratePair()){
					/// Semiauto method has projector intrinsics; proceed to estimate extrinsics
					if (disp_calib.getCalibMethod() == dp_calib::SemiAuto){
						/// Estimate extrinsics
						if (disp_calib.estimatePairExtrinsic()){
							disp_calib.getCurrProjector()++;
						}
					}
					else{
						disp_calib.getCurrProjector()++;
					}
				}
			} 

			/// sphere pose estimation
			if (disp_calib.getCalibMethod() == dp_calib::SemiAuto){
				disp_calib.estimateSpherePose();
				disp_calib.saveExtrinsics(file::data_path + file::extrinsic_file);
			}
		}
		catch (const std::runtime_error& e) {
			std::cerr << e.what() << std::endl;
			return -1;
		}
	}
	else
	{
		try {
			/// update params after optimization from maltab
			disp_calib.updateParams();

			/// compute geometry data 
			disp_calib.computePixel3D();

			/// create alpha mask 
			disp_calib.computeAlphaMask();

			/// Normalize and save the geometry or blending data
			disp_calib.saveCalibrationResult();

			/// display the calibrated result
			disp_calib.displayCorrectedPattern();
		}
		catch (std::runtime_error& e){
			std::cerr << e.what() << std::endl;
		}
	}

	if(disp_calib.cleanup())
		return 0;
}