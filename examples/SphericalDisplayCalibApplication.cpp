
#include "../src/global.h"

#ifdef DISP_CALIB

#include "../src/DisplayCalibration.h"

using namespace multi_proj_calib;

int main()
{
	const int num_proj = 3;

	DisplayCalibration disp_calib(num_proj);
	disp_calib.setCalibrationMethod(dp_calib::SemiAuto);
	try {
		disp_calib.setup();
		disp_calib.testWindowSequence();
	}
	catch (const std::runtime_error& e) 
	{
		std::cerr << e.what() << std::endl;
		return -1; 
	}

#ifndef DBG_MATLAB_
	try {
		while (disp_calib.getCurrProjector() <= num_proj)
		{
			/// Project and capture blob pattern
			if (disp_calib.calibratePair()) 
			{
				/// Semiauto method has projector intrinsics; proceed to estimate extrinsics
				if (disp_calib.getCalibMethod() == dp_calib::SemiAuto)
				{
					/// Estimate extrinsics
					if (disp_calib.estimatePairExtrinsic())
					{
						disp_calib.getCurrProjector()++;
					}
				}
				else
				{
					disp_calib.getCurrProjector()++;
				}
			}
		} /* end of loop */
		
		/* sphere pose estimation */
		if (disp_calib.getCalibMethod() == dp_calib::SemiAuto)
		{
			disp_calib.estimateSpherePose();
			disp_calib.saveExtrinsics(file::data_path+file::extrinsic_file);
		}
	}
	catch (const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		return -1;
	}
	
	/* todo: bundle adjustment */
	//disp_calib.bundleAdjustment();

#else
	/* debug: load NLO results from matlab */
	try {
		disp_calib.updateParams();
		//disp_calib.loadCalibrationResult();
		
		/* compute geometry data */

		disp_calib.computePixel3D();

		/* create alpha mask */

		disp_calib.computeAlphaMask();
		/// save unnormalized blending data

		/* Normalize and save geometry/blending data*/

		disp_calib.saveCalibrationResult();

		/* display calibrated result */
	}
	catch (std::runtime_error& e)
	{
		std::cerr << e.what() << std::endl;
	}
	disp_calib.displayCorrectedPattern();

#endif
	if(disp_calib.cleanup())
		return 0;
}

#endif 