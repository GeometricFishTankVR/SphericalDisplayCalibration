===== Library Dependences =====

- CalibrationBase.h: Base class that supports functionalities to calibrate a single device using the pinhole-camera-model. 
	- CameraCalibration: derived from CalibrationBase. Class for camera calibration. 
	- ProjectorCalibration: derived from CalibrationBase. 

- Flea3Cam.h: Camera class that gets data from camera using flyCapture2 library (FlyCapture2 2.9.3.43)

- ProjectorSys.h: Projectors class that control projectors using CalibRenderGL.
	- CalibRenderGL: Rendering Class for Calibration that controls projection patterns using OpenGL, GLEW, GLM, and GLFW (glew-1.12.0, glfw-3.2 and glm-0.9.4.0)

- DisplayCalibration.h: Core calibration class that calibrates the spherical display and computes 3D position for each pixel on the sphere using Flea3Cam, ProjectorSys and results from CameraCalibration, ProjectorCalibration.

- FundamentalMat.h: class that estimates the Fundamental Matrix, mostly controlled by m_thre_ratio. (Temporary class to estimate Essential Matrix, will be substitued by existing libray that gives a better estimation of essential matrix)

- Shader.h: Shader class for OpenGL

External Libraries: 
OpenCV-2.4.13
glew-1.12.0
glfw-3.2
glm-0.9.4.0
FlyCapture2-2.9.3.43

===== Notes of Calibration =====

* Camera calibration:

1. If strong distortions on lens: make sure the chart reaches image plane boundaries so that barrel curves can be observed.

2. When spatial variance is small, reprojection error would be high.

!How to use: 
Uncomment only #define CAM_CALIB in global.h. 
Press 's' to start, then detection will be automatic (roughly 2sec per img). Once it reaches a minimum amount of frames, the calibration will be triggered, then cleaning will be done to remove frames with large reprojection errors. By the end of calibration, camera intrinsic will be saved (normally with the reprojection error no more than 0.3).

* Projetcor calibration:

1. Make sure only one projector + your dbg monitor are turned on. 

2. Board you are using should be close to a plane. Better to be wood or any material that makes it rigid.

3. When spatial variance is small, reprojection error will be extremely high.

!How to use: 
Uncomment only #define PROJ_CALIB in global.h 
Run it. Right click mouse in the camera window once you find a nice spot that have both patterns inside the camera view. The rest is the same as camera calibration. (The final reproj err will be around 0.5)

* Display Calibration

1. When baseline is too short, fundemental matrix found might be quite inaccurate.

2. When observed projection area on Surface is small (close to a plane), the estimation of fundemental matrix will be inaccurate. 

3. Turn off any auto-functionalities on the projector: auto-keystone correction etc.

4. Blob detection/Projection hole Detection is light sensitive. May require to tweak detector params of these two. Currently the accuracy of detection depends on the light condition, improvements are on the way for different light conditions by automatically adjusting exposure/shutter speed based on captured images.

5. Be careful with out-of-focus area: if overlapping area is out-of-focus by projector, the ghosting effect is likely to happen. 

!How to use: 
Uncomment only #define DISP_CALIB in global.h 
<update>: Left click the bounding circle four times to locate the bounding circle (see the new video).
Then blob projection will start after that.
Two params are critical in this process: s_ratio( >.99) from fundamental mat and reproj-err(< 3) for each projector. 
Once the process is done, the ray-sphere intersection will compute the 3D pixel position. An image of grid will show up from the camera's viewpoint. 
Then switch to matlab script/main_params_refinement.m. Run it. Copy'n Paste data from New_Params.txt to DisplayCalibration::updateParams(). Uncomment #define DBG_MATLAB_ in global.h. Run again. This time it should update the refined params and do ray-sphere intersection again. A better grid pattern with or without alpha mask will show up depending on whether you uncomment the blending.

-----------------------------------
* To-dos:

1. Test/debug with display with more than 2 projectors: done
2. Robust Blob Detector and Circle Detector that deal with different light conditions: use flyCapture APIs to dynamically adjusting exposure and shutter speed to improve the quality of image.
3. Implement nonlinear optimization in C++ to make it fully automatic using LM packages below.
4. Initial guess of scaling factor between paris can be improved as stated in WACV paper.

------------------------------------
* Useful links for LM/sparseLM setup:

LM
http://users.ics.forth.gr/~lourakis/levmar/

sparseLM
http://users.ics.forth.gr/~lourakis/sparseLM/

sba
http://users.ics.forth.gr/~lourakis/sba/index.html

CLAPACK
http://www.netlib.org/clapack/LIB_WINDOWS/prebuilt_libraries_windows.html

cvsba: an OpenCV wrapper for sba library
http://www.uco.es/investiga/grupos/ava/node/39

Using Sparse Bundle Adjustment (SBA) in windows
https://planetanacreon.wordpress.com/2011/06/02/using-sparse-bundle-adjustment-sba-in-windows/
