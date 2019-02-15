# Multiple-Projector Spherical Screen Calibration

Source code for the paper:

Q. Zhou, G. Miller, K. Wu and S. Fels. [Automatic calibration of a multiple-projector spherical fish tank VR display](http://ieeexplore.ieee.org/abstract/document/7926707/). WACV 2017.   ([VIDEO](https://youtu.be/Dgs4FmHCvp8))

- Multiple-Projector Spherical Screen Calibration: this approach calibrates a multi-projector spherical screen. It recovers the geometry information (3d coordinates) of each pixel on the screen and computes the blending information (alpha mask) which adjusts the intensity of overlapping area for adjacent projectors. The result can be used to render wall-papered content on the surface and 3D content in the sphere ([video](https://youtu.be/KVKyXYCttfA)). 

- There are two related approaches using this library to calibrate the spherical screen: an automatic calibration approach and a semi-automatic approach. Both the automatic and semiautomatic approach require a calibrated camera with know intrinsic matrix and lens distortions. The semiautomatic approach requires additional information of projector instrinsic matrix as initial guess, while the automatic approach computes the projector intrinsics based on projected patterns on the spherical screen. The pipelines of each approach are summarized as below:

  - Automatic Calibration Approach
    - Step1. Calibrate the camera intrinsic
    - Step2. Project patterns onto the spherical screen and detect patterns with the calibrated camera
    - Step3. Compute initial guess of projector intrinsics, extrinsics and sphere pose using detected patterns
    - Step4. Non-linear Optimization of all intrinsics, extrinsics, and sphere pose
    - Step5. Compute 3d coordinates of each pixel per projector
    - Step6. Compute alpha mask of each pixel per projector 
    - Note: 
      - Example of Step1 is in the file *CameraCalibApplication.cpp*
      - Example of Step2-6 is in the file *SphericalDisplayCalibApplication.cpp*. ~~Currently Step3 and Step4 are temporarily implemented in MATLAB with the file *main_automatic_calib.m*.~~ Updated: all steps are implemented in cpp files. 
      - Assumption of this approach: the principle point of the projector is at the bottom center of the image plane. If the assumption does not hold, please use the Semi-automatic approach. Also, since the initial guess solely depends on the detection of projected patterns, the error of the detection may make the initial guess far away from the global minimum in the non-linear optimization, causing the final solution to be a local minimum. **It is strongly recommended to use semi-automatic approach to calibrate an initial guess for projectors first. **
  - Semi-Automatic Calibration Approach
    - Step1. Calibrate the camera intrinsic
    - Step2. Calibrate the projector intrinsic
    - Step3. Project patterns onto the spherical screen and detect patterns with the calibrated camera
    - Step4. Non-linear Optimization of all intrinsics, extrinsics, and sphere pose
    - Step5. Compute 3d coordinates of each pixel per projector
    - Step6. Compute alpha mask of each pixel per projector
    - Note:
      - Example of Step1 is in the file *CameraCalibApplication.cpp*
      - Example of Step2 is in the file *ProjectorCalibApplication.cpp*. If all projectors are the same model with roughly the same focal length, it is possible to just calibrate one projector and use it as initial guess for the rest.
      - Example of Step3-6 is in the file *SphericalDisplayCalibApplication.cpp*. 


# Demo

- Camera intrinsic calibration example
  - File: *CameraCalibApplication.cpp*
  - How-to-use: Press 's' to start, then detection will be automatic (roughly 2 Sec per frame). Once it reaches a minimum number of frames, the calibration will be triggered, then cleaning will be done to remove frames with large reprojection errors. By the end of calibration, camera intrinsic will be saved (normally with the reprojection error no more than 0.3).
  - Video: https://youtu.be/_vp6XlQdERg
  - Notes: 
    - If strong distortions on lens: make sure the chart reaches image plane boundaries so that barrel curves can be observed.
    - When spatial variance of the board is small, reprojection error would be high.
- Projector intrinsic calibration example
  - File: *ProjectorCalibApplication.cpp*
  - How-to-use: Right click mouse in the camera window once both the projected pattern and physical pattern are inside the camera view. Once it reaches a minimum number of frames, the calibration will be triggered, then cleaning will be done to remove frames with large reprojection errors. By the end of calibration, projector intrinsic will be saved (normally with the reprojection error no more than 0.5).
  - Video: https://youtu.be/gG2URmbu0Ik
  - Notes:
    - Only one projector + primary monitor are connected when running. 
    - The board should be rigid and as close to a plane as possible. 
    - When spatial variance of the board is small, reprojection error would be high.
    - Turn off any auto-functionalities on the projector: auto-keystone correction etc.
- Spherical display calibration example
  - File: *SphericalDisplayCalibApplication.cpp*, *main_automatic_calib.m* or *main_semiauto_calib.m*
  - How-to-use: 
    - Run *SphericalDisplayCalibApplication.cpp*, follow the instruction on the console window: 
      - Determine the projector order (projector #1, #2, etc); 
      - Define the projection circle (by clicking on the circle) on the spherical screen so that all pattern features are within the circle from the camera view; 
      - Blob projection and detection will be triggered for each projector; reprojection error will show up for each projector if it is in the Semi-automatic mode (normally below 2). 
      - Once all projectors have projected, feature data will be saved. If it is in the Semi-automatic mode, the initial guess of projector extrinsics and sphere pose will also be saved. 
    - ~~In the automatic calibration mode, an initial guess of extrinsics and sphere pose will be first generated and non-linear optimization will be applied. In the semiautomatic calibration mode, the initial guess will be read from file and non-linear optimization will be applied on that. Refined parameters will be saved after the optimization.~~
    - ~~Go back to *SphericalDisplayCalibApplication.cpp*, change the isOptimized flag to be true and run it. It will compute 3d coordinates and alpha mask automatically.~~ 
    - By the end of computation, a stitched grid pattern will appear.
  - Video: [auto](https://youtu.be/Fs4aBVG1dpM) [semiauto](https://youtu.be/V2vnG_PL8KQ)
  - Notes:
    - When the baseline between camera and projector is too short, the fundamental matrix might be inaccurate.
    - When observed projection area on Surface is too small (close to a plane), the fundamental matrix might be inaccurate. 
    - Blob detection is lighting-sensitive. May require adjusting the camera parameters and detector parameters based on the lighting. 
    - If the overlapping area across projectors is out-of-focus for projectors, the ghosting effect is likely to happen. 
- Sample data
  - *cam_calib.xml*: camera intrinsics for the Camera intrinsic calibration example
  - *proj_calibx.xml*: projector intrinsics of three projectors for the Projector intrinsic calibration example
  - *ProjxPairBlobData.xml*: the 2D positions of detected patterns per projector-camera pair for the Spherical display calibration example
  - *extrinsics.xml*: three projectors extrinsics and the sphere pose for the Semi-Automatic mode of Spherical display calibration example
  - *new_params.xml*: optimized params from matlab, including camera intrinsics, projector intrinscis, extrinsics and sphere pose.
  - All the sample data are in the data folder using the hardware described below.

# Hardware

- One Camera: Flea3 [FL3-U3-13Y3M-C](https://www.ptgrey.com/flea3-13-mp-mono-usb3-vision-vita-1300-camera)
- Three Projector: Asus [P2B](https://www.google.ca/search?q=Asus+P2B&oq=Asus+P2B&aqs=chrome..69i57j0l5.391j0j7&sourceid=chrome&ie=UTF-8) 
- Spherical screen: An acrylic spherical screen with the diameter of 30cm diameter and a projection hole of 21cm diameter

# Dependency

- OpenCV-2.4.13

- glew-1.12.0

- glfw-3.2

- glm-0.9.4.0

- FlyCapture2-2.9.3.43




