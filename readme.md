# Multiple-Projector Spherical Screen Calibration

Source code for the paper:

Q. Zhou, G. Miller, K. Wu and S. Fels. [Automatic calibration of a multiple-projector spherical fish tank VR display](http://ieeexplore.ieee.org/abstract/document/7926707/). WACV 2017.   ([VIDEO](https://youtu.be/Dgs4FmHCvp8))

- Multiple-Projector Spherical Screen Calibration: this approach calibrates a multi-projector spherical screen. It recovers the geometry information (3d coordinates) of each pixel on the screen and computes the blending information (alpha mask) which adjusts the intensity of overlapping area for adjacent projectors. The result can be used to render wall-papered content on the surface and 3D content in the sphere ([video](https://youtu.be/KVKyXYCttfA)). 

- There are two related approaches using this library to calibrate the spherical screen: an automatic calibration approach and a semi-automatic approach. Both the automatic and semiautomatic approach require a calibrated camera with know intrinsic matrix and lens distortions. The semiautomatic approach requires

  - Automatic Calibration Approach
    - ​

  - Semi-automatic Calibration Approach
    - ​

  ​

- ​

# Demo

- Camera intrinsic calibration example
  - File: CameraCalibApplication.cpp
  - How-to-use: Press 's' to start, then detection will be automatic (roughly 2 Sec per frame). Once it reaches a minimum amount of frames, the calibration will be triggered, then cleaning will be done to remove frames with large reprojection errors. By the end of calibration, camera intrinsic will be saved (normally with the reprojection error no more than 0.3).
  - Video: https://youtu.be/_vp6XlQdERg
  - Notes: 
    - If strong distortions on lens: make sure the chart reaches image plane boundaries so that barrel curves can be observed.
    - When spatial variance of the board is small, reprojection error would be high.
- Projector intrinsic calibration example
  - File: ProjectorCalibApplication.cpp
  - How-to-use: Right click mouse in the camera window once both the projected pattern and physical pattern are inside the camera view. Once it reaches a minimum amount of frames, the calibration will be triggered, then cleaning will be done to remove frames with large reprojection errors. By the end of calibration, projector intrinsic will be saved (normally with the reprojection error no more than 0.5).
  - Video: https://youtu.be/gG2URmbu0Ik
  - Notes:
    - Only one projector + primary monitor are connected when running. 
    - The board should be rigid and as close to a plane as possible. 
    - When spatial variance of the board is small, reprojection error would be high.
    - Turn off any auto-functionalities on the projector: auto-keystone correction etc.
- Spherical display calibration example
  - File: SphericalDisplayCalibApplication.cpp, main_automatic_calib.m or main_semiauto_calib.m
  - How-to-use: 
    - Run SphericalDisplayCalibApplication.cpp, follow the instruction on the console window: 
      - Determine the projector order (projector #1, #2, etc); 
      - Define the projection circle (by clicking on the circle) on the spherical screen so that all pattern features are within the circle from the camera view; 
      - Blob projection and detection will be triggered for each projector; reprojection error will show up for each projector if it is in the Semi-automatic mode (normally below 2). 
      - Once all projectors have projected, feature data will be saved. If it is in the Semi-automatic mode, the initial guess of projector extrinsics and sphere pose will also be saved. 
    - Run main_automatic_calib.m in Matlab if using automatic calibration mode, or main_semiauto_calib.m if using semiautomatic calibration mode. In the automatic calibration mode, an initial guess of extrinsics and sphere pose will be first generated and non-linear optimization will be applied. In the semiautomatic calibration mode, the initial guess will be read from file and non-linear optimization will be applied on that.
  - Video:
  - Notes:
    - When the baseline between camera and projector is too short, the fundamental matrix might be inaccurate.
    - When observed projection area on Surface is too small (close to a plane), the fundamental matrix might be inaccurate. 
    - Blob detection is lighting-sensitive. May require to adjust the camera parameters and detector parameters based on the lighting. 
    - If the overlapping area across projectors is out-of-focus for projectors, the ghosting effect is likely to happen. 

# Dependency

- OpenCV-2.4.13

- glew-1.12.0

- glfw-3.2

- glm-0.9.4.0

- FlyCapture2-2.9.3.43

# Hardware

- Camera: Flea3 [FL3-U3-13Y3M-C](https://www.ptgrey.com/flea3-13-mp-mono-usb3-vision-vita-1300-camera)
- Projector: Asus [P2B](https://www.google.ca/search?q=Asus+P2B&oq=Asus+P2B&aqs=chrome..69i57j0l5.391j0j7&sourceid=chrome&ie=UTF-8) 



