Using USB 3.0
The technology is entirely passive (no IR lighting or Time of flight)

##### ZED SDK - ROS Integration

**Running the ZED ROS Node**

Don't close until you're done with your ZED!

```
roslaunch zed_wrapper zed2.launch
```

To launch a zed2 with the rviz displaying the camera information

```
roslaunch zed_display_rviz display_zed2.launch
```

**Onboard Image Signal Processing vs. Host Computer**

See below for more information about the ISP. The Host Computer is the computer that the ZED is directly attached to via USB, and much of the impressive features of ZED require the host to calculate.  

 > The ZED camera **features an onboard ISP (Image Signal Processor) that performs various image processing algorithms on raw images** captured by dual image sensors. Several parameters of the ISP can be adjusted directly ... through the ZED SDK

Source - SDK Overview > Camera Controls > Adjusting Camera Settings

|SETTINGS|DESCRIPTION|VALUES|
|---|---|---|
|Resolution|Controls camera resolution.|HD2K, HD1080,  <br>HD720, VGA|
|FPS|Controls frame rate.|15, 30, 60, 100|
|Brightness|Controls image brightness.|[0 - 8]|
|Contrast|Controls image contrast.|[0 - 8]|
|Hue|Controls image color.|[0 - 11]|
|Saturation|Controls image color intensity.|[0 - 8]|
|Gamma|Controls gamma correction.|[0 - 8]|
|Sharpness|Controls image sharpness.|[0 - 8]|
|White Balance|Controls camera white balance.|[2800 - 6500]|
|Exposure|Controls shutter speed.  <br>Setting a long exposure time leads to  <br>an increase in motion blur.|[0 - 100]  <br>(% of camera frame rate)|
|Gain|Controls digital amplification of the  <br>signal from the camera sensors.|[0 - 100]|

These can be adjusted or configured on initialization. 

When it comes to information 

 > **The depth map is generated by the host computer the camera is attached to.** It is generated for each frame of the camera...
 
Source - Stereolabs, Medium. [Spatial Mapping in Computer vision using ZED.][https://stereolabs.medium.com/spatial-mapping-in-computer-vision-using-zed-69bce43c2e7a]

After process done with ROS middleware with the host computer, the below topics can be subscribed to with another ROS node to receive and record this data. 

**Information Published from the Node**

[Topics][https://www.stereolabs.com/docs/ros/zed-node/#published-topics]

*Images*
- Left camera
    - `rgb/image_rect_color`: Color rectified image (left RGB image by default)
    - `rgb/image_rect_gray`: Grayscale rectified image (left RGB image by default)
    - `rgb_raw/image_raw_color`: Color unrectified image (left RGB image by default)
    - `rgb_raw/image_raw_gray`: Grayscale unrectified image (left RGB image by default)
    - `rgb/camera_info`: Color camera calibration data
    - `rgb_raw/camera_info`: Color unrectified camera calibration data
    - `left/image_rect_color`: Left camera color rectified image
    - `left/image_rect_gray`: Left camera grayscale rectified image
    - `left_raw/image_raw_color`: Left camera color unrectified image
    - `left_raw/image_raw_gray`: Left camera grayscale unrectified image
    - `left/camera_info`: Left camera calibration data
    - `left_raw/camera_info`: Left unrectified camera calibration data
    - 
- Right camera
    - `right/image_rect_color`: Color rectified right image
    - `right_raw/image_raw_color`: Color unrectified right image
    - `right/image_rect_gray`: Grayscale rectified right image
    - `right_raw/image_raw_gray`: Grayscale unrectified right image
    - `right/camera_info`: Right camera calibration data
    - `right_raw/camera_info`: Right unrectified camera calibration data
    - 
- Stereo pair
    - `stereo/image_rect_color`: stereo rectified pair images side-by-side
    - `stereo_raw/image_raw_color`: stereo unrectified pair images side-by-side

*Depth and Point Cloud*
- `depth/depth_registered`: Depth map image registered on the left image (32-bit float in meters by default)
- `depth/camera_info`: Depth camera calibration data
- `point_cloud/cloud_registered`: Registered color point cloud
- `confidence/confidence_map`: Confidence image (floating point values to be used in your own algorithms)
- `disparity/disparity_image`: Disparity image

*Sensor Data*
- `imu/data`: Accelerometer, gyroscope, and orientation data in Earth frame
- `imu/data_raw`: Accelerometer and gyroscope data in Earth frame
- `imu/mag`: Calibrated magnetometer data 
- `atm_press`: Atmospheric pressure data 
- `temperature/imu`: Temperature of the IMU sensor 
- `temperature/left`: Temperature of the left camera sensor
- `temperature/right`: Temperature of the right camera sensor
- `left_cam_imu_transform`: Transform from the left camera sensor to IMU sensor position

*Diagnostics*
 > `/diagnostics`: ROS diagnostic message for ZED cameras

[Services][https://www.stereolabs.com/docs/ros/zed-node/#services]

Frames of References - different coordinate frames for ZED (position, orientation). Can be changed in the launch file. 

TODO
Information transfer specs? Ways of measuring? 

[ROS Integration > ZED Node][https://www.stereolabs.com/docs/ros/zed-node/]

**ZED Open Capture API and Low-Level Camera and Sensor Capture**

An open-source C++ library for low-level capture of camera and sensor data (raw video frames, calibration data, camera controls, and raw data from camera sensors) is made available by stereolabs, and can be run on a host computer without CUDA. 

Synchronization mechanism is available to match data with a given video frame. The ZED SDK output data is calibrated and compensated, while here extracted raw data is not corrected by camera and sensor calibration parameters. 

Prereqs: Linux OS, GCC (v7.5+), CMake (v3.1+). Optional for examples: OpenCV (v3.4.0+)

Get video and sensor data 

```C++
#include "videocapture.hpp"
sl_oc::video::VideoCapture cap;
cap.initializeVideo();
const sl_oc::video::Frame frame = cap.getLastFrame();

#include "sensorcapture.hpp"
sl_oc::sensors::SensorCapture sens;
std::vector<int> devs = sens.getDeviceList();
sens.initializeSensors( devs[0] );
const sl_oc::sensors::data::Imu imuData = sens.getLastIMUData(5000);
const sl_oc::sensors::data::Magnetometer magData = sens.getLastMagnetometerData(100);
const sl_oc::sensors::data::Environment envData = sens.getLastEnvironmentData(100);
const sl_oc::sensors::data::Temperature tempData = sens.getLastCameraTemperatureData(100);
```

[Open Capture API GitHub Repo][https://github.com/stereolabs/zed-open-capture]
[Open Capture API Docs][https://stereolabs.github.io/zed-open-capture/]

##### Calibration

**Camera Calibration Parameters through API**

Manual calibration is not recommended for ZED2 cameras as output could be degraded from factory calibration. (If you are using for in in-air application! Underwater - it's pretty essential)

Using the API you can retrieve many of the camera calibration parameters. 
```python
calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters

# Focal length of the left eye in pixels
focal_left_x = calibration_params.left_cam.fx

# First radial distortion coefficient
k1 = calibration_params.left_cam.disto[0]

# Translation between left and right eye on z-axis
tz = calibration_params.T.z

# Horizontal field of view of the left eye in degrees
h_fov = calibration_params.left_cam.h_fov
```
Focal length - fx, fy
Principal points - cx, cy
Lens distortion - k1, k2, k3, p1, p2
Horizontal, vertical, and diagonal FOV
Stereo calibration: rotation and translation between left and right eye

[Source - SDK Overview > Camera Calibration]

**Underwater Calibration** 

 > The ZED cameras are air calibrated, and as water acts as a lens, you will need to re-calibrate your camera in order to use it underwater. 
 > 
 > You can use the ZED Calibration tool if you have a setup that allows you to use a display underwater [no, I don't think we do], otherwise, you must use an external library like OpenCV and a water-resistant chessboard. 
 > Note that the ZED cameras are not water-resistant...

[Source - Stereolabs Support, Can I use the ZED camera underwater?][https://support.stereolabs.com/hc/en-us/articles/4402812389399-Can-I-use-the-ZED-camera-underwater-]

---

##### Links and References

[SDK Overview > Camera Calibration][https://www.stereolabs.com/docs/video/camera-calibration/]
[Integrations > ROS > Getting Started][https://www.stereolabs.com/docs/ros/]
[Integrations > ROS > ZED Node][https://www.stereolabs.com/docs/ros/zed-node/]

[Stereolabs Support, Using ZED Underwater][https://support.stereolabs.com/hc/en-us/articles/4402812389399-Can-I-use-the-ZED-camera-underwater-]
[Stereolabs Medium, Spatial Mapping in Computer vision using ZED.][https://stereolabs.medium.com/spatial-mapping-in-computer-vision-using-zed-69bce43c2e7a]

[Open Capture API GitHub Repo][https://github.com/stereolabs/zed-open-capture]
[Open Capture API Docs][https://stereolabs.github.io/zed-open-capture/]