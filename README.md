# Calibration

### Ressources
- [PR2 Full System Calibration](http://wiki.ros.org/pr2_calibration/Tutorials/Calibrating%20the%20PR2)
- [Calibrating the PR2's Cameras](http://wiki.ros.org/pr2_calibration/Tutorials/Calibrating%20the%20PR2's%20Cameras)
- [How to Calibrate a Monocular Camera](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
- [How to Calibrate a Stereo Camera](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)

https://answers.ros.org/question/228693/image_view-symbol-lookup-error/

image_pipeline  nodelet_core  pluginlib  pr2_apps  pr2_calibration  vision_opencv

### Wide Stereo
```
rosrun camera_calibration cameracheck.py stereo:=wide_stereo --size=7x6 --square=0.108
roslaunch pr2_calibration_launch calibrate_wide_stereo.launch
```
### Narrow Stereo
```
rosrun camera_calibration cameracheck.py stereo:=narrow_stereo --size=7x6 --square=0.108
roslaunch pr2_calibration_launch calibrate_narrow_stereo.launch
```
### Forearm Cameras
Probably it is K[0] to check focal length.
```
rosrun camera_calibration cameracheck.py monocular:=l_forearm_cam --size=7x6 --square=0.108
rostopic echo l_forearm_cam/camera_info/P[0]
roslaunch pr2_calibration_launch calibrate_forearm_left.launch

rosrun camera_calibration cameracheck.py monocular:=r_forearm_cam --size=7x6 --square=0.108
rostopic echo r_forearm_cam/camera_info/P[0]
roslaunch pr2_calibration_launch calibrate_forearm_right.launch
```

### Asus
- http://wiki.ros.org/topic_tools
```
rosrun topic_tools relay /head_mount_asus/rgb/image_rect_mono /head_mount_asus/rgb/image_rect
rosrun camera_calibration cameracheck.py monocular:=head_mount_asus/rgb --size=7x6 --square=0.108

rosrun topic_tools relay /head_mount_asus/ir/image /head_mount_asus/ir/image_rect
rosrun camera_calibration cameracheck.py monocular:=head_mount_asus/ir --size=7x6 --square=0.108
```


### Camera Calibration with [Kalibr](https://github.com/ethz-asl/kalibr/wiki)

#### Multiple Camera Calibration
Follow this tutorial [Multiple Camera Calibration](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration).

This repository provides the ImageViewer which allows you to collect images of multiple cameras. It will open a window for each given topic showing it's corresponding images. With key "s" you can save an image for each topic. The images are organized in one folder per topic. To create a ROS bag you can use the [bagcreater](https://github.com/ethz-asl/kalibr/wiki/bag-format) script.
````
roslaunch calibration image_viewer.launch topics:="[<topic1>, <topic2>, ...]"
````

````
roslaunch pr2_camera_calibration throttle.launch
rosbag record /wide_stereo/right/image_color_throttle /head_mount_asus/rgb/image_raw_throttle -O calibration.bag


target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.088           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
                         #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]


rosrun kalibr kalibr_calibrate_cameras --bag calibration.bag --topics /wide_stereo/right/image_color_throttle /head_mount_asus/rgb/image_raw_throttle --models omni-radtan pinhole-radtan --target ../config/aprilgrid.yaml
````

rostopic echo /wide_stereo/right/image_color/header/frame_id

rosrun tf tf_echo /head_plate_frame /wide_stereo_optical_frame
rosrun tf tf_echo /head_plate_frame /wide_stereo_gazebo_r_stereo_camera_optical_frame

- Translation: [0.047, 0.030, 0.060]
- Rotation: in Quaternion [-0.499, 0.509, -0.501, 0.491]
            in RPY (radian) [-1.587, -0.000, -1.591]
            in RPY (degree) [-90.912, -0.016, -91.131]

#### 3. Output
````
cam0:
  cam_overlaps: [1]
  camera_model: omni
  distortion_coeffs: [-0.08584868092448515, -0.33989363544848455, 0.008034409013963411,
    -9.547618235185697e-05]
  distortion_model: radtan
  intrinsics: [2.30221231427014, 1426.7679725413386, 1426.2456934693698, 312.6729274795651,
    248.63006936854748]
  resolution: [640, 480]
  rostopic: /wide_stereo/right/image_color_throttle
cam1:
  T_cn_cnm1:
  - [0.9997909506394976, 0.012894403439807033, -0.015867872551119, 0.08206561189636781]
  - [-0.01271692208844398, 0.9998560115701596, 0.011235480390570863, 0.08454121462996234]
  - [0.016010462578061536, -0.011031341121638352, 0.9998109694342686, 0.015899018156253115]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: [0]
  camera_model: pinhole
  distortion_coeffs: [0.06108229566360465, -0.1628852517939544, 0.00045416785168086527,
    0.0019971539226812276]
  distortion_model: radtan
  intrinsics: [539.9521302196091, 539.959741284173, 323.7968033915384, 239.42314993621315]
  resolution: [640, 480]
  rostopic: /head_mount_asus/rgb/image_raw_throttle


````
