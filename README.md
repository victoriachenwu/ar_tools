ar_tools
========

AR Marker tools for ROS

Make sure your camera is [calibrated](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). 

###Subscribed topics 

* /camera/camera_info - Calibration info for camera (specific to each physical camera)
* /camera/image_raw


###Running

To launch with live marker tracking:
```roslaunch ar_tools ar_pose_multi.launch```

To launch without live marker tracking (with a recorded camera_info rosbag):
``` roslaunch ar_tools headless_bundle_no_rviz.launch ```
