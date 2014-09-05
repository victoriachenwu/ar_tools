ar_tools
========

This is a fork of an existing package, ar_tools, which is a ROS (Robot Operating System) wrapper for ARToolkit. 
This modified package enables tracking of multiple markers, as if they were a single point. 

[Here] (http://vptarmigan.wordpress.com/2014/06/23/setting-up-ar_tools-ros-package/) is a short tutorial on running ROS and installing this package.

More information on this project is available [here] (https://sites.google.com/site/umdminirobotreu2014wu/).


###Subscribed topics 

* /camera/camera_info - Calibration info for camera (specific to each physical camera)
* /camera/image_raw

###Input Files



###Running

Make sure your camera is [calibrated](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). 

To launch with live marker tracking:
```
roslaunch ar_tools ar_pose_multi.launch
```

To launch without live marker tracking (with a recorded camera_info rosbag):
```
roslaunch ar_tools headless_bundle_no_rviz.launch
```
