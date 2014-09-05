ar_tools
========

This is a fork of an existing package, ar_tools, which is a ROS (Robot Operating System) wrapper for ARToolkit. 
This modified package enables tracking of multiple markers, as if they were a single point. 

[Here] (http://vptarmigan.wordpress.com/2014/06/23/setting-up-ar_tools-ros-package/) is a short tutorial on running ROS and installing this package.

Check the [project website] (https://sites.google.com/site/umdminirobotreu2014wu/) for more info.


###Subscribed topics 

* `/camera/camera_info` - Calibration info for camera (specific to each physical camera)
* `/camera/image_raw`

###Input Files
Two files are required - one describing pattern info, one describing pattern transforms. 
Modify the parameter values below in the launch file to point to the locations of these files. 

  * Parameter name: `marker_pattern_list` 
    * Specifies how many markers, and info for each marker.
    * [Example file] (https://github.com/vptarmigan/ar_tools/blob/master/ar_pose/config/multiple)
  * Parameter name:`marker_transforms_list`
    * Specifies, for each marker, the transformation matrix relating each marker frame to a common frame. 
    *  [Example file] (https://github.com/vptarmigan/ar_tools/blob/master/ar_pose/config/multiple_transforms)


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


###Future Work/Improvements
* This package uses `ar_pose` as its base. If more optimization is needed, it is recommended to take a look at, `ar_track_alvar`, that wraps a modified and optimized version of ARToolkit.    
* ARToolkit provides a built in [function] (http://www.hitl.washington.edu/artoolkit/documentation/tutorialmulti.htm), `arMultiGetTransMat()` that essentially tracks multiple markers as one target. That function encapsulates the algorithm in this package's `ar_bundle.cpp`, but is not used in this package due to time constraints + difficulty testing ARToolkit's function. Future work could be done in incorporating this function instead. 
