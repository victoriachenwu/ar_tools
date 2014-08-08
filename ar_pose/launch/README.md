Example Launch Files
====================
These examples require the uvc_camera driver
http://www.ros.org/wiki/uvc_camera

ar_pose_single.launch
  The TFs are setup for fixed camera and a moving marker

ar_pose_reverse.launch
  Launch file for a fixed marker and a moving camera

ar_pose_multi.launch
  Compute the TFs from the camera to multiple AR Markers

(Added)

ar_pose_bundle.launch
  Compute the TF from camera to a master coordinate system, given multiple AR markers

headless_bundle_no_rviz.launch
   Same as ar_pose_bundle, but does not open camera feed, does not open rviz. 

headless_bundle_with_rviz.launch
  Same as ar pose bundle, does not open camera feed, opens rviz.
