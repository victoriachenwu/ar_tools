/*
 *  Multi Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2013, I Heart Engineering
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  William Morris <bill@iheartengineering.com>
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://www.iheartengineering.com
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ar_pose/ar_bundle.h"
#include "ar_pose/object.h"
#include "ar_pose/transforms.h"
#include <ros/console.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ar_multi");

  /*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		     ros::console::notifyLoggerLevelsChanged();
  }
  */
  ros::NodeHandle n;
  ar_pose::ARBundlePublisher ar_multi (n);
  ros::spin ();
  return 0;
}

namespace ar_pose
{
  ARBundlePublisher::ARBundlePublisher (ros::NodeHandle & n):n_ (n), it_ (n_)
  {
    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
	std::string default_path = "data/object_4x4";
	std::string default_path_tfs = "data/tfs";
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    // **** get parameters

    if (!n_param.getParam ("publish_tf", publishTf_))
      publishTf_ = true;
    ROS_INFO ("\tPublish transforms: %d", publishTf_);

    if (!n_param.getParam ("publish_visual_markers", publishVisualMarkers_))
      publishVisualMarkers_ = true;
    ROS_INFO ("\tPublish visual markers: %d", publishVisualMarkers_);

    if (!n_param.getParam ("threshold", threshold_))
      threshold_ = 100;
    ROS_INFO ("\tThreshold: %d", threshold_);
	
	//modifications to allow path list from outside the package
	n_param.param ("marker_pattern_list", local_path, default_path);
	if (local_path.compare(0,5,"data/") == 0){
	  //according to previous implementations, check if first 5 chars equal "data/"
	  sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
	}
	else{
	  //for new implementations, can pass a path outside the package_path
	  sprintf (pattern_filename_, "%s", local_path.c_str ());
	}
	ROS_INFO ("Marker Pattern Filename: %s", pattern_filename_);
	

	//grab transform file name
	n_param.param ("marker_transforms_list", local_path, default_path_tfs);
	sprintf (transforms_filename_, "%s", local_path.c_str ());
	ROS_INFO ("Transforms Filename: %s", transforms_filename_);

    // **** subscribe

    ROS_INFO ("Subscribing to info topic");
    sub_ = n_.subscribe (cameraInfoTopic_, 1, &ARBundlePublisher::camInfoCallback, this);
    getCamInfo_ = false;

    // **** advertse 

    arMarkerPub_ = n_.advertise < ar_pose::ARMarkers > ("ar_pose_marker", 0);
    if(publishVisualMarkers_)
    {
      rvizMarkerPub_ = n_.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
    }
  }

  ARBundlePublisher::~ARBundlePublisher (void)
  {
    //cvReleaseImage(&capture_); //Don't know why but crash when release the image
    arVideoCapStop ();
    arVideoClose ();
  }

  void ARBundlePublisher::camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
  {
    if (!getCamInfo_)
    {
      cam_info_ = (*cam_info);

      cam_param_.xsize = cam_info_.width;
      cam_param_.ysize = cam_info_.height;

      cam_param_.mat[0][0] = cam_info_.P[0];
      cam_param_.mat[1][0] = cam_info_.P[4];
      cam_param_.mat[2][0] = cam_info_.P[8];
      cam_param_.mat[0][1] = cam_info_.P[1];
      cam_param_.mat[1][1] = cam_info_.P[5];
      cam_param_.mat[2][1] = cam_info_.P[9];
      cam_param_.mat[0][2] = cam_info_.P[2];
      cam_param_.mat[1][2] = cam_info_.P[6];
      cam_param_.mat[2][2] = cam_info_.P[10];
      cam_param_.mat[0][3] = cam_info_.P[3];
      cam_param_.mat[1][3] = cam_info_.P[7];
      cam_param_.mat[2][3] = cam_info_.P[11];

      cam_param_.dist_factor[0] = cam_info_.K[2];       // x0 = cX from openCV calibration
      cam_param_.dist_factor[1] = cam_info_.K[5];       // y0 = cY from openCV calibration
      if ( cam_info_.distortion_model == "plumb_bob" && cam_info_.D.size() == 5)
        cam_param_.dist_factor[2] = -100*cam_info_.D[0];// f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      else
        cam_param_.dist_factor[2] = 0;                  // We don't know the right value, so ignore it

      cam_param_.dist_factor[3] = 1.0;                  // scale factor, should probably be >1, but who cares...
      
      arInit ();

      ROS_INFO ("Subscribing to image topic");
      cam_sub_ = it_.subscribe (cameraImageTopic_, 1, &ARBundlePublisher::getTransformationCallback, this);
      getCamInfo_ = true;
    }
  }

  void ARBundlePublisher::arInit ()
  {


    ROS_INFO("Starting arInit");
    arInitCparam (&cam_param_);
    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load in the object data - trained markers and associated bitmap files
    if ((object = ar_object::read_ObjData (pattern_filename_, &objectnum)) == NULL)
      ROS_BREAK ();
    ROS_INFO("Objectfile num = %d", objectnum);

	// load in the transform data - transform of marker frame wrt center frame
    if ((tfs= ar_transforms::read_Transforms (transforms_filename_, objectnum)) == NULL)
      ROS_BREAK ();
    ROS_INFO("Read in transforms successfully");
    

    sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
#if ROS_VERSION_MINIMUM(1, 9, 0)
// FIXME: Why is this not in the object
    cv_bridge::CvImagePtr capture_; 
#else
// DEPRECATED: Fuerte support ends when Hydro is released
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
#endif

	
  }

  void ARBundlePublisher::getTransformationCallback (const sensor_msgs::ImageConstPtr & image_msg)
  {
    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int knownPatternCount, k, j;

    /* Get the image from ROSTOPIC
     * NOTE: the dataPtr format is BGR because the ARToolKit library was
     * build with V4L, dataPtr format change according to the 
     * ARToolKit configure option (see config.h).*/
#if ROS_VERSION_MINIMUM(1, 9, 0)
    try
    {
      capture_ = cv_bridge::toCvCopy (image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what()); 
    }
    dataPtr = (ARUint8 *) ((IplImage) capture_->image).imageData;
#else
    try
    {
      capture_ = bridge_.imgMsgToCv (image_msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException & e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what()); 
    }
    dataPtr = (ARUint8 *) capture_->imageData;
#endif

    // detect the markers in the video frame
    if (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0)
    {
      argCleanup ();
      ROS_BREAK ();
    }

    arPoseMarkers_.markers.clear ();
    // check for known patterns
    for (knownPatternCount = 0; knownPatternCount < objectnum; knownPatternCount++)
    {
      k = -1;	//haven't yet seen my pattern yet. 
	  //marker_num is how many markers were actually found
      for (j = 0; j < marker_num; j++)
      {
        if (object[knownPatternCount].id == marker_info[j].id)
        {
          if (k == -1)	//if this is the first wild sighting
            k = j;		//which marker matches my pattern?
          else                  // make sure you have the best pattern (highest confidence factor)
          if (marker_info[k].cf < marker_info[j].cf)
            k = j;
        }
      }//end for (j)
      if (k == -1)	//didn't find my pattern :(
      {
        object[knownPatternCount].visible = 0;
        continue;	//ok. so this just skips all the way to the next knownPatternCount
      }

      object[knownPatternCount].visible = 1;	//woohoo mark as found

	  getMarkerTransform(knownPatternCount, marker_info, k);	//transform is stored in object[knownPatternCount].trans

      double arQuat[4], arPos[3];	//for the marker
      double masterARQuat[4], masterARPos[3];	//for the master/center of the box 


	  //find the transform for the pattern to the center of the box
	  //updates master_trans_
	  findTransformToCenter(object[knownPatternCount].trans, knownPatternCount);
      
	  //arUtilMatInv (object[i].trans, cam_trans);
      arUtilMat2QuatPos (object[knownPatternCount].trans, arQuat, arPos);
      arUtilMat2QuatPos (master_trans_, masterARQuat, masterARPos);

      // **** convert to ROS frame
      double quat[4], pos[3];
      double masterQuat[4], masterPos[3];
	  convertToRosFrame(arQuat, arPos, quat, pos);
	  convertToRosFrame(masterARQuat, masterARPos, masterQuat, masterPos);

	  ROS_DEBUG (" Object num %i------------------------------------------------", knownPatternCount);
      ROS_DEBUG (" QUAT: Pos x: %3.5f  y: %3.5f  z: %3.5f", pos[0], pos[1], pos[2]);
      ROS_DEBUG ("     Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f", quat[0], quat[1], quat[2], quat[3]);

      // **** prepare to publish the marker
	  stuffARMarkerMsg(knownPatternCount, pos, quat,image_msg->header, marker_info);		
 

      // **** publish transform between camera and marker

#if ROS_VERSION_MINIMUM(1, 9, 0)
      tf::Quaternion rotation (quat[0], quat[1], quat[2], quat[3]);
      tf::Vector3 origin (pos[0], pos[1], pos[2]);
      tf::Transform t (rotation, origin);
#else
// DEPRECATED: Fuerte support ends when Hydro is released
      btQuaternion rotation (quat[0], quat[1], quat[2], quat[3]);
      btVector3 origin (pos[0], pos[1], pos[2]);
      btTransform t (rotation, origin);
#endif

	  tf::Quaternion masterRotation (masterQuat[0], masterQuat[1], masterQuat[2], masterQuat[3]);
      tf::Vector3 masterOrigin (masterPos[0], masterPos[1], masterPos[2]);
      tf::Transform masterTransform (masterRotation, masterOrigin);

      if (publishTf_)
      {
        tf::StampedTransform camToMarker (t, image_msg->header.stamp, image_msg->header.frame_id, object[knownPatternCount].name);
        broadcaster_.sendTransform(camToMarker);

		tf::StampedTransform camToMaster (masterTransform, image_msg->header.stamp, image_msg->header.frame_id, "master");
        broadcaster_.sendTransform(camToMaster);

      }

      // **** publish visual marker

      if (publishVisualMarkers_)
      {
#if ROS_VERSION_MINIMUM(1, 9, 0)
        tf::Vector3 markerOrigin (0, 0, 0.25 * object[knownPatternCount].marker_width * AR_TO_ROS);
        tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
        tf::Transform markerPose = t * m; // marker pose in the camera frame 
#else
// DEPRECATED: Fuerte support ends when Hydro is released
        btVector3 markerOrigin (0, 0, 0.25 * object[i].marker_width * AR_TO_ROS);
        btTransform m (btQuaternion::getIdentity (), markerOrigin);
        btTransform markerPose = t * m; // marker pose in the camera frame
#endif

		publishVisualMarker(knownPatternCount, markerPose, image_msg->header); 
	  }
  
	} //end outer loop of for 
    arMarkerPub_.publish(arPoseMarkers_);
  
  }

  void ARBundlePublisher::getMarkerTransform(int knownPatternCount, ARMarkerInfo *marker_info, int seenPatternCount)	{
  
      // calculate the transform for each marker
      if (object[knownPatternCount].visible == 0)	//if the marker was not found the previous time
      {
        arGetTransMat (&marker_info[seenPatternCount], object[knownPatternCount].marker_center, object[knownPatternCount].marker_width, object[knownPatternCount].trans);
      }
      else	//if the marker was found the previous time, use the transform with history
      {
        arGetTransMatCont (&marker_info[seenPatternCount], object[knownPatternCount].trans,
                           object[knownPatternCount].marker_center, object[knownPatternCount].marker_width, object[knownPatternCount].trans);
      }


  }
  void ARBundlePublisher::findTransformToCenter(double camera_to_marker_trans[3][4], int knownPatternCount)	{
	arUtilMatMul(camera_to_marker_trans, tfs[knownPatternCount], master_trans_);
  }
  void ARBundlePublisher::stuffARMarkerMsg(int knownPatternCount,  
  	double pos[3], double quat[4],std_msgs::Header image_header, ARMarkerInfo *marker_info)		{
  
      ar_pose::ARMarker ar_pose_marker;
      ar_pose_marker.header.frame_id = image_header.frame_id;
      ar_pose_marker.header.stamp = image_header.stamp;
      ar_pose_marker.id = object[knownPatternCount].id;

      ar_pose_marker.pose.pose.position.x = pos[0];
      ar_pose_marker.pose.pose.position.y = pos[1];
      ar_pose_marker.pose.pose.position.z = pos[2];

      ar_pose_marker.pose.pose.orientation.x = quat[0];
      ar_pose_marker.pose.pose.orientation.y = quat[1];
      ar_pose_marker.pose.pose.orientation.z = quat[2];
      ar_pose_marker.pose.pose.orientation.w = quat[3];

      ar_pose_marker.confidence = round(marker_info->cf * 100);
      arPoseMarkers_.markers.push_back (ar_pose_marker);


  }

 
  void ARBundlePublisher::convertToRosFrame(double arQuat[4], double arPos[3], double quat[4], double pos[3])	{
	  pos[0] = arPos[0] * AR_TO_ROS;
      pos[1] = arPos[1] * AR_TO_ROS;
      pos[2] = arPos[2] * AR_TO_ROS;

      quat[0] = -arQuat[0];
      quat[1] = -arQuat[1];
      quat[2] = -arQuat[2];
      quat[3] = arQuat[3];
  }

  void ARBundlePublisher::publishVisualMarker(int knownPatternCount, tf::Transform markerPose, std_msgs::Header image_header)	{
 
        tf::poseTFToMsg (markerPose, rvizMarker_.pose);

        rvizMarker_.header.frame_id = image_header.frame_id;
        rvizMarker_.header.stamp = image_header.stamp;
        rvizMarker_.id = object[knownPatternCount].id;

        rvizMarker_.scale.x = 1.0 * object[knownPatternCount].marker_width * AR_TO_ROS;
        rvizMarker_.scale.y = 1.0 * object[knownPatternCount].marker_width * AR_TO_ROS;
        rvizMarker_.scale.z = 0.5 * object[knownPatternCount].marker_width * AR_TO_ROS;
        rvizMarker_.ns = "basic_shapes";
        rvizMarker_.type = visualization_msgs::Marker::CUBE;
        rvizMarker_.action = visualization_msgs::Marker::ADD;
        switch (knownPatternCount)
        {
          case 0:
            rvizMarker_.color.r = 0.0f;
            rvizMarker_.color.g = 0.0f;
            rvizMarker_.color.b = 1.0f;
            rvizMarker_.color.a = 1.0;
            break;
          case 1:
            rvizMarker_.color.r = 1.0f;
            rvizMarker_.color.g = 0.0f;
            rvizMarker_.color.b = 0.0f;
            rvizMarker_.color.a = 1.0;
            break;
          default:
            rvizMarker_.color.r = 0.0f;
            rvizMarker_.color.g = 1.0f;
            rvizMarker_.color.b = 0.0f;
            rvizMarker_.color.a = 1.0;
        }
        rvizMarker_.lifetime = ros::Duration (1.0);

        rvizMarkerPub_.publish(rvizMarker_);
        ROS_DEBUG ("Published visual marker");
      } //fi publishVisualMarkers
  
} // end namespace ar_pose
