//C++
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>

#include <time.h>
 

//ROS 
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>


//ROS(自作)
#include <roch_tts/RochTTSAction.h>
#include <roch_gpsr/SoundEffectAction.h>
//#include <roch_gpsr/CamLiftAction.h>
//#include <roch_gpsr/ObjRecAction.h>


//PCL(基本)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//PCL(下采样)
#include <pcl/filters/voxel_grid.h>

//PCL(直通滤波)
#include <pcl/filters/passthrough.h>

//PCL(平面提取)
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

//PCL(平面去除)
#include <pcl/filters/extract_indices.h>

//PCL(分割)
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
