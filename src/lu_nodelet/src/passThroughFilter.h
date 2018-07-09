//C
#include <pluginlib/class_list_macros.h>
#include "nodelet/nodelet.h"
#include <string>
#include <iostream>
#include "printUtil.h"
//ROS:
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <lidar_utility_msgs/lidarUtilitySettings.h>
#include <lidar_utility_msgs/roadInfo.h>
#include <lidar_utility_msgs/objectInfo.h>
#include "std_msgs/String.h"
#include <lidar_utility_msgs/roadInfo.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
//decleare globals
static int mode =1;
static std::string nodeName("pass_through_filter");
static 	float xMinf, xMaxf, yMinf, yMaxf, zMinf, zMaxf;
static float roadMin_setting=-4;
static float roadMax_setting=-1.2;
static float objectMin_setting=-1.7;
static float objectMax_setting=1.25;
static float boxMargin_setting=.2;
static float yRangeBoost_setting=0;
//nodelet
#include <nodelet/nodelet.h>
namespace lu_nodelet
{

    class passThroughFilter : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };

}
