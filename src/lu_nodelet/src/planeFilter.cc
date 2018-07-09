/* planeFilter
 * Nikolas Gamarra -+- nxgamarra@gmail.com
 * Description: Performs planar segmentation on input and republishes. 
 * Also publishes min/max custom msg about road bounds
 * Available modes: segmentation, projection
 */

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
#include <lidar_utility_msgs/roadInfo.h>
#include <lidar_utility_msgs/objectInfo.h>
#include "std_msgs/String.h"
#include <lidar_utility_msgs/roadInfo.h>
//PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
//PCL local includes
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/search/search.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
//decleare globals
static int mode =1;
static std::string nodeName("planeFilter");
static float setDistanceThreshold_setting=.2;
static float setLeafSize_setting=.01;
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
//#include "planeFilter.h" //structured without headerfile
namespace lu_nodelet
{
	class planeFilter : public nodelet::Nodelet
	{
		public:
			void onInit()
			{
				ROS_INFO("Initializing nodelet...");

				ros::NodeHandle private_nh;//create node handles
				//ros::NodeHandle nh;
				nh = getNodeHandle();
				private_nh = getPrivateNodeHandle();

				//update settings
				nh.getParam("settings/plane_DistThresh", setDistanceThreshold_setting);
				nh.getParam("settings/plane_LeafSize", setLeafSize_setting);

				//initialize default topics for subscribing and publishing
				const std::string defaultCloudSubscriber("cloud_in");
				const std::string defaultCloudPublisher("cloud_out");
				const std::string defaultMode("r");
				//const std::string defaultMsgSubscriber("plane_segmented_msg");

				nodeName = getName();//Update nodelet name

				//set parameters on new name
				const std::string subscriberParamName(nodeName + "/subscriber");
				const std::string publisherParamName(nodeName + "/publisher");
				const std::string modeParamName(nodeName + "/mode");
				//const std::string subscriberParamName2(nodeName + "/msgSubscriber");

				printf(COLOR_BLUE BAR COLOR_RST);//Print a blue bar
				ROS_INFO("Node Name: %s",nodeName.c_str());

				//Create variables that control the topic names
				std::string sTopic;
				std::string pTopic;
				std::string myMode;
				std::string sTopic2;



				if(nh.hasParam(subscriberParamName)){//Check if the user specified a subscription topic
					nh.getParam(subscriberParamName,sTopic);
					printf(COLOR_GREEN BAR COLOR_RST);
					ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
				}else{
					sTopic=defaultCloudSubscriber;//set to default if not specified
					printf(COLOR_RED BAR COLOR_RST);
					ROS_INFO("%s: No param set **%s**  \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
				}
				/*
				   if(nh.hasParam(subscriberParamName2)){//Check if the user specified a subscription topic for msgs
				   nh.getParam(subscriberParamName2,sTopic2);
				   printf(COLOR_GREEN BAR COLOR_RST);
				   ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
				   }else{
				   sTopic2=defaultMsgSubscriber;//set to default if not specified
				   printf(COLOR_RED BAR COLOR_RST);
				   ROS_INFO("%s: No param set **%s**  \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
				   }
				 */
				if(nh.hasParam(publisherParamName)){//Check if the user specified a publishing topic
					printf(COLOR_GREEN BAR COLOR_RST);
					nh.getParam(publisherParamName,pTopic);
					ROS_INFO("%s: A param has been set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
				}else{printf(COLOR_RED BAR COLOR_RST);
					pTopic=defaultCloudPublisher;//set to default if not specified
					ROS_INFO("%s: No param set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
				}

				if(nh.hasParam(modeParamName)){	//Check if the user specified a mode
					nh.getParam(modeParamName,myMode);
					printf(COLOR_GREEN BAR COLOR_RST);
					ROS_INFO("%s: A param has been set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
				}else{
					myMode=defaultMode;//set to default if not specified
					printf(COLOR_RED BAR COLOR_RST);
					ROS_INFO("%s: No param set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
				}

				//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
				nh.deleteParam(subscriberParamName);
				nh.deleteParam(publisherParamName);
				nh.deleteParam(modeParamName);
				//nh.deleteParam(subscriberParamName2);
				if(myMode=="1"||myMode=="s"||myMode=="segmentation"){//interpret mode
					mode=1;
				}else if(myMode=="2"||myMode=="p"||myMode=="projection"){
					mode=2;
				}else if(myMode=="3"||myMode=="x"||myMode=="x"){
					mode=3;
				}else if(myMode=="4"||myMode=="x"||myMode=="x"){
					mode=4;
				}

				//set up subscribers and publishers
				pc2_sub = nh.subscribe(sTopic, 10,&planeFilter::cloud_cb, this,ros::TransportHints().tcpNoDelay(true));//subscribe to point cloud

				//msg_sub = nh.subscribe(sTopic2, 10,&planeFilter::message_cb, this,ros::TransportHints().tcpNoDelay(true));//subscribe to msgs

				pc2_pub = private_nh.advertise<sensor_msgs::PointCloud2>(pTopic+"_pts", 1);
				msg_pub= private_nh.advertise<lidar_utility_msgs::roadInfo>(pTopic+"_msg",1);

			}
			void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){//callback to process cloud
				//Callback for filtering and republishing recived data
				ROS_INFO("%s: In Callback",nodeName.c_str());

				pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;// Create a container for the data and filtered data.
				pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

				pcl_conversions::toPCL(*cloud_msg, *cloud);	//Convert to PCL data type

				//create PCLXYZ for

				pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ for input to be converted to 
				pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
				pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
				pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ


				pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);

				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
				//Filter

				// Create the filtering object: downsample the dataset using a leaf size of 1cm
				pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
				sor.setInputCloud (cloudPtr);
				sor.setLeafSize (0.01f, 0.01f, 0.01f);//.01 defaultSETTING
				sor.filter (*cloud_filtered_blob);

				// Convert to the templated PointCloud
				pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);//PCLPC2toPCLXYZ

				if(mode==1){//segmentation

					//std::cerr << "Point cloud data: " << cloud->points.size ()<<std::endl;

					pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
					pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
					// Create the segmentation object
					pcl::SACSegmentation<pcl::PointXYZ> seg;
					// Optional
					seg.setOptimizeCoefficients (true);
					// Mandatory
					seg.setModelType (pcl::SACMODEL_PLANE);
					seg.setMethodType (pcl::SAC_RANSAC);
					seg.setDistanceThreshold (setDistanceThreshold_setting);//SETTING

					seg.setInputCloud (cloud_filtered);
					seg.segment (*inliers, *coefficients);

					if (inliers->indices.size () == 0)
					{	printf(COLOR_RED BAR COLOR_RST);
						ROS_INFO("ERROR: Could not estimate a planar model for the given dataset.");

					}
					std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
						<< coefficients->values[1] << " "
						<< coefficients->values[2] << " " 
						<< coefficients->values[3] << std::endl;

					pcl::ExtractIndices<pcl::PointXYZ> extract;
					// Extract the inliers
					extract.setInputCloud (cloud_filtered);
					extract.setIndices (inliers);
					extract.setNegative (false);
					extract.filter (*cloud_p);

				}else if (mode==2){//planar projection

					// Create a set of planar coefficients with X=Y=0,Z=1
					pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
					coefficients->values.resize (4);
					coefficients->values[0] = coefficients->values[1] = 0;
					coefficients->values[2] = 1.0;
					coefficients->values[3] = 0;

					// Create the filtering object
					pcl::ProjectInliers<pcl::PointXYZ> proj;
					proj.setModelType (pcl::SACMODEL_PLANE);
					proj.setInputCloud (cloud_filtered);
					proj.setModelCoefficients (coefficients);
					proj.filter (*cloud_p);

				}else if (mode==3){

				}//mode

				//float xMax,xMin,yMax,yMin,zMax,zMin;
				//Eigen::Vector4f min_pt, max_pt;

				//pcl::PointT pMin;
				//pcl::PointT pMax;

				//pcl::PointXYZRGB pMin;
				//pcl::PointXYZRGB pMax;

				pcl::PointXYZ pMin,pMax;
				pcl::getMinMax3D (*cloud_p,pMin,pMax);

				//custom msg

				lidar_utility_msgs::roadInfo msg;

				msg.headerstamp = ros::Time::now();
				msg.header.frame_id = "/world";
				msg.xMax=pMax.x;
				msg.xMin=pMin.x;
				msg.yMax=pMax.y;
				msg.yMin=pMin.y;
				msg.zMax=pMax.z;
				msg.zMin=pMin.z;

				msg_pub.publish(msg);

				sensor_msgs::PointCloud2 output;//create output container
				pcl::PCLPointCloud2 temp_output;//create PCLPC2
				pcl::toPCLPointCloud2(*cloud_p,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
				pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
				pc2_pub.publish (output);// Publish the data.
			}//cloud callback
			//create subscribers and publishers
			ros::NodeHandle nh;
			ros::Subscriber pc2_sub;
			//ros::Subscriber msg_sub;
			ros::Publisher pc2_pub;
			ros::Publisher msg_pub;

	};//class
	PLUGINLIB_EXPORT_CLASS(lu_nodelet::planeFilter, nodelet::Nodelet)
}//ns
