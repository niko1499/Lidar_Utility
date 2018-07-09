/* outlierRemovalFilter
 * Nikolas Gamarra -+- nxgamarra@gmail.com
 * Description: Filter and republish a cloud with its outliers removed
 * statistical, radial, conditional outlier removal available
 * Available modes: statistical, radial, conditional
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
//PCl local includes
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
//decleare globals
static int mode =1;
static std::string nodeName("outlierRemovalFilter");
static float setMeanK_setting=75;
static float setStddevMulThresh_setting=.9;
static float setRadiusSearch_setting=.8;
static float setMinNeighborsInRadius_setting=2;
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace lu_nodelet
{
	class outlierRemovalFilter : public nodelet::Nodelet
	{
		public:
			void onInit()
			{
				ROS_INFO("Initializing nodelet...");

				ros::NodeHandle private_nh;//create node handles
				ros::NodeHandle nh;
				nh = getNodeHandle();
				private_nh = getPrivateNodeHandle();

				//update settings
				nh.getParam("settings/outlierRemoval_MeanK", setMeanK_setting);
				nh.getParam("settings/outlierRemoval_StdDev", setStddevMulThresh_setting);
				nh.getParam("settings/outlierRemoval_SearchRadius", setRadiusSearch_setting);
				nh.getParam("settings/outlierRemoval_MinNeighborsInRadius", setMinNeighborsInRadius_setting);

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
				//std::string sTopic2;

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
				if(myMode=="1"||myMode=="s"||myMode=="statistical"){//interpret mode
					mode=1;
				}else if(myMode=="2"||myMode=="r"||myMode=="radial"){
					mode=2;
				}else if(myMode=="3"||myMode=="c"||myMode=="conditional"){
					mode=3;
				}else if(myMode=="x"||myMode=="x"||myMode=="x"){
					mode=4;
				}

				//set up subscribers and publishers
				pc2_sub = nh.subscribe(sTopic, 10,&outlierRemovalFilter::cloud_cb, this,ros::TransportHints().tcpNoDelay(true));//subscribe to point cloud

				//msg_sub = nh.subscribe(sTopic2, 10,&passThroughFilter::message_cb, this,ros::TransportHints().tcpNoDelay(true));//subscribe to msgs

				pc2_pub = private_nh.advertise<sensor_msgs::PointCloud2>(pTopic, 10);

			}



			void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){//callback to process cloud
				ROS_INFO("%s: In Callback",nodeName.c_str());

		//Callback for filtering and republishing recived data
	//Comment out as needed. Useful for debuging
	ROS_INFO("%s: In Callback",nodeName.c_str());

	//OUTLIER REMOVAL
	if(mode==1){//Statistical

	// Create a container for the data and filtered data.
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);


	// Do data processing here...

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	pcl::PCLPointCloud2 cloud_filtered;
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (cloudPtr);
		sor.setMeanK (setMeanK_setting);//SETTING
		sor.setStddevMulThresh (setStddevMulThresh_setting);//SETTING
		sor.filter (cloud_filtered);
//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered,output);
	pcl_conversions::fromPCL(cloud_filtered,output);
	// Publish the data.
	pc2_pub.publish (output);
	}else if (mode==2){//radial
		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2

		pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> indicies;
		pcl::fromPCLPointCloud2(pcl_pc2, *in_cloud);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);


		cloud->is_dense = false;
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	    // build the filter
	    outrem.setInputCloud(cloud);
	    outrem.setRadiusSearch(setRadiusSearch_setting);
	    outrem.setMinNeighborsInRadius (setMinNeighborsInRadius_setting);
	    // apply filter
	    outrem.filter (*cloud_filtered);
sensor_msgs::PointCloud2 output;//create output container
	pcl::PCLPointCloud2 temp_output;//create PCLPC2
	pcl::toPCLPointCloud2(*cloud_filtered,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
	pc2_pub.publish (output);// Publish the data.

	}else if (mode==3){//conditional
/*
		// build the condition
		pcl::ConditionAnd<pcl::PCLPointCloud2>::Ptr range_cond (new
		pcl::ConditionAnd<pcl::PCLPointCloud2> ());
		range_cond->addComparison (pcl::FieldComparison<pcl::PCLPointCloud2>::ConstPtr (new
		pcl::FieldComparison<pcl::PCLPointCloud2> ("z", pcl::ComparisonOps::GT, 0.0)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PCLPointCloud2>::ConstPtr (new
		pcl::FieldComparison<pcl::PCLPointCloud2> ("z", pcl::ComparisonOps::LT, 0.8)));
		// build the filter
		pcl::ConditionalRemoval<pcl::PCLPointCloud2> condrem;
		condrem.setCondition (range_cond);
		condrem.setInputCloud (cloudPtr);
		condrem.setKeepOrganized(true);
		// apply filter
		condrem.filter (cloud_filtered);
			    */
	}//mode
			}//cloud callback
			//create subscribers and publishers
			ros::NodeHandle nh;
			ros::Subscriber pc2_sub;
			ros::Publisher pc2_pub;
			//ros::Subscriber msg_sub;
	};//class
	PLUGINLIB_EXPORT_CLASS(lu_nodelet::outlierRemovalFilter, nodelet::Nodelet)
}//ns
