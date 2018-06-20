#include <ros/ros.h>
#include <ros/master.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <lidar_utility_msgs/lidarUtilitySettings.h>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "--------------------------------------------------------------------------------------\n"

static int mode;

ros::Publisher pc2_pub;
ros::Publisher msg_pub;
	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//Callback to echo any reciveed point cloud topics
		printf(COLOR_BLUE BAR COLOR_RST);
	ROS_INFO("--Lidar Utility: In Callback--");

	// Create a container for the data.
	sensor_msgs::PointCloud2 output;

	// Do data processing here...
	output = *input;

	// Publish the data.
	pc2_pub.publish (output);
}

	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultPublisher("lidar_utility_points");

	std::string nodeName("lidar_utility");//temp name to initialize with

	// Initialize ROS
	ros::init (argc, argv, nodeName);
	ros::NodeHandle nh;

	nodeName = ros::this_node::getName();//update node name

	//Create Parameters on the new name
	const std::string subscriberParamName(nodeName + "/subscriber");
	const std::string publisherParamName(nodeName + "/publisher");

	//Create constants for expected topics
	const std::string potentialSubscription1("/cloud_pcd");
	const std::string potentialSubscription2("/rslidar_points");
	const std::string potentialSubscription3("/pandar_points");

	printf(COLOR_BLUE BAR COLOR_RST);
	ROS_INFO("LIDAR UTILITY RUNNING");
	ROS_INFO("Node Name: %s",nodeName.c_str());

	//Create variables that control the topic names
	std::string sTopic;
	std::string pTopic;


	//Check if the user specified a subscription topic
	if(nh.hasParam(subscriberParamName)){
		nh.getParam(subscriberParamName,sTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
	}else{
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set. Searching advertised topics for appropriate subscribers...",nodeName.c_str());
		ros::master::V_TopicInfo master_topics;
		ros::master::getTopics(master_topics);


		std::string currentTopic;
		for(ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++){
			const ros::master::TopicInfo& info= *it;
			std::cout <<"topic_"<<it - master_topics.begin()<<": " <<info.name<<std::endl;

			currentTopic=info.name;
			if(currentTopic==potentialSubscription1)
			{
				ROS_INFO("%s: Topic found! Subscribing to: %s",nodeName.c_str(),currentTopic.c_str());
				sTopic=info.name;
				mode=1;
				break;

			}else if(info.name==potentialSubscription2){
				ROS_INFO("%s: Topic found! Subscribing to: %s",nodeName.c_str(),currentTopic.c_str());
				sTopic=info.name;
				mode=2;
				break;

			}else if(info.name==potentialSubscription3){
				ROS_INFO("%s: Topic found! Subscribing to: %s",nodeName.c_str(),currentTopic.c_str());
				sTopic=info.name;
				mode=3;
				break;

			}else{
				ROS_INFO("%s: Searching... If no topic if found default is: %s",nodeName.c_str(), defaultSubscriber.c_str());
				mode=4;
				sTopic=defaultSubscriber;
			}

		}//end of for loop
	}//end of subscription check

	//Check if the user specified a publishing topic
	if(nh.hasParam(publisherParamName)){
		nh.getParam(publisherParamName,pTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
	}else{
		pTopic=defaultPublisher;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
	}

	//Clears the assigned parameters Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);
	//nh.deleteParam(modeParamName);
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);
	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	msg_pub=nh.advertise<lidar_utility_msgs::lidarUtilitySettings>(pTopic+"_settings",1);


//custom msg
lidar_utility_msgs::lidarUtilitySettings msg;

msg.headerstamp = ros::Time::now();
msg.header.frame_id = "/world";
msg.downSampleLeafSize_A = 00;
msg.downSampleLeafSize_B = 0.01;
msg.planeDistanceThreshold = 00;
msg.lowestRoadPoint = 00;
msg.highestRoadPoint = 00;
msg.lowestObjectPoint = 00;
msg.highestObjectPoint = 00;
msg.outlierMeanK = 00;
msg.outlierStdDev = 00;
msg.outlierRadius = 00;
msg.outlierMinumNeighbors = 00;
msg.intensityMinimum_A = 00;
msg.intensityMaximum_A = 00;
msg.intensityMinimum_B = 00;
msg.intensityMaximum_B = 00;
msg.planeSegMaxIterations = 00;
msg.planeSegDistThresh = 00;
msg.objDetectMaxIterations = 100;
msg.objDetectDistThresh = 0.02;
msg.objDetectClusterTolerance = 0.325;//a
msg.objDetectMinClusterSize = 100;
msg.objDetectMaxClusterSize = 40000;



if(mode==1){

}else if (mode==2){

}else if (mode==3){

}else if (mode==4){

}else{

}

msg_pub.publish(msg);

	// Spin
	ros::spin ();
}
