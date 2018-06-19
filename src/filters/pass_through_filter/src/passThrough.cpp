#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <lidar_utility_msgs/roadInfo.h>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
static int mode =1;//fix this later
static std::string nodeName("pass_through_filter");

//This node subscribes to a PointCloud2 topic, peforms a pass through filter, and republishes the point cloud. 

ros::Publisher pc2_pub;

class Listener{
	public:
	float xMin, xMax, yMin, yMax, zMin, zMax;
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	void message_cb(const lidar_utility_msgs::roadInfo& data);
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};


void Listener::message_cb (const lidar_utility_msgs::roadInfo& data)
{
	xMin = data.xMin;
	xMax = data.xMax;
	yMin = data.yMin;
	yMax = data.yMax;
	zMin = data.zMin;
	zMax = data.zMax;	
}	

void Listener::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//Callback for filtering and republishing recived data
	ROS_INFO("%s: In Callback",nodeName.c_str());
	// Create a container for the data and filtered data.

	pcl_conversions::toPCL(*cloud_msg, *cloud);


	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	pcl::PCLPointCloud2 cloud_filtered;
/*

//asdf


	// Do data processing here...

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
*/
pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud (cloudPtr);

	if(mode==1){
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-1.2, 7.0);//FOR VELODYNE +Z is DOWN VALUDES FOR VELODYNE PCD (-1.5,7.0)
		pass.setFilterLimitsNegative (true);


	}else if (mode==2){
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-1.65,1.25);//FOR VELODYNE +Z is DOWN VALUDES FOR VELODYNE PCD ()
		pass.setFilterLimitsNegative (false);
	}else if (mode==3){
/*
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (listener.xMin,listener.xMax);
pass.setFilterFieldName ("y");
		pass.setFilterLimits (listener.yMin,listener.yMax);
pass.setFilterFieldName ("z");
		pass.setFilterLimits (listener.zMin,listener.zMax);
		pass.setFilterLimitsNegative (false);
*/

	}

		pass.filter (cloud_filtered);

	//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered,output);

	pcl_conversions::fromPCL(cloud_filtered,output);
	// Publish the data.
	pc2_pub.publish (output);


}

	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultPublisher("passThrough_filtered");
	const std::string defaultMode("r");

	// Initialize ROS
	ros::init (argc, argv, nodeName);
	ros::NodeHandle nh;

	nodeName = ros::this_node::getName();//Update name

	//set parameters on new name
	const std::string subscriberParamName(nodeName + "/subscriber");
	const std::string publisherParamName(nodeName + "/publisher");
	const std::string modeParamName(nodeName + "/mode");
	printf(COLOR_BLUE BAR COLOR_RST);
	ROS_INFO("Node Name: %s",nodeName.c_str());

	//Create variables that control the topic names
	std::string sTopic;
	std::string pTopic;
	std::string myMode;

	//Check if the user specified a subscription topic
	if(nh.hasParam(subscriberParamName)){
		nh.getParam(subscriberParamName,sTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
	}else{
		sTopic=defaultSubscriber;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s**  \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
	}

	//Check if the user specified a publishing topic
	if(nh.hasParam(publisherParamName)){
		printf(COLOR_GREEN BAR COLOR_RST);
		nh.getParam(publisherParamName,pTopic);
		ROS_INFO("%s: A param has been set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
	}else{printf(COLOR_RED BAR COLOR_RST);
		pTopic=defaultPublisher;//set to default if not specified
		ROS_INFO("%s: No param set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
	}

	//Check if the user specified a mode
	if(nh.hasParam(modeParamName)){
		printf(COLOR_GREEN BAR COLOR_RST);		
		nh.getParam(modeParamName,myMode);

		ROS_INFO("%s: A param has been set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
	}else{
		myMode=defaultMode;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
		ROS_INFO("%s: Mode options for parameter %s are: ""road"", ""objects"", ""x""",nodeName.c_str(),modeParamName.c_str());
	}

	//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);
	nh.deleteParam(modeParamName);
	if(myMode=="1"||myMode=="R"||myMode=="road"){
		mode=1;
	}else if(myMode=="o"||myMode=="O"||myMode=="objects"){
		mode=2;
	}else if(myMode=="3"||myMode=="C"||myMode=="asdf"){
		mode=3;
	}
Listener listener;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, &Listener::cloud_cb, &listener);

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());



ros::Subscriber sub2 = nh.subscribe("topic_subscribed", 1, &Listener::message_cb, &listener);
	
	while(ros::ok()){
		ros::spinOnce();

		// = listener.xMin;
/*
	pcl::PCLPointCloud2 cloud_filtered;
printf("test((( %f",listener.xMax);
	pcl::PCLPointCloud2ConstPtr cloudPtr(listener.cloud);
	
	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud (cloudPtr);

	if(mode==1){
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-1.2, 7.0);//FOR VELODYNE +Z is DOWN VALUDES FOR VELODYNE PCD (-1.5,7.0)
		pass.setFilterLimitsNegative (true);


	}else if (mode==2){
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-1.65,1.25);//FOR VELODYNE +Z is DOWN VALUDES FOR VELODYNE PCD ()
		pass.setFilterLimitsNegative (false);
	}else if (mode==3){
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (listener.xMin,listener.xMax);
pass.setFilterFieldName ("y");
		pass.setFilterLimits (listener.yMin,listener.yMax);
pass.setFilterFieldName ("z");
		pass.setFilterLimits (listener.zMin,listener.zMax);
		pass.setFilterLimitsNegative (false);
	}

		pass.filter (cloud_filtered);

	//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered,output);

	pcl_conversions::fromPCL(cloud_filtered,output);
	// Publish the data.
	pc2_pub.publish (output);
		//loop_rate.sleep();
*/

	}	

	}		

