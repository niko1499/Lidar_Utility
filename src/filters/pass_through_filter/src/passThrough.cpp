#include <ros/ros.h>
#include <lidar_utility_msgs/lidarUtilitySettings.h>
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

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
static int mode =1;//fix this later
static std::string nodeName("pass_through_filter");
static 	float xMinf, xMaxf, yMinf, yMaxf, zMinf, zMaxf;
static bool called = false;

static float roadMin_setting=-4;
static float roadMax_setting=-1.2;
static float objectMin_setting=-1.7;
static float objectMax_setting=1.25;
static float boxMargin=.2;
static float yRangeBoost=0;

//This node subscribes to a PointCloud2 topic, peforms a pass through filter, and republishes the point cloud. 
ros::Publisher pc2_pub;
void settings_cb (const lidar_utility_msgs::lidarUtilitySettings& data)
{
 roadMin_setting=data.passThroughRoadMin;
 roadMax_setting=data.passThroughRoadMax;
 objectMin_setting=data.passThroughObjectMin;
 objectMax_setting=data.passThroughObjectMax;
 boxMargin=data.passThroughBoxMargin;
 yRangeBoost=data.passThroughRangeBoost;
}

void message_cb (const lidar_utility_msgs::roadInfo& data)
{
	xMinf = data.xMin;
	xMaxf = data.xMax;
	yMinf = data.yMin;
	yMaxf = data.yMax;
	zMinf = data.zMin;
	zMaxf = data.zMax;	
}	

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

	//pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;

	//Callback for filtering and republishing recived data
	ROS_INFO("%s: In Callback",nodeName.c_str());
	// Create a container for the data and filtered data.
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Do data processing here...

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	called = true;


	// Do data processing here...

	//Convert to PCL data type
	//pcl_conversions::toPCL(*cloud_msg, *cloud);

	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud (cloudPtr);

	if(mode==1){//road
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (roadMin_setting, roadMax_setting);//FOR VELODYNE +Z is DOWN VALUDES FOR VELODYNE PCD (-1.5,7.0)//SETTING
		pass.setFilterLimitsNegative (false);

		pass.filter (cloud_filtered);

		//convert to ROS data type
		sensor_msgs::PointCloud2 output;

		pcl_conversions::fromPCL(cloud_filtered,output);
		// Publish the data.
		pc2_pub.publish (output);

	}else if (mode==2){//objects
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (objectMin_setting,objectMax_setting);//FOR VELODYNE +Z is DOWN VALUDES FOR VELODYNE PCD ()//SETTING
		pass.setFilterLimitsNegative (false);

		pass.filter (cloud_filtered);

		//convert to ROS data type
		sensor_msgs::PointCloud2 output;

		pcl_conversions::fromPCL(cloud_filtered,output);
		// Publish the data.
		pc2_pub.publish (output);

	}else if (mode==3){//adv objects

		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ


		pcl::PointIndices::Ptr indices_x (new pcl::PointIndices);
		pcl::PointIndices::Ptr indices_xy (new pcl::PointIndices);

		///pcl::PCLPointX cloud_filtered_x;
		//pcl::PCLPointCloud2 cloud_filtered_xz;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices
		ptfilter.setInputCloud (temp_cloud);
		ptfilter.setFilterFieldName ("x");
		ptfilter.setFilterLimits (xMinf+boxMargin, xMaxf-boxMargin);
		ptfilter.filter (*cloud_filtered_x);

		ptfilter.setInputCloud(cloud_filtered_x);
		ptfilter.setFilterFieldName ("y");
		ptfilter.setFilterLimits (yMinf+boxMargin,yMaxf-boxMargin+yRangeBoost);
		ptfilter.filter (*cloud_filtered_xy);

	//float zMid = zMaxf -((zMaxf-zMinf)/2);

		ptfilter.setInputCloud(cloud_filtered_xy);
		ptfilter.setFilterFieldName ("z");
		ptfilter.setFilterLimits (zMaxf-.1,1.2);//SETTING
		ptfilter.setNegative (false);
		ptfilter.filter (*cloud_filtered_xyz);

		sensor_msgs::PointCloud2 output;//create output container
		pcl::PCLPointCloud2 temp_output;//create PCLPC2
		pcl::toPCLPointCloud2(*cloud_filtered_xyz,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
		pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
		pc2_pub.publish (output);// Publish the data.
	}else if(mode==4){
pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ


		pcl::PointIndices::Ptr indices_x (new pcl::PointIndices);
		pcl::PointIndices::Ptr indices_xy (new pcl::PointIndices);

		///pcl::PCLPointX cloud_filtered_x;
		//pcl::PCLPointCloud2 cloud_filtered_xz;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices
		ptfilter.setInputCloud (temp_cloud);
		ptfilter.setFilterFieldName ("y");
		ptfilter.setFilterLimits (-1,23);
		ptfilter.filter (*cloud_filtered_x);

		ptfilter.setInputCloud(cloud_filtered_x);
		ptfilter.setFilterFieldName ("x");
		ptfilter.setFilterLimits (-12,12);
		ptfilter.filter (*cloud_filtered_xy);

	//float zMid = zMaxf -((zMaxf-zMinf)/2);

		ptfilter.setInputCloud(cloud_filtered_xy);
		ptfilter.setFilterFieldName ("z");
		ptfilter.setFilterLimits (-3,4);//SETTING
		ptfilter.setNegative (false);
		ptfilter.filter (*cloud_filtered_xyz);

		sensor_msgs::PointCloud2 output;//create output container
		pcl::PCLPointCloud2 temp_output;//create PCLPC2
		pcl::toPCLPointCloud2(*cloud_filtered_xyz,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
		pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
		pc2_pub.publish (output);// Publish the data.
	}
}
	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultSubscriber2("plane_segmented_msg");
	const std::string defaultPublisher("passThrough_filtered");
	const std::string defaultMode("r");

	// Initialize ROS
	ros::init (argc, argv, nodeName);
	ros::NodeHandle nh;

	nodeName = ros::this_node::getName();//Update name

	//set parameters on new name
	const std::string subscriberParamName(nodeName + "/subscriber");
	const std::string subscriberParamName2(nodeName + "/msgSubscriber");
	const std::string publisherParamName(nodeName + "/publisher");
	const std::string modeParamName(nodeName + "/mode");
	printf(COLOR_BLUE BAR COLOR_RST);
	ROS_INFO("Node Name: %s",nodeName.c_str());

	//Create variables that control the topic names
	std::string sTopic;
	std::string sTopic2;
	std::string pTopic;
	std::string myMode;

	if(nh.hasParam(subscriberParamName)){//Check if the user specified a subscription topic
		nh.getParam(subscriberParamName,sTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
	}else{
		sTopic=defaultSubscriber;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s**  \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
	}

	if(nh.hasParam(subscriberParamName2)){//Check if the user specified a subscription topic for msgs
		nh.getParam(subscriberParamName2,sTopic2);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
	}else{
		sTopic2=defaultSubscriber2;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s**  \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
	}

	if(nh.hasParam(publisherParamName)){//Check if the user specified a publishing topic
		printf(COLOR_GREEN BAR COLOR_RST);
		nh.getParam(publisherParamName,pTopic);
		ROS_INFO("%s: A param has been set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
	}else{printf(COLOR_RED BAR COLOR_RST);
		pTopic=defaultPublisher;//set to default if not specified
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
	nh.deleteParam(subscriberParamName2);
	if(myMode=="1"||myMode=="R"||myMode=="road"){
		mode=1;
	}else if(myMode=="o"||myMode=="O"||myMode=="objects"){
		mode=2;
	}else if(myMode=="3"||myMode=="C"||myMode=="advObjects"){
		mode=3;
	}else if(myMode=="4"||myMode=="F"||myMode=="forward"){
		mode=4;
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());
	//if(mode==3){
	ros::Subscriber sub2 = nh.subscribe(sTopic2.c_str(), 1, message_cb);
	//}
	ros::Subscriber sub3 = nh.subscribe("lidar_utility_settings", 1, settings_cb);
	
	ros::spin();
}		

