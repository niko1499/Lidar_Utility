#include <iostream>

#include <stdio.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#define COLOR_RED "\x1b[31m"
#define COLOR_GREEN "\x1b[31"
#define COLOR_RST "\x1b[0m"
int mode =1;//fix this later

//This node subscribes to a PointCloud2 topic, peforms a statistical outlier filter, and republishes the point cloud. 

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	
	//Callback for filtering and republishing recived data
	//Comment out as needed. Useful for debuging
	ROS_INFO("Outlier Removal Filer: In Callback");
	// Create a container for the data and filtered data.
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Do data processing here...

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	//Perform filtering

	//KEEP THIS BLOCK FOR REFERENCE. 
	//Examples from PCL.org will need to be changed to match format	
	/*
	   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	   sor.setInputCloud(cloudPtr);
	   sor.setLeafSize(0.1,0.1,0.1);
	   sor.filter (cloud_filtered);
	 */

	//OUTLIER REMOVAL
	if(mode==1){
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setMeanK (75);//THE NUMBER OF NEIGHBORS TO ANALIZE FOR EACH POINT 50 defaulet
	sor.setStddevMulThresh (.9);//STD DEV MULTIPLIER 1.0 default
	sor.filter (cloud_filtered);
	}else if (mode==2){
/*
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloudPtr);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (cloud_filtered);

	}else if (mode==3){
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloudPtr);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (cloud_filtered);*/
	}

	//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered,output);

	pcl_conversions::fromPCL(cloud_filtered,output);

	// Publish the data.
	pub.publish (output);
}


	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultPublisher("outliers_filtered");
	const std::string defaultMode("s");

	std::string nodeName("outlier_removal_filter");//temp name to initialize with

	// Initialize ROS
	ros::init (argc, argv, nodeName);
	ros::NodeHandle nh;

	nodeName = ros::this_node::getName();//Update name

	//set parameters on new name
	const std::string subscriberParamName(nodeName + "/subscriber");
	const std::string publisherParamName(nodeName + "/publisher");
	const std::string modeParamName(nodeName + "/mode");

	ROS_INFO("Outlier Removal Running");
	ROS_INFO("Node Name: %s",nodeName.c_str());
	ROS_INFO("Mode options for parameter %s are: ""s"", ""r"", ""c"" for statistical, radial, and conditional",modeParamName.c_str());
	//Create variables that control the topic names
	std::string sTopic;
	std::string pTopic;
	std::string myMode;
	

	//Check if the user specified a subscription topic
	if(nh.hasParam(subscriberParamName)){
		nh.getParam(subscriberParamName,sTopic);
		ROS_INFO("A param has been set **%s** \n         Setting subsceiber to: %s",subscriberParamName.c_str(), sTopic.c_str());
	}else{
		sTopic=defaultSubscriber;//set to default if not specified
		ROS_INFO("No param set **%s**  \n         Setting subsceiber to: %s",subscriberParamName.c_str(), sTopic.c_str());
	}

	//Check if the user specified a publishing topic
	if(nh.hasParam(publisherParamName)){
		nh.getParam(publisherParamName,pTopic);
		ROS_INFO("A param has been set **%s** \n          Setting publisher to: %s",publisherParamName.c_str(), pTopic.c_str());
	}else{
		pTopic=defaultPublisher;//set to default if not specified
		ROS_INFO("No param set **%s** \n          Setting publisher to: %s",publisherParamName.c_str(), pTopic.c_str());
	}

	//Check if the user specified a mode
	if(nh.hasParam(modeParamName)){
		nh.getParam(modeParamName,myMode);
		ROS_INFO("A param has been set **%s** \n          Setting mode to: %s",modeParamName.c_str(), myMode.c_str());
	}else{
		myMode=defaultMode;//set to default if not specified
		ROS_INFO("No param set **%s** \n          Setting publisher to: %s",modeParamName.c_str(), myMode.c_str());
		ROS_INFO("Mode options for parameter %s are: ""s"", ""r"", ""c"" for statistical, radial, and conditional",modeParamName.c_str());
	}



	//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);

if(myMode=="s"||myMode=="S"){
		mode=1;
	}else if(myMode=="r"||myMode=="R"){
		mode=2;
	}else if(myMode=="c"||myMode=="C"){
mode=3;
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);

	ROS_INFO("Subscribing to %s",sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("Publishing to %s",pTopic.c_str());

	// Spin
	ros::spin ();
}
