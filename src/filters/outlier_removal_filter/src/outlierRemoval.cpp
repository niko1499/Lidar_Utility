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

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
int mode =1;//fix this later
static std::string nodeName("outlier_removal_filter");
//This node subscribes to a PointCloud2 topic, peforms a statistical outlier filter, and republishes the point cloud. 

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//Callback for filtering and republishing recived data
	//Comment out as needed. Useful for debuging
	ROS_INFO("%s: In Callback",nodeName.c_str());
	// Create a container for the data and filtered data.
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Do data processing here...

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	//OUTLIER REMOVAL
	if(mode==1){
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (cloudPtr);
		sor.setMeanK (75);//THE NUMBER OF NEIGHBORS TO ANALIZE FOR EACH POINT 50 defaulet//SETTING
		sor.setStddevMulThresh (.9);//STD DEV MULTIPLIER 1.0 default//SETTING
		sor.filter (cloud_filtered);
	}else if (mode==2){
		//Convert PointCloud2 to PointXYZ

		pcl::PCLPointCloud2 temp;
		pcl_conversions::toPCL(*cloud_msg,temp);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(temp,*cloudXYZ);


		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		// build the filter
		outrem.setInputCloud(cloudXYZ);
		outrem.setRadiusSearch(0.8);//SETTING
		outrem.setMinNeighborsInRadius (2);//SETTING
		//setup xyzholder

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredXYZ (new pcl::PointCloud<pcl::PointXYZ>);
		// apply filter
		outrem.filter (*cloud_filteredXYZ);
		//convert bask
		//		pcl::toPCLPointCloud2(*cloud_filteredXYZ,*cloud_filtered);
		//pcl_conversions::toPPCLPointCloud2(cloud_filteredXYZ,cloud_filtered);

	}else if (mode==3){/*
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
		ROS_INFO("%s: No param set **%s**\nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
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
		nh.getParam(modeParamName,myMode);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
	}else{
		myMode=defaultMode;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
		ROS_INFO("%s: Mode options for parameter %s are: ""statistical"", ""radial"", ""conditional""",nodeName.c_str(),modeParamName.c_str());
	}

	//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);
	nh.deleteParam(modeParamName);
	if(myMode=="S"||myMode=="S"||myMode=="statistical"){
		mode=1;
	}else if(myMode=="r"||myMode=="R"||myMode=="radial"){
		mode=2;
	}else if(myMode=="c"||myMode=="C"||myMode=="conditional"){
		mode=3;
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	// Spin
	ros::spin ();
}
