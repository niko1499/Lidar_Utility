#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
int mode =1;//fix this later

//This node subscribes to a PointCloud2 topic, searches for the road, and publishes xxx. 

ros::Publisher pub;

	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
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
		/*
		   std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
		   for (size_t i = 0; i < cloud->points.size (); ++i)
		   std::cerr << "    " << cloud->points[i].x << " "
		   << cloud->points[i].y << " "
		   << cloud->points[i].z << std::endl;
		 */

		//NEW CONVERSION https://stackoverflow.com/questions/36380217/pclpclpointcloud2-usage
		//or not
		/*

		   fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, cl::PointCloud<PointT>& cloud);
		   void toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg);

		 */
		/*

		   pcl::
		   pcl::fromPCLPointCloud2 (*cloud_msg, pcl_pc);


		   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		seg.setInputCloud (cloudPtrXYX);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
		ROS_INFO("ERROR: Could not estimate a planar model for the given dataset.");
		}

		std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " " 
		<< coefficients->values[3] << std::endl;

		 */
		/*
		   std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
		   for (size_t i = 0; i < inliers->indices.size (); ++i)
		   std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		   << cloud->points[inliers->indices[i]].y << " "
		   << cloud->points[inliers->indices[i]].z << std::endl;
		 */
	}else if (mode==2){

	}else if (mode==3){

	}

	/*
	//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered,output);

	pcl_conversions::fromPCL(cloud_filtered,output);


	// Publish the data.
	pub.publish (output);
	 */



}

	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("road_points1");
	const std::string defaultPublisher("road");
	const std::string defaultMode("r");

	std::string nodeName("road_detective");//temp name to initialize with

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
	ROS_INFO("Mode options for parameter %s are: ""r"", ""r"", ""r"" for road, , and ",modeParamName.c_str());
	//Create variables that control the topic names
	std::string sTopic;
	std::string pTopic;
	std::string myMode;


	//Check if the user specified a subscription topic
	if(nh.hasParam(subscriberParamName)){
		nh.getParam(subscriberParamName,sTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("A param has been set **%s** \nSetting subsceiber to: %s",subscriberParamName.c_str(), sTopic.c_str());
	}else{
		sTopic=defaultSubscriber;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("No param set **%s**  \nSetting subsceiber to: %s",subscriberParamName.c_str(), sTopic.c_str());
	}

	//Check if the user specified a publishing topic
	if(nh.hasParam(publisherParamName)){
		printf(COLOR_GREEN BAR COLOR_RST);
		nh.getParam(publisherParamName,pTopic);
		ROS_INFO("A param has been set **%s** \nSetting publisher to: %s",publisherParamName.c_str(), pTopic.c_str());
	}else{printf(COLOR_RED BAR COLOR_RST);
		pTopic=defaultPublisher;//set to default if not specified
		ROS_INFO("No param set **%s** \nSetting publisher to: %s",publisherParamName.c_str(), pTopic.c_str());
	}

	//Check if the user specified a mode
	if(nh.hasParam(modeParamName)){
		nh.getParam(modeParamName,myMode);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("A param has been set **%s** \nSetting mode to: %s",modeParamName.c_str(), myMode.c_str());
	}else{
		myMode=defaultMode;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("No param set **%s** \nSetting mode to: %s",modeParamName.c_str(), myMode.c_str());
		ROS_INFO("Mode options for parameter %s are: ""s"", ""r"", ""c"" for statistical, radial, and conditional",modeParamName.c_str());
	}



	//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);
	nh.deleteParam(modeParamName);

	if(myMode=="r"||myMode=="R"){
		mode=1;
	}else if(myMode=="f"||myMode=="F"){
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
