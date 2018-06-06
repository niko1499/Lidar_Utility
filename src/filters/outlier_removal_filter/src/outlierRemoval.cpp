#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>



ros::Publisher pub;

	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	ROS_INFO("In Callback");
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

	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setMeanK (50);//50 defaule
	sor.setStddevMulThresh (1.0);//1.0 default
	sor.filter (cloud_filtered);

	//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered,output);

	pcl_conversions::fromPCL(cloud_filtered,output);


	/*	
		pcl::PCLPointCloud2 tmp_cloud;
		pcl::toPCLpointCloud2(*cloud_filtered,tmp_cloud);
		pcl_conversions::fromPCL(tmp_cloud,output);
	 */

	// Publish the data.
	pub.publish (output);
}

	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultPublisher("outliers_filtered");
	//asdfasdfasdfasddf

	//std::string name= ros::this_node::getNamespace();
	//ROS_INFO(name.c_str());
	const std::string nodeName("outlier_removal_filter");
	const std::string subscriberParamName(nodeName + "/subscriber");
	const std::string publisherParamName(nodeName + "/publisher");
	ROS_INFO("Outlier Removal Running");
	// Initialize ROS
	ros::init (argc, argv, nodeName);
	ros::NodeHandle nh;

	//Create variables that control the topic names
	std::string sTopic;
	std::string pTopic;

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

	//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);
	ROS_INFO("Subscribing to %s",sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("Publishing to %s",pTopic.c_str());

	// Spin
	ros::spin ();
}
