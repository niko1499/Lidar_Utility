//ROS
#include <ros/ros.h>
//C
#include <string>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <iostream>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "--------------------------------------------------------------------------------------\n"
static std::string nodeName("pass_through_filter");

static int mode;

ros::Publisher pc2_pub;
ros::Publisher msg_pub;
	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	ROS_INFO("%s: In Callback",nodeName.c_str());

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)

	pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointNormal> ());

	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*input,pcl_pc2);//convert ROSPC2 to PCLPC2

	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ

	pcl::PointCloud<pcl::PointNormal> temp_cloud2;
	//float theta =M_PI/2;
	float theta = 0;// define rotation
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0,0)= cos(theta);
	transform(0,1)= -sin(theta);
	transform(1,0)= sin(theta);
	transform(1,1)= cos(theta);


	transform (0,3) = 2.5;// Define a translation of 2.5 meters on the x axis.

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ> ());

	printf ("Transform: Matrix4f\n");
	std::cout << transform << std::endl;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::transformPointCloudWithNormals(*temp_cloud,*cloud_transformed,transform);

	sensor_msgs::PointCloud2 output;//create output container
	pcl::PCLPointCloud2 temp_output;//create PCLPC2
	pcl::toPCLPointCloud2(*cloud_transformed,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type



	// Publish the data.
	pc2_pub.publish (output);
}

	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultPublisher("output");
	const std::string defaultMode("1");

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

	if(nh.hasParam(subscriberParamName)){//Check if the user specified a subscription topic
		nh.getParam(subscriberParamName,sTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
	}else{
		sTopic=defaultSubscriber;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s**  \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
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

	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	ros::spin();
}
