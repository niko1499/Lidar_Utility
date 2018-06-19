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

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <plane_filter/RoadLoc.h>

//#include <pcl/common.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>

//#include <pcl/impl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
//#include <Eigen>

#include <pcl/common/projection_matrix.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
//msg
#include <plane_filter/RoadLoc.h>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
static int mode =1;//fix this later
static std::string nodeName("plane_filter");

//This node subscribes to a PointCloud2 topic, peforms a pass through filter, and republishes the point cloud. 

ros::Publisher pc2_pub;
ros::Publisher msg_pub;
	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
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
		sor.setLeafSize (0.01f, 0.01f, 0.01f);//.01 default
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
		seg.setDistanceThreshold (0.01);

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

	}





//float xMax,xMin,yMax,yMin,zMax,zMin;
//Eigen::Vector4f min_pt, max_pt;

//pcl::PointT pMin;
//pcl::PointT pMax;

//pcl::PointXYZRGB pMin;
//pcl::PointXYZRGB pMax;

pcl::PointXYZ pMin,pMax;
pcl::getMinMax3D (*cloud_p,pMin,pMax);

//custom msg

plane_filter::RoadLoc msg;

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
}
	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultPublisher("passThrough_filtered");
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
		printf(COLOR_GREEN BAR COLOR_RST);		
		nh.getParam(modeParamName,myMode);

		ROS_INFO("%s: A param has been set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
	}else{
		myMode=defaultMode;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
		
	}

	//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic

	ROS_INFO("%s: Mode options for parameter %s are: ""segmentation"", ""projection"", ""x""",nodeName.c_str(),modeParamName.c_str());
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);
	nh.deleteParam(modeParamName);
	if(myMode=="S"||myMode=="S"||myMode=="segmentation"){
		mode=1;
	}else if(myMode=="p"||myMode=="P"||myMode=="projection"){
		mode=2;
	}else if(myMode=="c"||myMode=="C"){
		mode=3;
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> ("testx", 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	msg_pub=nh.advertise<plane_filter::RoadLoc>(pTopic+"_msg",1);


	// Spin
	ros::spin ();
}
