#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

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
ros::Publisher vis_pub;



visualization_msgs::Marker markerBuilder(float xLoc,float yLoc, float zLoc, float xScale, float yScale, float zScale,int type){
float r,g,b;
if(type==1){//truck||bus
r=1.0;
g=0.0;
b=0.0;
}else if(type==2){//car
r=0.0;
g=1.0;
b=0.0;
}else if(type==3){//person
r=0.0;
g=0.0;
b=1.0;
}else if(type==4){//bike||motorcycle
r=0.0;
g=1.0;
b=1.0;
}else if(type==5){//type unknown
r=1.0;
g=1.0;
b=1.0;
}
visualization_msgs::Marker marker;
marker.header.frame_id = "base_link";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::CUBE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = xLoc;
marker.pose.position.y = yLoc;
marker.pose.position.z = zLoc;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = xScale;
marker.scale.y = yScale;
marker.scale.z = zScale;
marker.color.a = 0.5; // Don't forget to set the alpha!
marker.color.r = r;
marker.color.g = g;
marker.color.b = b;
//only if using a MESH_RESOURCE marker type:
marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
return marker;
}

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


visualization_msgs::Marker markerArray;

markerArray=markerBuilder(0,5,0,1,1,1,3);

vis_pub.publish( markerArray );


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
	const std::string defaultSubscriber("object_points1");
	const std::string defaultPublisher("objects_visualized");
	const std::string defaultMode("o");

	std::string nodeName("object_detective");//temp name to initialize with

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

	if(myMode=="o"||myMode=="O"){
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


	vis_pub = nh.advertise<visualization_msgs::Marker>( pTopic, 0 );

//ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	//pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("Publishing to %s",pTopic.c_str());

	// Spin
	ros::spin ();
}
