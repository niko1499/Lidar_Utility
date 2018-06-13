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

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

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



visualization_msgs::Marker markerBuilder(int i,float xLoc,float yLoc, float zLoc, float xScale, float yScale, float zScale,int type){
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
	}else{//type unknown
		r=1.0;
		g=1.0;
		b=1.0;
	}
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = i;
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

	//https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

	ROS_INFO("ObjectDetective: In Callback");
	// Create a container for the data and filtered data.
	//-----------------------
	//NEW CONVERSION
/*
	pcl::PCLPointCloud2 pcl_pc2;//create a PCL PointCloud2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert the ROS PC2 to PCL PC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create a PCL PointXYZ
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert the PCL PC2 to PCL XYZ
*/




/*
	sensor_msgs::PointCloud2 output;//create output in ROS format
	pcl_conversions::fromPCL(cloud_filtered,output);//convert result to ros output
	pc2_pub.publish (output);//publish the output
*/

	//----------------------
	int n=2;//number of objects currently detected

	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker marker;

	//add for loop through point clusters processing for centroid and extrapolating geometry and pose.


	marker=markerBuilder(0,-5,5,0,1,1,1,3);
	markerArray.markers.push_back(marker);
	marker=markerBuilder(1,0,0,0,1,1,1,9);
	markerArray.markers.push_back(marker);
	marker=markerBuilder(2,9,0,9,1,1,1,9);
	markerArray.markers.push_back(marker);
	vis_pub.publish(markerArray);


	if(mode==1){

	}else if (mode==2){

	}else if (mode==3){

	}

}


	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("object_points1");
	const std::string defaultPublisher("objects_");
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
	ROS_INFO("%s: Mode options for parameter %s are: ""r"", ""r"", ""r"" for road, , and ",nodeName.c_str(),modeParamName.c_str());
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
		nh.getParam(modeParamName,myMode);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
	}else{
		myMode=defaultMode;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
		ROS_INFO("%s: Mode options for parameter %s are: ""s"", ""r"", ""c"" for statistical, radial, and conditional",nodeName.c_str(),modeParamName.c_str());
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

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud


	vis_pub = nh.advertise<visualization_msgs::MarkerArray>( pTopic+"_visualized", 0 );

	//ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	//pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic+"_points", 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	// Spin
	ros::spin ();
}
