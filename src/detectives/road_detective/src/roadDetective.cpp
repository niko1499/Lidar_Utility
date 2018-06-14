#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/OccupancyGrid.h"

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

#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
//#include <pcl/centroid.hpp>


 
#include "pcl/ros/conversions.h"
#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
static int mode =1;//fix this later
static std::string nodeName("road_detective");

//This node subscribes to a PointCloud2 topic, searches for the road, and publishes xxx. 

ros::Publisher pc2_pub;
ros::Publisher vis_pub;
ros::Publisher grid_pub;

visualization_msgs::Marker markerBuilder(int i,float xLoc,float yLoc, float zLoc, float xScale, float yScale, float zScale,int type){
	float r,g,b;
	float alpha=.5;
	if(type==1){//road
		r=0.0;
		g=0.0;
		b=0.0;
		alpha=.9;
	}else if(type==2){//type x
		r=0.0;
		g=1.0;
		b=0.0;
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
	marker.color.a =alpha; // Don't forget to set the alpha!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	return marker;
}

nav_msgs::OccupancyGrid OgridBuilder(){

	nav_msgs::OccupancyGrid grid;
	grid.header.frame_id = "base_link";
	grid.header.stamp = ros::Time();


	return grid;
}


	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/
	ROS_INFO("ObjectDetective: In Callback");
	

	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker marker;


//pcl::CentroidPoint<pcl::PointXYZ> centroid;
Eigen::Matrix<Scalar,4,1> centroid;

 //Eigen::Vector4f centroid;

//	pcl::CentroidPoint<pcl::PointXYZ> centroid;
	//pcl::Centroid<pcl::PointXYZ> centroid;
	 pcl::compute3DCentroid(temp_cloud,centroid);
		pcl::PointXYZ c1;
//	centroid.get (c1);

float xMin=0,xMax=0,yMin=0,yMax=0,zMin=0,zMax=0;
/*
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (temp_cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.1); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (250000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (temp_cloud);
	ec.extract (cluster_indices);
*/
/*
for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
if(temp_cloud->points[*it].x>xMax){
xMax=temp_cloud->points[*it];
}else if (temp_cloud->points[*it].x<xMin){
xMin=temp_cloud->points[*it].x;
}
 if(temp_cloud->points[*it].y>yMax){
yMax=temp_cloud->points[*it];
}else if (temp_cloud->points[*it].y<yMin){
yMin=temp_cloud->points[*it].y;
}
if(temp_cloud->points[*it].z>zMax){
zMax=temp_cloud->points[*it];
}else if (temp_cloud->points[*it].z<zMin){
zMin=temp_cloud->points[*it].z;
}
}

*/
/*

for (size_t i = 0; i < temp_cloud.size(); ++i)
    {//find min and max
if(temp_cloud[i].x>xMax){
xMax=temp_cloud[i];
}else if (temp_cloud[i].x<xMin){
xMin=temp_cloud[i];
}
 if(temp_cloud[i].y>yMax){
yMax=temp_cloud[i];
}else if (temp_cloud[i].y<yMin){
yMin=temp_cloud[i];
}
if(temp_cloud[i].z>zMax){
zMax=temp_cloud[i];
}else if (temp_cloud[i].z<zMin){
zMin=temp_cloud[i];
}
   }

*/
float xScale=abs(xMax)-abs(xMin);
float yScale=abs(yMax)-abs(yMin);
float zScale=abs(zMax)-abs(zMin);

 xScale=10;
 yScale=10;
 zScale=.5;
			marker=markerBuilder(1,c1.x,c1.y,c1.z,xScale,yScale,zScale,1);
			markerArray.markers.push_back(marker);
		
	
	//publih
	vis_pub.publish(markerArray);

	sensor_msgs::PointCloud2 output;//create output container
	pcl::PCLPointCloud2 temp_output;//create PCLPC2

	pcl::toPCLPointCloud2(*temp_cloud,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
	pc2_pub.publish (output);// Publish the data.
}
	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("road_points");
	const std::string defaultPublisher("road");//dynamic name ending See belows
	const std::string defaultMode("f");

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
		nh.getParam(modeParamName,myMode);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
	}else{
		myMode=defaultMode;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
		ROS_INFO("%s: Mode options for parameter %s are: ""filtered"", ""unfiltered"", ""l"" for statistical, radial, and conditional",nodeName.c_str(),modeParamName.c_str());
	}

	//Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);
	nh.deleteParam(modeParamName);

	if(myMode=="f"||myMode=="F"||myMode=="filtered"){
		mode=1;
	}else if(myMode=="u"||myMode=="U"||myMode=="unfiltered"){
		mode=2;
	}else if(myMode=="l"||myMode=="L"){
		mode=3;//2 then 3
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud

	vis_pub = nh.advertise<visualization_msgs::MarkerArray>( pTopic+"_detective_visualized", 0 );


	//ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	//pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic+"_detective_points", 1);


	grid_pub = nh.advertise<nav_msgs::OccupancyGrid> (pTopic+"_detective_grid", 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	// Spin
	ros::spin ();
}
