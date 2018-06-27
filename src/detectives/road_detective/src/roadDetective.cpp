#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
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
ros::Publisher poly_pub;
static 	float xMinRoad, xMaxRoad, yMinRoad, yMaxRoad, zMinRoad, zMaxRoad;
void road_cb (const lidar_utility_msgs::roadInfo& data)
{
	ROS_INFO("%s: In msg Callback",nodeName.c_str());
	xMinRoad = data.xMin;
	xMaxRoad = data.xMax;
	yMinRoad = data.yMin;
	yMaxRoad = data.yMax;
	zMinRoad = data.zMin;
	zMaxRoad = data.zMax;	
	
	geometry_msgs::Point32 p1;
	geometry_msgs::Point32 p2;
	geometry_msgs::Point32 p3;
	geometry_msgs::Point32 p4;

	p1.x=xMinRoad;
	p1.y=yMinRoad;
	p1.z=zMaxRoad;
	p2.x=xMinRoad;
	p2.y=yMaxRoad;
	p2.z=zMaxRoad;
	p3.x=xMaxRoad;
	p3.y=yMinRoad;
	p3.z=zMaxRoad;
	p4.x=xMaxRoad;
	p4.y=yMaxRoad;
	p4.z=zMaxRoad;
	

	geometry_msgs::PolygonStamped polygon;
	polygon.header.stamp = ros::Time();
	polygon.polygon.points.reserve(4);
	polygon.polygon.points[0]=p1;
	polygon.polygon.points[1]=p2;
	polygon.polygon.points[2]=p3;
	polygon.polygon.points[3]=p4;
	
				poly_pub.publish(polygon);
}
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
	ROS_INFO("ObjectDetective: In Callback");
	

}
	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("road_points");
	const std::string defaultSubscriber2("plane_segmented_msg");
	const std::string defaultPublisher("road");//dynamic name ending See below
	const std::string defaultMode("f");

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
	poly_pub = nh.advertise<geometry_msgs::PolygonStamped> (pTopic+"_bounds", 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	ros::Subscriber sub2 = nh.subscribe(sTopic2.c_str(), 1, road_cb);

	// Spin
	ros::spin ();
}
