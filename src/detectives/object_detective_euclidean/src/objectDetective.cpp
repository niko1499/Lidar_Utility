//ROS:
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <lidar_utility_msgs/lidarUtilitySettings.h>
//C
#include <string>
//PCL/Basic:
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
//PCL/common:
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/search.h>
#include <pcl/common/projection_matrix.h>
//PCL/DON:
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
static int mode =1;//fix this later
static std::string nodeName("object_detective");

//This node subscribes to a PointCloud2 topic, searches for the road, and publishes xxx. 

ros::Publisher pc2_pub;
ros::Publisher vis_pub;

//settings 
static float leaf_setting=0.01;
static float setMaxIterations_setting=100;
static float setDistanceThreshold_setting=.02;
static float setClusterTolerance_setting=.325;
static float setMinClusterSize_setting=100;
static float setMaxClusterSize_setting=0000;//default 40000 0 for test

void settings_cb (const lidar_utility_msgs::lidarUtilitySettings& data)
{

	setMaxIterations_setting = data.objDetectMaxIterations;
	setDistanceThreshold_setting = data.objDetectDistThresh;
	setClusterTolerance_setting = data.objDetectClusterTolerance;
	setMinClusterSize_setting = data.objDetectMinClusterSize;
	setMaxClusterSize_setting = data.objDetectMaxClusterSize;
	leaf_setting = data.downSampleLeafSize_B;
}

visualization_msgs::Marker markerBuilder(int i,float xLoc,float yLoc, float zLoc, float xScale, float yScale, float zScale,int type){
	float r,g,b;
	float alpha=.75;
	if(type==1){//truck||bus
		r=1.0;
		g=0.0;
		b=0.0;
	}else if(type==2){//car
		r=1.0;
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
	}else if(type==5){//plate
		r=1.0;
		g=1.0;
		b=1.0;
		alpha=.85;
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
	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/
	ROS_INFO("%s: In Callback", nodeName.c_str());
	//NEW CONVERSION
	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ
	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker marker;
	//euclidian cluster extraxion
	//these were moved here for scope of mode
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDWriter writer;
	if(mode==1){
		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setInputCloud (temp_cloud);
		vg.setLeafSize (leaf_setting, leaf_setting, leaf_setting);//default (0.01f, 0.01f, 0.01f)//SETTING
		vg.filter (*cloud_filtered);
		std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (setMaxIterations_setting);//SETTING
		seg.setDistanceThreshold (setDistanceThreshold_setting);//SETTING

		int i=0, nr_points = (int) cloud_filtered->points.size ();
		while (cloud_filtered->points.size () > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (false);
			// Get the points associated with the planar surface
			extract.filter (*cloud_plane);
			std::cout << "ObjDetEuc: PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			//add missing cloud_f //NI
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			*cloud_filtered = *cloud_f;
		}
	}else if(mode==2||mode==3){
		cloud_filtered=temp_cloud;
	}
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (setClusterTolerance_setting); // 2cm//SETTING
	ec.setMinClusterSize (setMinClusterSize_setting);//SETTING
	ec.setMaxClusterSize (setMaxClusterSize_setting);//SETTING
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "ObjDetEuc: PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
		pcl::CentroidPoint<pcl::PointXYZ> centroid;//add cluster centroid to array

		for (std::vector<int>::const_iterator pit2 = it->indices.begin (); pit2 != it->indices.end (); ++pit2){
			centroid.add(pcl::PointXYZ(cloud_filtered->points[*pit2].x,cloud_filtered->points[*pit2].y,cloud_filtered->points[*pit2].z));
		}
		pcl::PointXYZ c1;
		centroid.get (c1);
		if(mode!=3){


pcl::PointXYZ pMin,pMax;
pcl::getMinMax3D (*cloud_cluster,pMin,pMax);

float xScale=abs(pMax.x-pMin.x);
float yScale=abs(pMax.y-pMin.y);
float zScale=abs(pMax.z-pMin.z);

			marker=markerBuilder(j,c1.x,c1.y,c1.z,xScale,yScale,zScale,2);
			markerArray.markers.push_back(marker);
		}else if (mode==3){
			marker=markerBuilder(j,c1.x,c1.y,c1.z,1,.5,.1,5);
			markerArray.markers.push_back(marker);
		}
	}
	//publih


	vis_pub.publish(markerArray);

	sensor_msgs::PointCloud2 output;//create output container
	pcl::PCLPointCloud2 temp_output;//create PCLPC2
	pcl::toPCLPointCloud2(*cloud_filtered,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
	pc2_pub.publish (output);// Publish the data.
}
	int
main (int argc, char** argv)
{
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("object_points1");
	const std::string defaultPublisher("objects_");
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
	ros::Subscriber sub2 = nh.subscribe("lidar_utility_settings", 1, settings_cb);
	// Create a ROS publisher for the output point cloud

	vis_pub = nh.advertise<visualization_msgs::MarkerArray>( pTopic+"_visualized", 0 );

	//ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	//pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic+"_points", 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	// Spin
	ros::spin ();
}


