//ROS:
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <lidar_utility_msgs/lidarUtilitySettings.h>
//C
#include <string>
//PCL:Base
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
//PCL:common
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
//PCL:DON
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/impl/point_types.hpp>


#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
static int mode =1;//fix this later
static std::string nodeName("object_detective");

//This node subscribes to a PointCloud2 topic, searches for the road, and publishes xxx. 

ros::Publisher pc2_pub;
ros::Publisher vis_pub;
ros::Publisher cl0_pub;
ros::Publisher cl1_pub;
ros::Publisher cl2_pub;
ros::Publisher cl3_pub;
ros::Publisher cl4_pub;
ros::Publisher cl5_pub;
ros::Publisher cl6_pub;
ros::Publisher cl7_pub;
ros::Publisher cl8_pub;
ros::Publisher cl9_pub;


//settings 
/*
static float leaf_setting=0.01;
static float setMaxIterations_setting=100;
static float setDistanceThreshold_setting=.02;
static float setClusterTolerance_setting=.325;
static float setMinClusterSize_setting=100;
static float setMaxClusterSize_setting=40000;
*/

  static double scale1=10;//The smallest scale to use in the DoN filter.
  static double scale2=20;  //The largest scale to use in the DoN filter.
  static double threshold=.1;  //The minimum DoN magnitude to threshold by
  static double segradius=.5;//segment scene into clusters with given distance tolerance using euclidean clustering

void settings_cb (const lidar_utility_msgs::lidarUtilitySettings& data)
{
	scale1 = data.donScale1;
	scale2 = data.donScale2;
	threshold = data.donThreshold;
	segradius = data.donSegradius;
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
	ROS_INFO("%s ObjectDetective: In Callback", nodeName.c_str());
	//NEW CONVERSION
	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	

//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//create
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//note pcl:
//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromPCLPointCloud2(pcl_pc2,*cloud);//convert PCLPC2 to PCLXYZ
	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker marker;
	//euclidian cluster extraxion
	
//these were moved here for scope of mode
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCDWriter writer;
//___BEGIN___DON

//!!!!!!!!!!!!!!!!!!!!!!!!HEREds



  // Load cloud in blob format
  //pcl::PCLPointCloud2 blob;
  //pcl::io::loadPCDFile (infile.c_str (), blob);

  //pcl::fromPCLPointCloud2 (blob, *cloud);

  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    ROS_INFO("Error: Large scale must be > small scale!");
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  
//   NOTE: setting viewpoint is very important, so that we can ensure
//   normals are all pointed in the same direction!

  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  ROS_INFO( "Calculating normals for scale...");
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  ROS_INFO( "Calculating normals for scale..." );
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointNormal>(*cloud, *doncloud);

  ROS_INFO( "Calculating DoN... ");
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  ROS_INFO( "Filtering out DoN mag <= %f ",threshold );

  // Build the condition for filtering
  pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<pcl::PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                               new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  ROS_INFO( "Filtered Pointcloud: %i data points.",doncloud->points.size());

  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  ROS_INFO( "Clustering using EuclideanClusterExtraction with tolerance <= %f" ,segradius);

  pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;




    //Save cluster
//
//    ROS_INFO( "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
  //  stringstream ss;
    //ss << "don_cluster_" << j << ".pcd";
  //  writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);

  
//publish mini clusters
sensor_msgs::PointCloud2 output;//create output container
pcl::PCLPointCloud2 temp_output;//create PCLPC2
pcl::toPCLPointCloud2(*doncloud,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
pc2_pub.publish (output);// Publish the data.



}



//END DON_____
	
	if(mode==1){
		
	}
	//publih
	vis_pub.publish(markerArray);

	
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


//cluster publishers
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl0", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl1", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl2", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl3", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl4", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl5", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl6", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl7", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl8", 1);
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl9", 1);
	// Spin
	ros::spin ();
}


