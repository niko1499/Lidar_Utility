//ROS:
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <lidar_utility_msgs/lidarUtilitySettings.h>
#include <lidar_utility_msgs/roadInfo.h>
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
#include <pcl/features/don.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/project_inliers.h>
// PCL specific includes
#include <pcl/filters/passthrough.h>

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


//Settings. Note will get overwritten by setting_cb 
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
static double setMinClusterSize_setting=50;
static double setMaxClusterSize_setting=40000;
static bool canContinue=false;
static 	float xMinRoad, xMaxRoad, yMinRoad, yMaxRoad, zMinRoad, zMaxRoad;
static int lastMarkerMax=0;

void settings_cb (const lidar_utility_msgs::lidarUtilitySettings& data)
{
	canContinue=data.permissionToContinue;
	scale1 = data.donScale1;
	scale2 = data.donScale2;
	threshold = data.donThreshold;
	segradius = data.donSegradius;
	setMinClusterSize_setting=data.objDetectDoNMinClusterSize;
	setMaxClusterSize_setting=data.objDetectDoNMaxClusterSize;
}
void road_cb (const lidar_utility_msgs::roadInfo& data)
{
	xMinRoad = data.xMin;
	xMaxRoad = data.xMax;
	yMinRoad = data.yMin;
	yMaxRoad = data.yMax;
	zMinRoad = data.zMin;
	zMaxRoad = data.zMax;	
}

visualization_msgs::Marker markerBuilder(int id,float xLoc,float yLoc, float zLoc, float xScale, float yScale, float zScale,int size){
	float r,g,b;
	float alpha=.75;
	size=800;
	if(size<=100){//person: green
		r=0.0;
		g=1.0;
		b=0.0;
	}else if(size<=200){//bike||motorcycle: blue/green
		r=0.0;
		g=1.0;
		b=1.0;
	}else if(size<=300){//car: blue
		r=0.0;
		g=0.0;
		b=1.0;
	}else if(size<=400){//large car: blue/red
		r=1.0;
		g=0.0;
		b=1.0;
	}else if(size<=500){//truck||large vehicle: red
		r=1.0;
		g=0.0;
		b=0.0;
		alpha=.85;
	}else{//type unknown: white
		r=1.0;
		g=1.0;
		b=1.0;
	}
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = id;
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


visualization_msgs::Marker markerBuilder(int id,float xLoc,float yLoc, float zLoc,float xScale,float yScale,float zScale, float headding,int size){
	float r,g,b;
	float alpha=.5;
	float xScaleResult,yScaleResult,zScaleResult;
	float distMultiplier =  sqrt((xLoc*xLoc)+(yLoc*yLoc))-1.25;
	float constantMultiplier=.025;
	float locMultiplier;

	if(abs(xLoc)<=2.35){//close to center of x
	locMultiplier=1.25;
	if(yLoc<0){
	yLoc=yLoc-1;
	}else{
	yLoc=yLoc+1;
	}
	}else if (abs(xLoc)<=1.35){//very close to center of x
	locMultiplier=2;
	if(yLoc<0){
	yLoc=yLoc-1.9;
	}else{
	yLoc=yLoc+1.9;
	}
	}else if (abs(yLoc)<=1.5){//close to center of y
	locMultiplier=1.3;
	if(xLoc<0){
	xLoc=xLoc-.25;
	}else{
	xLoc=xLoc+.25;
	}
	}else if (abs(yLoc)<=.75){//very close to center of y
	locMultiplier=1.6;
	if(xLoc<0){
	xLoc=xLoc-.5;
	}else{
	xLoc=xLoc+.5;
	}
	}else{
	locMultiplier=1;
	}

float zMidRoad = zMaxRoad -((zMaxRoad-zMinRoad)/2);
	size=size*distMultiplier*distMultiplier*constantMultiplier*locMultiplier;
	//size=800;

for(int i=0; i<3;i++){
	if(size<=100){//person: green
		r=0.0;
		g=1.0;
		b=0.0;
		xScaleResult=.75;
		yScaleResult=.75;
		zScaleResult=1.25;
	}else if(size<=150){//bike||motorcycle: blue/green
		r=0.0;
		g=1.0;
		b=1.0;
		xScaleResult=.75;
		yScaleResult=1.9;
		zScaleResult=1.25;
	}else if(size<=300){//car: blue
		r=0.0;
		g=0.0;
		b=1.0;
		xScaleResult=2.3;
		yScaleResult=6;
		zScaleResult=2;
	}else if(size<=600){//large car: blue/red
		r=1.0;
		g=0.0;
		b=1.0;
		xScaleResult=2.5;
		yScaleResult=8;
		zScaleResult=2.5;
	}else if(size<=700){//truck||large vehicle: red
		r=1.0;
		g=0.0;
		b=0.0;
		xScaleResult=3;
		yScaleResult=10;
		zScaleResult=3;
	}else{//type unknown: white
		r=1.0;
		g=1.0;
		b=1.0;
		xScaleResult=5;
		yScaleResult=5;
		zScaleResult=3;
	}
	if(xScaleResult<=xScale){
	size=size+(size/2);
}

}
	zLoc=zMidRoad+(zScaleResult/2);//set z same

//create msg
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = xLoc;
	marker.pose.position.y = yLoc;
	marker.pose.position.z = zLoc;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = xScaleResult;
	marker.scale.y = yScaleResult;
	marker.scale.z = zScaleResult;
	marker.color.a =alpha; // Don't forget to set the alpha!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.text = "test";
	//marker.lifetime = 1.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	return marker;
}
	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	if(canContinue){
		
pcl::PCDWriter writer;
		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2

		pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		std::vector<int> indicies;
		pcl::fromPCLPointCloud2(pcl_pc2, *in_cloud);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

		visualization_msgs::MarkerArray markerArray;
		visualization_msgs::Marker marker;

		cloud->is_dense = false;

		// Create a search tree, use KDTreee for non-organized data.
		pcl::search::Search<pcl::PointXYZ>::Ptr tree;
		if (cloud->isOrganized ())
		{
			tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
		}
		else
		{
			tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
		}

		// Set the input pointcloud for the search tree
		tree->setInputCloud (cloud);

		if (scale1 >= scale2)
		{
			ROS_INFO("Error: Large scale must be > small scale!");
			exit (EXIT_FAILURE);
		}

		pcl::removeNaNFromPointCloud(*cloud, *cloud, indicies);

		// Compute normals using both small and large scales at each point
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
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
		pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *doncloud);

		ROS_INFO( "Calculating DoN... ");
		// Create DoN operator
		pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
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
		//pcl::PCDWriter writer;//moved for scope
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
		ec.setMinClusterSize (setMinClusterSize_setting);//SETTING old was 100000
		ec.setMaxClusterSize (setMaxClusterSize_setting);//SETTING old was 100000
		ec.setSearchMethod (segtree);
		ec.setInputCloud (doncloud);
		ec.extract (cluster_indices);
		int cloudNum=0;
		int j = 0;

for(int k=0;k<9;k++){
markerArray.markers.push_back(markerBuilder(k,0,0,0,0,0,0,0));
}

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

			std::cout << "ObjDetDoN: PointCloud representing the Cluster: " << cloud_cluster_don->points.size ()<< " data points."<< std::endl;

			//publish mini clusters
			sensor_msgs::PointCloud2 output;//create output container
			pcl::PCLPointCloud2 temp_output;//create PCLPC2
		
			pcl::PointCloud<pcl::PointXYZ>::Ptr temX (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

			//Cut a box
			pcl::PointNormal pMin,pMax;
			pcl::getMinMax3D (*cloud_cluster_don,pMin,pMax);
			float xMaxf=pMax.x;
			float xMinf=pMin.x;
			float yMaxf=pMax.y;
			float yMinf=pMin.y;
			float zMaxf=pMax.z;
			float zMinf=pMin.z;

			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ

			pcl::PointIndices::Ptr indices_x (new pcl::PointIndices);
			pcl::PointIndices::Ptr indices_xy (new pcl::PointIndices);

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PassThrough<pcl::PointXYZ> ptfilter (true); 
			ptfilter.setInputCloud (temp_cloud);
			ptfilter.setFilterFieldName ("x");
			ptfilter.setFilterLimits (xMinf, xMaxf);
			ptfilter.filter (*cloud_filtered_x);

			ptfilter.setInputCloud(cloud_filtered_x);
			ptfilter.setFilterFieldName ("y");
			ptfilter.setFilterLimits (yMinf,yMaxf);
			ptfilter.filter (*cloud_filtered_xy);

			ptfilter.setInputCloud(cloud_filtered_xy);
			ptfilter.setFilterFieldName ("z");
			ptfilter.setFilterLimits (zMinf,zMaxf);//SETTING
			ptfilter.setNegative (false);
			ptfilter.filter (*cloud_filtered_xyz);

			pcl::toPCLPointCloud2(*cloud_filtered_xyz,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
			pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type

//box

float xScale=abs(pMax.x-pMin.x);
float yScale=abs(pMax.y-pMin.y);
float zScale=(pMax.z-pMin.z);

float xLoc= ((pMax.x-pMin.x)/2)+pMin.x;
float yLoc= ((pMax.y-pMin.y)/2)+pMin.y;
float zLoc= ((pMax.z-pMin.z)/2)+pMin.z;
//Discard bad clusters
if((zLoc-.1<zMaxRoad)){
ROS_INFO("Discarding Cluster: Too low");
}else if(zScale<.4){
ROS_INFO("Discarding Cluster: Too short");
}else{

//NEED TO ADD SOMETHING TO CLEAR MARKER ARRAY.
/*
if(j=0){
for(int k=0;k<9;k++){
markerArray.markers.push_back(markerBuilder(k,0,0,0,0,0,0,0));
}
}
*/


//rviz_visual_tools::RvizVisualTools.deleteAllMarkers();
//marker=markerBuilder(j,xLoc,yLoc,zLoc,xScale,yScale,zScale,cloud_cluster_don->width);
marker=markerBuilder(j,xLoc,yLoc,zLoc,xScale,yScale,zScale,0,cloud_cluster_don->width);
			markerArray.markers.push_back(marker);

			switch(cloudNum){
				case 0:
					cl0_pub.publish (output);
					break;
				case 1:
					cl1_pub.publish (output);
					break;
				case 2:
					cl2_pub.publish (output);
					break;
				case 3:
					cl3_pub.publish (output);
					break;
				case 4:
					cl4_pub.publish (output);
					break;
				case 5:
					cl5_pub.publish (output);
					break;
				case 6:
					cl6_pub.publish (output);
					break;
				case 7:
					cl7_pub.publish (output);
					break;
				case 8:
					cl8_pub.publish (output);
					break;
				case 9:
					cl9_pub.publish (output);
					break;
				default:
					ROS_INFO("ERR: More clusters than available pc2 topics.");
					cloudNum=-1;
					break;
			}
			cloudNum++;
		}
		if(mode==1){
		}
		//publih
//for(cloudNum;cloudNum<=lastMarkerMax;cloudNum++){
//markerArray.markers.push_back(markerBuilder(cloudNum,0,0,0,0,0,0,0));
//}
		vis_pub.publish(markerArray);
		ROS_INFO("%s: Out of callback",nodeName.c_str());
	lastMarkerMax=cloudNum;
	}
}
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

	ros::Subscriber sub3 = nh.subscribe("plane_segmented_msg", 1, road_cb);

	//cluster publishers
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl0", 1);
	cl1_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl1", 1);
	cl2_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl2", 1);
	cl3_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl3", 1);
	cl4_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl4", 1);
	cl5_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl5", 1);
	cl6_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl6", 1);
	cl7_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl7", 1);
	cl8_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl8", 1);
	cl9_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl9", 1);
	// Spin
	ros::spin ();
}


