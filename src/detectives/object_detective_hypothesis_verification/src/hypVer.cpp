//ROS:
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <lidar_utility_msgs/lidarUtilitySettings.h>
#include <lidar_utility_msgs/roadInfo.h>
#include <lidar_utility_msgs/objectInfo.h>
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
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>

//#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);
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
ros::Publisher msg_pub;

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
static	int markerID=0;
static std::string frame_id("base_link");
static std::string forwardAxis("x");
static float personSize=100;
static float bikeSize=150;
static float carSize=300;
static float truckSize=900;
static float personScaleX=.75;
static float personScaleY=.75;
static float personScaleZ=1.25;
static float bikeScaleX=.75;
static float bikeScaleY=1.9;
static float bikeScaleZ=1.25;
static float carScaleX=2.3;
static float carScaleY=6;
static float carScaleZ=2;
static float truckScaleX=3;
static float truckScaleY=10;
static float truckScaleZ=3;
class XYZ{
	public:
		float x,y,z;
};

void settings_cb (const lidar_utility_msgs::lidarUtilitySettings& data)
{
	canContinue=data.permissionToContinue;
	scale1 = data.donScale1;
	scale2 = data.donScale2;
	threshold = data.donThreshold;
	segradius = data.donSegradius;
	setMinClusterSize_setting=data.objDetectDoNMinClusterSize;
	setMaxClusterSize_setting=data.objDetectDoNMaxClusterSize;
	frame_id=data.frame_id;
	personSize=data.personSize;
	bikeSize=data.bikeSize;
	carSize=data.carSize;
	truckSize=data.truckSize;
	personScaleX=data.personScale[0];
	personScaleY=data.personScale[1];
	personScaleZ=data.personScale[2];
	bikeScaleX=data.bikeScale[0];
	bikeScaleY=data.bikeScale[1];
	bikeScaleZ=data.bikeScale[2];
	carScaleX=data.carScale[0];
	carScaleY=data.carScale[1];
	carScaleZ=data.carScale[2];
	truckScaleX=data.truckScale[0];
	truckScaleY=data.truckScale[1];
	truckScaleZ=data.truckScale[2];
	forwardAxis=data.forwardAxis;
	
model_ss_= data.HVmodel_ss;
scene_ss_ = data.HVscene_ss;
rf_rad_ = data.HVrf_rad;
descr_rad_= data.HVdescr_rad;
cg_size_ = data.HVcg_size;
cg_thresh_= data.HVcg_thresh;	
	ROS_INFO("Settings are set");
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

visualization_msgs::Marker markerBuilder(int id){
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::DELETE;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0;
	marker.scale.y = 0;
	marker.scale.z = 0;
	marker.color.a = 0; // Don't forget to set the alpha!
	marker.color.r = 0;
	marker.color.g = 0;
	marker.color.b = 0;
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	return marker;
}
visualization_msgs::Marker markerBuilder(int id,XYZ loc,XYZ scale,XYZ min,XYZ max, float headding,int size){
	float xLoc=loc.x;
	float yLoc=loc.y;
	float zLoc=loc.z;	
	float xScale=scale.x;
	float yScale=scale.y;
	float zScale=scale.z;

	float r,g,b;
	float alpha=.5;
	float xScaleResult,yScaleResult,zScaleResult;
	float distMultiplier =  sqrt((xLoc*xLoc)+(yLoc*yLoc))-1.25;
	float constantMultiplier=.025;
	float locMultiplier;
	std::string type;
	if(yLoc>0){
		yLoc=yLoc+.75;
	}else{
		yLoc=yLoc-.75;
	}
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
	float sizeResult=size*distMultiplier*distMultiplier*constantMultiplier*locMultiplier;
	//size=800;
	printf("Modified size: %f \n",sizeResult);
	for(int i=0; i<4;i++){
		if(sizeResult<=personSize){//person: green
			r=0.0;
			g=1.0;
			b=0.0;
			xScaleResult=personScaleX;
			yScaleResult=personScaleY;
			zScaleResult=personScaleZ;
			type="person";
		}else if(sizeResult<=bikeSize){//bike||motorcycle: blue/green
			r=0.0;
			g=1.0;
			b=1.0;
			xScaleResult=bikeScaleX;
			yScaleResult=bikeScaleY;
			zScaleResult=bikeScaleZ;
			type="bike";
		}else if(sizeResult<=carSize){//car: blue
			r=0.0;
			g=0.0;
			b=1.0;
			xScaleResult=carScaleX;
			yScaleResult=carScaleY;
			zScaleResult=carScaleZ;
			type="car";
		}else if(sizeResult<=(carSize+((truckSize-carSize)/2))){//large car: blue/red
			r=1.0;
			g=0.0;
			b=1.0;
			xScaleResult=carScaleX+.1;
			yScaleResult=carScaleY+.2;
			zScaleResult=carScaleZ+.6;
			type="car";
		}else if(sizeResult<=truckSize){//truck||large vehicle: red
			r=1.0; 
			g=0.0;
			b=0.0;
			xScaleResult=truckScaleX;
			yScaleResult=truckScaleY;
			zScaleResult=truckScaleZ;
			type="truck";
		}else{//type unknown: white
			r=1.0;
			g=1.0;
			b=1.0;
			xScaleResult=5;
			yScaleResult=5;
			zScaleResult=3;
		}
		if(xScaleResult<=xScale){
			sizeResult=sizeResult+(sizeResult/2);
		}
		if((yLoc+(yScaleResult/2))<max.y){
			yLoc=yLoc+1;
		}

	}
	zLoc=zMidRoad+(zScaleResult/2);//set z same

	//create msg
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
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
	marker.text = type;
	//marker.lifetime = 1.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	


	return marker;
}

	double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	ros::Time begin = ros::Time::now();
	if(canContinue){


double res = 0.0;
	


	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

	//
	//  Load clouds
	//
	
pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2

		pcl::PointCloud<PointType>::Ptr in_cloud (new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);

		std::vector<int> indicies;
		pcl::fromPCLPointCloud2(pcl_pc2, *in_cloud);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		pcl::removeNaNFromPointCloud(*in_cloud, *scene, indicies);

		visualization_msgs::MarkerArray markerArray;
		visualization_msgs::Marker marker;

		scene->is_dense = false;

model_filename_="~/Lidar_Utility/PCL_testspace/manipulatePCD/build/merged6.pcd";

if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
	}

/* DELETE ME
	if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		showHelp (argv[0]);
		return (-1);
	}
	if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		showHelp (argv[0]);
		return (-1);
	}
*/
	//
	//  Set up resolution invariance
	//
	if (use_cloud_resolution_)
	{
		float resolution = static_cast<float> (computeCloudResolution (model));
		if (resolution != 0.0f)
		{
			model_ss_   *= resolution;
			scene_ss_   *= resolution;
			rf_rad_     *= resolution;
			descr_rad_  *= resolution;
			cg_size_    *= resolution;
		}

		std::cout << "Model resolution:       " << resolution << std::endl;
		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
	}

	//
	//  Compute Normals
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);
	//
	//  Downsample Clouds to Extract keypoints
	//

	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	//uniform_sampling.filter (*model_keypoints);
	pcl::PointCloud<int> keypointIndices1;
	uniform_sampling.compute(keypointIndices1);
	pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);
	std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;


	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	//uniform_sampling.filter (*scene_keypoints);
	pcl::PointCloud<int> keypointIndices2;
	uniform_sampling.compute(keypointIndices2);
	pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);
	std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


	//
	//  Compute Descriptor for keypoints
	//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch (descr_rad_);

	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);

	descr_est.setInputCloud (scene_keypoints);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < scene_descriptors->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_)
	{
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
		pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles (true);
		rf_est.setRadiusSearch (rf_rad_);

		rf_est.setInputCloud (model_keypoints);
		rf_est.setInputNormals (model_normals);
		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (scene_keypoints);
		rf_est.setInputNormals (scene_normals);
		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize (cg_size_);
		clusterer.setHoughThreshold (cg_thresh_);
		clusterer.setUseInterpolation (true);
		clusterer.setUseDistanceWeight (false);

		clusterer.setInputCloud (model_keypoints);
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (scene_keypoints);
		clusterer.setSceneRf (scene_rf);
		clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);
	}
	else // Using GeometricConsistency
	{
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (model_keypoints);
		gc_clusterer.setSceneCloud (scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}



	//
	//  Output results
	//

	std::cout << "Model instances found: " << rototranslations.size () << std::endl;
	for (size_t i = 0; i < rototranslations.size (); ++i)
	{

		//remove xy rotation
		rototranslations[i].block<3,3>(0, 0)(0,0)=1;
		rototranslations[i].block<3,3>(0, 0)(0,1)=0;
		rototranslations[i].block<3,3>(0, 0)(0,2)=0;
		rototranslations[i].block<3,3>(0, 0)(1,0)=0;
		rototranslations[i].block<3,3>(0, 0)(1,1)=1;
		rototranslations[i].block<3,3>(0, 0)(1,2)=0;
		rototranslations[i].block<3,3>(0, 0)(2,0)=0;
		rototranslations[i].block<3,3>(0, 0)(2,1)=0;
		rototranslations[i].block<3,3>(0, 0)(2,2)=1;


		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

		printf ("\n");
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		printf ("\n");
		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}

	//
	//  Visualization
	//
	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	viewer.addPointCloud (scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	if (show_correspondences_ || show_keypoints_)
	{
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
		viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}

	for (size_t i = 0; i < rototranslations.size (); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

		if (show_correspondences_)
		{
			for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	}

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}

	//return (0);

}
	ros::Time end =ros::Time::now();
	float duration=end.toSec() - begin.toSec();
	ROS_INFO("%s: Out of callback. Duration: %f",nodeName.c_str(),duration);
	//ROS_INFO(duration);
}

	int
main (int argc, char** argv)
{	
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("object_points1");
	const std::string defaultSubscriber2("plane_segmented_msg");
	const std::string defaultPublisher("objects_");
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

	vis_pub = nh.advertise<visualization_msgs::MarkerArray>( pTopic+"_visualized", 0 );

	//ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	//pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic+"_points", 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	msg_pub = nh.advertise<lidar_utility_msgs::objectInfo> (pTopic+"_msg", 1);

	ros::Subscriber sub2 = nh.subscribe(sTopic2.c_str(), 1, road_cb);
	ros::Subscriber sub3 = nh.subscribe("lidar_utility_settings", 1, settings_cb);

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


