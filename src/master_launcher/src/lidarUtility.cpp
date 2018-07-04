#include <ros/ros.h>
#include <ros/master.h>

// PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/eigen.h>

#include <lidar_utility_msgs/lidarUtilitySettings.h>

#include <pcl/registration/boost.h>
#include <pcl/correspondence.h>
#include <pcl/common/common.h>

#include <pcl/common/projection_matrix.h>
//#include <pcl/common/impl/transforms.hpp>
#include <pcl/common/transforms.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>                  //allows us to use pcl::transformPointCloud function
#include <pcl/visualization/pcl_visualizer.h>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "--------------------------------------------------------------------------------------\n"

static int mode;

ros::Publisher pc2_pub;
ros::Publisher msg_pub;
	void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//Callback to echo any reciveed point cloud topics
	printf(COLOR_BLUE BAR COLOR_RST);
	ROS_INFO("--Lidar Utility: In Callback--");

	// Create a container for the data.
	sensor_msgs::PointCloud2 output;

	if(mode==1){

		//http://www.mamicode.com/info-detail-1597219.html
		//http://docs.pointclouds.org/trunk/namespacepcl.html
		output = *input;
	}else if (mode==3){

		pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointNormal> ());

		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*input,pcl_pc2);//convert ROSPC2 to PCLPC2

		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ

		pcl::PointCloud<pcl::PointNormal> temp_cloud2;
		float theta =M_PI/2;
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		transform(0,0)= cos(theta);
		transform(0,1)= -sin(theta);
		transform(1,0)= sin(theta);
		transform(1,1)= cos(theta);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ> ());

		printf ("Transform: Matrix4f\n");
		std::cout << transform << std::endl;

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointNormal> ());
		pcl::transformPointCloudWithNormals(*temp_cloud,*cloud_transformed,transform);


		pcl::PCLPointCloud2 temp_output;//create PCLPC2
		pcl::toPCLPointCloud2(*cloud_transformed,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
		pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type

	}

	// Publish the data.
	pc2_pub.publish (output);
}

void timer_cb (const ros::TimerEvent& event){
	//custom msg
	lidar_utility_msgs::lidarUtilitySettings msg;
	//all distances in meters
	msg.headerstamp = ros::Time::now();
	msg.permissionToContinue=true;
	msg.header.frame_id = "/world";


	if(mode==1){//setting for velodyne .pcd
		msg.outlierRemovalMeanK=75;//The number of neighbors to analize for each point//was 50
		msg.frame_id="base_link";
		msg.forwardAxis="y";		
		msg.outlierRemovalStdDev=.4;//STD DEV multiplier//was 1
		msg.outlierRemovalSearchRadius=.25;//Search radius for radial outlier removal//was.8
		msg.outlierRemovalMinNeighborsInRadius=10;//Minum number of neighbors a point must have to be kept in radial outlier removal
		msg.passThroughRoadMin=-4;
		msg.passThroughRoadMax=-1.2;
		msg.passThroughObjectMin=-1.7;
		msg.passThroughObjectMax=1.25;
		msg.passThroughBoxMargin=.5;
		msg.passThroughRangeBoost=0;
		msg.downSampleLeafSize = 0.01;//leaf size for downsampling
		msg.downSampleLeafSize_B = 0.01;//leaf size for downsampling
		msg.intensityMinimum_A = 00;//placeholder
		msg.intensityMaximum_A = 00;//placeholder
		msg.intensityMinimum_B = 00;//placeholder
		msg.intensityMaximum_B = 00;//placeholder
		msg.planeSegMaxIterations = 00;
		msg.objDetectMaxIterations = 100;
		msg.objDetectDistThresh = 0.02;
		msg.objDetectClusterTolerance = 0.325;//a
		msg.objDetectMinClusterSize = 100;
		msg.objDetectMaxClusterSize = 40000;
		msg.donScale1=.3;
		msg.donScale2=20;
		msg.donThreshold=0.05;
		msg.donSegradius=1;//Segmentation radiius for DoN
		msg.objDetectDoNMinClusterSize = 100;//Min cluster size for DoN clusters
		msg.objDetectDoNMaxClusterSize = 40000;//Max cluster size for DoN clusters
		msg.planeSegThreshold=.2;
		msg.personSize=100;
		msg.bikeSize=150;
		msg.carSize=300;
		msg.truckSize=900;
		msg.personScale[0]=.75;
		msg.personScale[1]=.75;
		msg.personScale[2]=1.25;
		msg.bikeScale[0]=.75;
		msg.bikeScale[1]=1.9;
		msg.bikeScale[2]=1.25;
		msg.carScale[0]=2.3;
		msg.carScale[1]=6;
		msg.carScale[2]=2;
		msg.truckScale[0]=3;
		msg.truckScale[1]=10;
		msg.truckScale[2]=3;
		msg.HVmodel_ss=.2;
		msg.HVscene_ss=.6;
		msg.HVrf_rad=.545;
		msg.HVdescr_rad=.22;
		msg.HVcg_size=3;
		msg.HVcg_thresh=2;


	}else if (mode==2){//settings for rslidar

	}else if (mode==3){//settings for Pandar40
		msg.frame_id="pandar";
		msg.forwardAxis="x";
		msg.outlierRemovalMeanK=75;//The number of neighbors to analize for each point//was 50
		msg.outlierRemovalStdDev=2;//STD DEV multiplier//was 1
		msg.outlierRemovalSearchRadius=.25;//Search radius for radial outlier removal//was.8
		msg.outlierRemovalMinNeighborsInRadius=10;//Minum number of neighbors a point must have to be kept in radial outlier removal
		msg.passThroughRoadMin=-4;
		msg.passThroughRoadMax=-1.2;
		msg.passThroughObjectMin=-1.7;
		msg.passThroughObjectMax=1.25;
		msg.passThroughBoxMargin=.5;
		msg.passThroughRangeBoost=5;
		msg.downSampleLeafSize = 0.01;//leaf size for downsampling
		msg.downSampleLeafSize_B = 0.01;//leaf size for downsampling
		msg.intensityMinimum_A = 00;//placeholder
		msg.intensityMaximum_A = 00;//placeholder
		msg.intensityMinimum_B = 00;//placeholder
		msg.intensityMaximum_B = 00;//placeholder
		msg.planeSegMaxIterations = 00;
		msg.objDetectMaxIterations = 100;
		msg.objDetectDistThresh = 0.02;
		msg.objDetectClusterTolerance = 0.325;//a
		msg.objDetectMinClusterSize = 100;
		msg.objDetectMaxClusterSize = 40000;
		msg.donScale1=.3;
		msg.donScale2=20;
		msg.donThreshold=0.1;
		msg.donSegradius=1;//Segmentation radiius for DoN
		msg.objDetectDoNMinClusterSize = 200;//Min cluster size for DoN clusters
		msg.objDetectDoNMaxClusterSize = 1500;//Max cluster size for DoN clusters
		msg.planeSegThreshold=.05;
		msg.personSize=1000;
		msg.bikeSize=1500;
		msg.carSize=2300;
		msg.truckSize=3000;
		msg.personScale[0]=.75;
		msg.personScale[1]=.75;
		msg.personScale[2]=1.25;
		msg.bikeScale[0]=.75;
		msg.bikeScale[1]=1.9;
		msg.bikeScale[2]=1.25;
		msg.carScale[0]=2.3;
		msg.carScale[1]=6;
		msg.carScale[2]=2;
		msg.truckScale[0]=3;
		msg.truckScale[1]=10;
		msg.truckScale[2]=3;
		msg.HVmodel_ss=.2;
		msg.HVscene_ss=.6;
		msg.HVrf_rad=.545;
		msg.HVdescr_rad=.22;
		msg.HVcg_size=3;
		msg.HVcg_thresh=2;

	}else if (mode==4){

	}else{

	}
	msg_pub.publish(msg);
}

	int
main (int argc, char** argv)
{
	//ROS_INFO("Waiting to ensure drivers are ready...");
	//sleep(1500);
	//initialize default topics for subscribing and publishing
	const std::string defaultSubscriber("cloud_pcd");
	const std::string defaultPublisher("lidar_utility");

	std::string nodeName("lidar_utility");//temp name to initialize with

	// Initialize ROS
	ros::init (argc, argv, nodeName);
	ros::NodeHandle nh;

	nodeName = ros::this_node::getName();//update node name

	//Create Parameters on the new name
	const std::string subscriberParamName(nodeName + "/subscriber");
	const std::string publisherParamName(nodeName + "/publisher");

	//Create constants for expected topics
	const std::string potentialSubscription1("/cloud_pcd");
	const std::string potentialSubscription2("/rslidar_points");
	const std::string potentialSubscription3("/pandar_points");

	printf(COLOR_BLUE BAR COLOR_RST);
	ROS_INFO("LIDAR UTILITY RUNNING");
	ROS_INFO("Node Name: %s",nodeName.c_str());

	//Create variables that control the topic names
	std::string sTopic;
	std::string pTopic;


	//Check if the user specified a subscription topic
	if(nh.hasParam(subscriberParamName)){
		nh.getParam(subscriberParamName,sTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
	}else{
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set. Searching advertised topics for appropriate subscribers...",nodeName.c_str());
		ros::master::V_TopicInfo master_topics;
		ros::master::getTopics(master_topics);


		std::string currentTopic;
		for(ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++){
			const ros::master::TopicInfo& info= *it;
			std::cout <<"topic_"<<it - master_topics.begin()<<": " <<info.name<<std::endl;

			currentTopic=info.name;
			if(currentTopic==potentialSubscription1)
			{
				ROS_INFO("%s: Topic found! Subscribing to: %s",nodeName.c_str(),currentTopic.c_str());
				sTopic=info.name;
				mode=1;
				break;

			}else if(info.name==potentialSubscription2){
				ROS_INFO("%s: Topic found! Subscribing to: %s",nodeName.c_str(),currentTopic.c_str());
				sTopic=info.name;
				mode=2;
				break;

			}else if(info.name==potentialSubscription3){
				ROS_INFO("%s: Topic found! Subscribing to: %s",nodeName.c_str(),currentTopic.c_str());
				sTopic=info.name;
				mode=3;
				break;

			}else{
				ROS_INFO("%s: Searching... If no topic if found default is: %s",nodeName.c_str(), defaultSubscriber.c_str());
				mode=4;
				sTopic=defaultSubscriber;
			}

		}//end of for loop
	}//end of subscription check

	//Check if the user specified a publishing topic
	if(nh.hasParam(publisherParamName)){
		nh.getParam(publisherParamName,pTopic);
		printf(COLOR_GREEN BAR COLOR_RST);
		ROS_INFO("%s: A param has been set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
	}else{
		pTopic=defaultPublisher;//set to default if not specified
		printf(COLOR_RED BAR COLOR_RST);
		ROS_INFO("%s: No param set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
	}

	//Clears the assigned parameters Without this default will never be used but instead the last spefified topic
	nh.deleteParam(subscriberParamName);
	nh.deleteParam(publisherParamName);
	//nh.deleteParam(modeParamName);
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);
	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic+"_points", 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

	msg_pub=nh.advertise<lidar_utility_msgs::lidarUtilitySettings>(pTopic+"_settings",1);

	ros::Timer timer= nh.createTimer(ros::Duration(1.5),timer_cb,true);

	//NTS: ADD BUFFER SIZE MODE - NIKO 5/21/18 


	//ROS_INFO("HERE XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXspin");
	//r.sleep();
	//ros::spinOnce();
	//}
	// Spin
	ros::spin ();
	}
