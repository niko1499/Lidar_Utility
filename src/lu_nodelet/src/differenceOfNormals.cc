/* differencceOfNormals
 * Nikolas Gamarra -+- nxgamarra@gmail.com
 * Description: Filter and republish a cloud cut down to a smaller box in xyz coordinates using
 * predefined bounds or ones supplied by a custom msg.
 * Available modes: 1
 */
//C
#include <pluginlib/class_list_macros.h>
#include "nodelet/nodelet.h"
#include <string>
#include <iostream>
#include "printUtil.h"
//ROS:
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <lidar_utility_msgs/roadInfo.h>
#include <lidar_utility_msgs/objectInfo.h>
#include "std_msgs/String.h"
#include <lidar_utility_msgs/roadInfo.h>
//PCL specific includes
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
//PCL local includes
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
//decleare globals
static int mode =1;
static std::string nodeName("differenceOfNormals");
static 	float xMinRoad, xMaxRoad, yMinRoad, yMaxRoad, zMinRoad, zMaxRoad;
static double scale1=10;//The smallest scale to use in the DoN filter.
static double scale2=20;  //The largest scale to use in the DoN filter.
static double threshold=.1;  //The minimum DoN magnitude to threshold by
static double segradius=.5;//segment scene into clusters with given distance tolerance using euclidean clustering
static double setMinClusterSize_setting=50;
static double setMaxClusterSize_setting=40000;
static std::string frame_id("base_link");
static int lastMarkerMax=0;
static int markerID=0;
//non setting
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
static bool canContinue=true;
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
//#include "differenceOfNormals.h" //structured without headerfile
namespace lu_nodelet
{
	class differenceOfNormals : public nodelet::Nodelet
	{
		public:
			void onInit()
			{
				ROS_INFO("Initializing nodelet...");

				ros::NodeHandle private_nh;//create node handles
				ros::NodeHandle nh;
				nh = getNodeHandle();
				private_nh = getPrivateNodeHandle();

				//update settings
				nh.getParam("settings/don_Scale1", scale1);
				nh.getParam("settings/don_Scale2", scale2);
				nh.getParam("settings/don_Threshold", threshold);
				nh.getParam("settings/don_Segradius", segradius);
				nh.getParam("settings/don_MinClusterSize", setMinClusterSize_setting);
				nh.getParam("settings/don_maxClusterSize", setMaxClusterSize_setting);
				nh.getParam("settings/frame_id", frame_id);
				
				//initialize default topics for subscribing and publishing
				const std::string defaultCloudSubscriber("cloud_in");
				const std::string defaultCloudPublisher("cloud_out");
				const std::string defaultMode("1");
				const std::string defaultMsgSubscriber("plane_segmented_msg");

				nodeName = getName();//Update nodelet name

				//set parameters on new name
				const std::string subscriberParamName(nodeName + "/subscriber");
				const std::string publisherParamName(nodeName + "/publisher");
				const std::string modeParamName(nodeName + "/mode");
				const std::string subscriberParamName2(nodeName + "/msgSubscriber");

				printf(COLOR_BLUE BAR COLOR_RST);//Print a blue bar
				ROS_INFO("Node Name: %s",nodeName.c_str());

				//Create variables that control the topic names
				std::string sTopic;
				std::string pTopic;
				std::string myMode;
				std::string sTopic2;

				if(nh.hasParam(subscriberParamName)){//Check if the user specified a subscription topic
					nh.getParam(subscriberParamName,sTopic);
					printf(COLOR_GREEN BAR COLOR_RST);
					ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
				}else{
					sTopic=defaultCloudSubscriber;//set to default if not specified
					printf(COLOR_RED BAR COLOR_RST);
					ROS_INFO("%s: No param set **%s**  \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
				}

				if(nh.hasParam(subscriberParamName2)){//Check if the user specified a subscription topic for msgs
					nh.getParam(subscriberParamName2,sTopic2);
					printf(COLOR_GREEN BAR COLOR_RST);
					ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
				}else{
					sTopic2=defaultMsgSubscriber;//set to default if not specified
					printf(COLOR_RED BAR COLOR_RST);
					ROS_INFO("%s: No param set **%s**  \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
				}

				if(nh.hasParam(publisherParamName)){//Check if the user specified a publishing topic
					printf(COLOR_GREEN BAR COLOR_RST);
					nh.getParam(publisherParamName,pTopic);
					ROS_INFO("%s: A param has been set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
				}else{printf(COLOR_RED BAR COLOR_RST);
					pTopic=defaultCloudPublisher;//set to default if not specified
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
				if(myMode=="1"||myMode=="r"||myMode=="road"){//interpret mode
					mode=1;
				}else if(myMode=="2"||myMode=="o"||myMode=="objects"){
					mode=2;
				}else if(myMode=="3"||myMode=="a"||myMode=="advObjects"){
					mode=3;
				}else if(myMode=="4"||myMode=="f"||myMode=="forward"){
					mode=4;
				}

				//set up subscribers and publishers
				pc2_sub = nh.subscribe(sTopic, 10,&differenceOfNormals::cloud_cb, this,ros::TransportHints().tcpNoDelay(true));//subscribe to point cloud

				msg_sub = nh.subscribe(sTopic2, 10,&differenceOfNormals::message_cb, this,ros::TransportHints().tcpNoDelay(true));//subscribe to msgs

				pc2_pub = private_nh.advertise<sensor_msgs::PointCloud2>(pTopic, 10);

			}

			void message_cb(const lidar_utility_msgs::roadInfo& data){//callback to store data from msgs locally
	xMinRoad = data.xMin;
	xMaxRoad = data.xMax;
	yMinRoad = data.yMin;
	yMaxRoad = data.yMax;
	zMinRoad = data.zMin;
	zMaxRoad = data.zMax;	
			}//msg callback

			void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){//callback to process cloud

			}//cloud callback
			//create subscribers and publishers
			ros::NodeHandle nh;
			ros::Subscriber pc2_sub;
			ros::Subscriber msg_sub;
			ros::Publisher pc2_pub;
	};//class
	PLUGINLIB_EXPORT_CLASS(lu_nodelet::differenceOfNormals, nodelet::Nodelet)
}//ns
