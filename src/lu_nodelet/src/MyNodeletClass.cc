#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include "nodelet/nodelet.h"
//#include "MyNodeletClass.h"
#include <string>
//ROS:
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <lidar_utility_msgs/lidarUtilitySettings.h>
#include <lidar_utility_msgs/roadInfo.h>
#include <lidar_utility_msgs/objectInfo.h>
#include "std_msgs/String.h"
//#include <pandar_pointcloud/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

namespace lu_nodelet
{

  class MyNodeletClass : public nodelet::Nodelet
  {
  public:
  MyNodeletClass();

  private:
  virtual void onInit(){
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  nh = getNodeHandle();
  private_nh = getPrivateNodeHandle();
  timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(& MyNodeletClass::timerCb, this, _1));
 // sub_ = nh.subscribe("incoming_chatter", 10, boost::bind(& MyNodeletClass::messageCb, this, _1));
 sub_ = nh.subscribe("pandar_points", 10,
                     &MyNodeletClass::messageCb, this,
                     ros::TransportHints().tcpNoDelay(true));
  pub_ = private_nh.advertise<std_msgs::String>("outgoing_chatter", 10);
  };

  void timerCb(const ros::TimerEvent& event){
  // Using timers is the preferred 'ROS way' to manual threading
  NODELET_INFO_STREAM("The time is now " << event.current_real);
  }

  // must use a ConstPtr callback to use zero-copy transport
  void messageCb(const std_msgs::StringConstPtr message){

  // can republish the old message no problem, since we're not modifying it
  pub_.publish(message);

  std_msgs::String new_message;
 // new_message.data = message.data + " fizz buzz";
  pub_.publish(new_message);

  // we can't modify any messages after they've been published, unless we want our subscribers to get VERY confused
  // new_message.data = "can't do this!";
   }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
  };

} // namespace @(namespace)

PLUGINLIB_DECLARE_CLASS(lu_nodelet, MyNodeletClass, lu_nodelet::MyNodeletClass, nodelet::Nodelet);



/*

// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(example_pkg::MyNodeletClass, nodelet::Nodelet)

namespace example_pkg
{
    void MyNodeletClass::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
    }
}*/
