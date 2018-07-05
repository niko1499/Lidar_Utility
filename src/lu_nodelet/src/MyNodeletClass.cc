#include "ros/ros.h"
#include "nodelet/nodelet.h"

namespace ns1
{

  class @(NodeletClass) : public nodelet::Nodelet
  {
  public:
  @(NodeletClass)();

  private:
  virtual void onInit(){
  nh = getNodeHandle();
  private_nh = getPrivateNodeHandle();
  timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(& @(NodeletClass)::timerCb, this, _1));
  sub_ = nh.subscribe("incoming_chatter", 10, boost::bind(& @(NodeletClass)::messageCb, this, _1));
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
  new_message.data = message.data + " fizz buzz";
  pub_.publish(new_message);

  // we can't modify any messages after they've been published, unless we want our subscribers to get VERY confused
  // new_message.data = "can't do this!";
   }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
  };

} // namespace @(namespace)

PLUGINLIB_DECLARE_CLASS(@(package), @(NodeletClass), @(namespace)::@(NamespaceClass), nodelet::Nodelet);



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
