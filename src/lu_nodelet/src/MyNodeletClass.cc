// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "MyNodeletClass.h"
// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(my_nodelet::MyNodeletClass, nodelet::Nodelet)

namespace my_nodelet
{
    void MyNodeletClass::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
    }
}
