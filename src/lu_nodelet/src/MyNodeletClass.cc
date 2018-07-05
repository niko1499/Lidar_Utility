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
}
