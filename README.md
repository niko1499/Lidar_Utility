# Lidar_Utility
Nikolas Xarles Gamarra -+- nxgamarra@gmail.com -+- https://github.com/niko1499/Lidar_Utility
## Description 
This "Lidar_Utility" is a collection or ROS nodes that are useful for filtering and interpreting point cloud data. It depends on [ROS](http://wiki.ros.org/) and [PCL](http://pointclouds.org/documentation/). A basic understanding of ROS is recommended for being able to understand the nodes and topics that make this project work. This project can be used with any source of data that publishes a point cloud to a ROS PointCloud2 topic. 

## To download and compile the project:
```
cd ~/
git clone https://github.com/niko1499/Lidar_Utility.git
cd Lidar_Utility
source devel/setup.bash 
catkin_make
```
To install without any supplemental files or point cloud data use:
```
git clone -b PX2 https://github.com/niko1499/Lidar_Utility.git
```
## To Run the project
1. **Select the source of your lidar data and launch it (you may need to launch roscore first):**

- **rslidar:** Below is the roslaunch command to launch the [rslidar](http://www.robosense.ai/) driver that is included in this workspace. It has been modified slightly from its origional form. It is recommended you use the setIP script before launching the driver to configure your static IP. 
```
roslaunch rslidar_pointcloud rs_lidar_16.launch
```
- **Pandar40:** Below is the roslaunch command to launch the [Pandar40](http://www.hesaitech.com/en/index.html) driver that is included in this workspace. It has been modified slightly from its origional form. It is recommended you use the setIP script before launching the driver to configure your static IP. 
```
roslaunch pandar_pointcloud Pandar40_points.launch
```
- **.pcd file:** Below are a couple examples of how to use ROS to publish a .pcd as a ros PointCloud2 topic. See the pcd directory for more valid file names. 
_
```
rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/PointCloudData/pcd/velodyne1/2826laser.pcd .1
rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/PointCloudData/pcd/velodyne1/2321laser.pcd .1
```
- **rosbag:**
First cd into the bag directory. Then run rosbag. The -l loops the file. The -r specifies a rate multiplier. See the rosbag directory for more valid file names. *Note: Bag files only in local repo. Too large to upload to Git. 
```
cd ~/Lidar_Utility/PointCloudData/rosbag/SAIC_campus
rosbag play -l -r .3 veh3.bag
```
- **OTHER:**
Other sources should also work with the Lidar_Utility as long as they publish PointCloud2
data as a ROS topic. See the next step for how to specify a subscription topic a launch time. Note you may need to change setting inside lidarUtility.cpp in order to get the program to work with the new data set.

2. **Launch the Lidar_Utility**	

There are several launch files inside the master_launcher node. 
For .pcd files:
```
roslaunch master_launcher pcd.launch
```
For Pandar 40(bag or live data):
```
roslaunch master_launcher pandar.launch
```
For sources without a specific launch file:
```
roslaunch master_launcher master.launch
```
or configure your own launch file based on master.launch as an example.


To manually set your main subscriber simply add the subscriber param as such
'''
roslaunch master_launcher master.launch subscriber:="your_topic"
'''

The master.launch will launch other launch files with the naming convention node_name_slave.launch.

The master_launcher node (lidarUtility.cpp) will search for expected PointCloud2 topic and automatically subscribe to the first one it finds. 


## Useful Links
[ROSWIKI/PCL](wiki.ros.org/pcl) Useful documentation on using PCL in ROS

[PCL/Tutorials](http://pointclouds.org/documentation/tutorials/). Useful tutorials for how to use PCL library in your own projects. Some conversions will be required for to convert the ROS PointCloud2 sensor messages into PCL types. 

[ROSWIKI/PCL/Tutorials](http://wiki.ros.org/pcl/Tutorials) Useful tutorial on getting PCL tutorials to work in ROS. If you get stuck look at some of the filters in this project for complete examples. 
 
[Wireshark](https://www.wireshark.org/) Usefull for debuging IP issues with your lidar communication and checking the IP of connected devices. 

## Directory overview
- **/build:** 
ROS generated executables
- **/devel:** 
ROS generated scripts
- **/src:** 
Where all the code goes
- **/src/drivers:** 
ROS Lidar Drivers Provided by manufacturers. See local directory README for more info on how to use. Some modifications to the drivers had to be to get them to work with the hardware I was using. 
- **/src/master_launcher:** 
Node and launch files for organizing entire project together. Subscribes automatically or manually to the raw PointCloud2 topic and republishes it for use by other nodes. Also publishes a topic of settings to be read by other nodes. The settings that are published can be automatically changed for the source of lidar data. 
- **/src/my_pcl_tutorial:** 
Partial implementation of [this](http://wiki.ros.org/pcl/Tutorials) tutorial. Useful as a skeleton for other code. Simply subscribes on a topic and echos what it hears to a publisher. See [link](http://www.pointclouds.org/documentation/tutorials/) for ways to add to the skeleton code. Note that appropriate conversions between ROS and PCL may will need to be made. 
- **/src/filters:** 
A number of ROS nodes that subscribe to PointCloud2 topics filter it and republish it to another PointCloud2 topic. Each one has a default topic and the ability to change the topics via parameters specified after rosrun or in a launch file.
- **/src/detectives:** 
A number of nodes that subscribe to ROS PointCloud2 topics interpret it in a in a viraty of ways such as detecting the road or an objects location/type and then publish the data for visualization or to be used my other more advanced nodes.
- **src/lidar_utility_msgs:**
Files defining custom message types. Used to send settings from the master_launcher node to nodes all worker nodes and meta data between worker nodes. 
-**/src/SLAM:** 
Put slam here if/when implemented.
- **/setIP:** 
A script and .txt files that are useful for configuring a static or dynamic IP address. This is useful when connecting back and forth between internet and an ethernet connected lidar with static IP. For details on how to use it see the comments in the setIP.sh file.
- **/PointCloudData:**
Recorded point cloud data for offline testing in multiple formats
- **/rviz:** 
RVIZ configuration files for a number of useful configurations. Named appropriately. Also a script to easily transfer them between the default rviz configuration file directory. Note this is not the location of the main rviz config file. The primary rviz config exists in the maser_launcher node. These are just alternate configs that may be useful. 
- **/PCL_testspace:**
 A place to test C++ PCL code before trying to integrate it into a ROS node. Also a set of useful programs for doing simple manipulations to point clouds from the command line. 

## Parameters overview
There are three parameters that all the nodes in this project have. They are: subscriber, publisher, and mode. Others may exist but at a minimum all the nodes have these. Note that when using the master_launcher launch files all parameters will need to be set and defaults can't be used. This is the preferred method for launching this project as it would be unreasonable to launch all these nodes in separate terminals. 

- **subscriber:** sets subscriber, if one isn't specified a default will be used as defined in the cpp file.

- **publisher:** sets publisher, if one isn't specified a default will be used as defined in the cpp file. If the node has multiple publishers the parameter will be used as a prefix for other automatically named topics. 

- **mode:** sets mode, if one isn't specified a default will be used as defined in the cpp file
Check the cpp file for valid modes.


- **msgSubscriber:** sets a topic to subscribe to for listening to custom messages that are defined in lidar_utility_msgs. Not all nodes have this parameter. 

to set a parameter at launch include the argument: ```parameter_name:="your_setting"```
to set a parameter in a launch file see this [ROS documentation](http://wiki.ros.org/roslaunch/) or se master.launch for examples. 




