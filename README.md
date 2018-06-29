# Lidar_Utility
Nikolas Xarles Gamarra -+- nxgamarra@gmail.com -+- https://github.com/niko1499/Lidar_Utility
## Description 
This "Lidar_Utility" is a coclection or ROS nodes that are useful for filtering and interpreting point cloud data. It depends on [ROS](http://wiki.ros.org/) and [PCL](http://pointclouds.org/documentation/). A basic understanding of ROS is required to understand the nodes and topics that make this project work. 

## To download and compile the project:
```
cd ~/
mkdir -p Lidar_Utility
cd Lidar_Utility
git clone https://github.com/niko1499/Lidar_Utility.git
source devel/setup.bash 
catkin_make
```
## To Run the project
1. **Select the source of your lidar data and launch it (you may need to launch roscore first):**

- **rslidar:** Below is the roslaunch command to launch the [rslidar](http://www.robosense.ai/) driver that is included in this workspace. It has been modified slightly from its origional form. 
```
roslaunch rslidar_pointcloud rs_lidar_16.launch
```
- **Pandar40:** Below is the roslaunch command to launch the [Pandar40](http://www.hesaitech.com/en/index.html) driver that is included in this workspace. It has been modified slightly from its origional form.
```
roslaunch pandar_pointcloud Pandar40_points.launch
```
- **.pcd file:** Below are a couple examples of how to use ROS to publis a .pcd as a ros PointCloud2 topic. See the pcd directory for more valid file names. 
_
```
rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/PointCloudData/pcd/velodyne1/2826laser.pcd .1
rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/PointCloudData/pcd/velodyne1/2321laser.pcd .1
```
- **rosbag:**
First cd into the bag directory. Then run rosbag. The -l loops the file. The -r specifies a rate multiplier. See the pcd directory for more valid file names. 
```
cd ~/Lidar_Utility/PointCloudData/rosbag/SAIC_campus
rosbag play -l -r .3 veh5.bag
```
- **OTHER:**
Other sources should also work with the Lidar_Utility as long as they publish PointCloud2
data as a ROS topic. See the next step for how to specify a subscription topic a launch time. Note you may need to change setting inside lidarUtility.cpp in order to get the program to work with the new data set.  

2. **Launch the Lidar_Utility**	

There are several launch files inside the master_launcher node. master.launch is the main one however depending on the source of data other launch files may work better. 
For .pcd files:
```
roslaunch master_launcher pcd.launch
```
For Pandar 40:
```
roslaunch master_launcher pandar.launch
```
To manually set your main subscriber simply add the subscriber param as such
'''
roslaunch master_launcher master.launch subscriber:="your_topic"
'''

The Lidar_Utility.launch will launch other launch files with the nameing convention node_name_core.launch these launch files contain important parameters and sometimes launch multiple instance with different setting of the same node. 

The main purpose of this launch file is to search for all the possib

## Useful Links
[ROSWIKI/PCL](wiki.ros.org/pcl) Useful documentation on using PCL in ROS

[PCL/Tutorials](http://pointclouds.org/documentation/tutorials/). Useful tutorials for how to use PCL library in your own projects. Some conversions will be required for using ros PointCloud2 messages. 

[ROSWIKI/PCL/Tutorials](http://wiki.ros.org/pcl/Tutorials) Useful tutorial on getting PCL tutorials to work in ROS. If you get stuck look at some of the filters in this project for complete examples. ROS PointCloud2 sensor mesages will need to be converted to PCL PointCloud2 types and then possibly converted again to the PCL type used by the filter or alogrythem you want to implement. 
 
[Wireshark](https://www.wireshark.org/) Usefull for debuging IP issues with your lidar communication and checking the IP of connected devices. 

## Directory overview
- /build: 
ROS generated excutables
- /devel: 
ROS generated scripts
- /src: 
Where all the code goes
- 	/src/drivers: 
ROS Lidar Drivers Provided by manufacturers. See included README for more info on how to use. These are the drivers and versions for the hardware I used while at SAIC Motor. Some modifications to the drivers had to be made. 
- 	/src/master_launcher: 
Node and launch files for organizing entire project together. Subscribes automatically or manually to the raw PointCloud2 topic and republishes it for use by other nodes. Also publishes a topic of settings to be read by other nodes. The settings that are published can be automatically changed for the source of lidar data. 
-	/src/my_pcl_tutorial: 
Partial implementation of [this](http://wiki.ros.org/pcl/Tutorials) tutorial. Useful as a skeleton for other code. Simply subscribes on a topic and echos what it hears to a publisher. See [link](http://www.pointclouds.org/documentation/tutorials/) for ways to add to the skeleton code. Note that appropriate conversions between ROS and PCL may will need to be made. 
-	/src/filters: 
A number of ROS nodes that subscribe to PointCloud2 topics filter it and republish it to another ROS topic. Each one has a default topic and the ability to change the topics via parameters specified after rosrun or in a launch file.
-	/src/detectives: 
A number of nodes that subscribe to ROS PointCloud2 topics interpret it in a in a viraty of ways such as detecting the road or an objects location/type and then publish the data for visuilization or to be used my other more advanced nodes.
- 	src/lidar_utility_msgs:
Files defining custom message types. Used to send settings to nodes all sub-nodes and metadata between worker nodes. 
-	/src/SLAM: 
Put slam here when implemented.
-	/src/master_launcher: 
- /setIP: 
A script and two .txt files that are usefult for configuring a static or dynamic IP address that are needed when connecting to the rslidar. This is useful when connecting back and forth between internet and an ethernet connected lidar with static IP. For details on how to use it see the comments in the setIP.sh file.
- /data:
Recorded point cloud data for offline testing in multiple formats
- /rviz: 
RVIZ configuration files for a number of useful configurations. Named appropriately. Also a script to easily transfer them between the default rviz configuration file directory. Note this is not the location of the main rviz config file. The primary rviz config exists in the maser_launcher node. These are just alternate configs that may be useful. 
- /PCL_testspace: A place to test C++ PCL code before trying to integrate it into a ROS node. 

## Parameters overview
There are three parameters that all the nodes in this project have. They are: subscriber, publisher, and mode. Others may exist but at a minimum all the nodes have these.

subscriber: sets subsccriber, if one isn't specified a default will be used as defined in the cpp file
publisher: sets publisher, if one isn't specified a default will be used as defined in the cpp file
mode: sets mode, if one isn't specified a default will be used as defined in the cpp file
For ease of use the mode can be set by a capital letter, lower case letter, or lowercase single word.
Modes are typically things like filtered vs unfiltered, radial vs statistical ect... Look at the cpp file for a node to see available modes. 

to set a parameter at launch include the arguement: ```parameter_name:="your_setting"```
to set a parameter in a launch file see this [ROS documentation]()






