## Lidar_Utility
## 6/4/18
## Nikolas Xarles Gamarra

# Description 
This "Lidar_Utility" is a coclection or ROS nodes that are useful for filtering and interpreting point cloud data. It depends on [ROS](http://wiki.ros.org/) and [PCL](http://pointclouds.org/documentation/). A basic understanding of ROS is required to understand the nodes and topics that make this project work. 

# Useful Links
[ROSWIKI/PCL](wiki.ros.org/pcl_ros#pcd_to_pointcloud) Useful documentation on using PCL in ROS
[PCL/Tutorials](http://pointclouds.org/documentation/tutorials/). Useful tutorials for how to use PCL library in your own projects. Some conversions will be required for using ros PointCloud2 messages. 
[ROSWIKI/PCL/Tutorials](http://wiki.ros.org/pcl/Tutorials) Useful tutorial on getting PCL tutorials to work in ROS. If you get stuck look at some of the filters in this project for complete examples. 
[Wireshark](https://www.wireshark.org/) Usefull for debuging IP issues with your lidar communication and checking the IP of connected devices. 
# Here is an overview of the directories
- /build: 
ROS generated excutables
- /devel: 
ROS generated scripts
- /src: 
Where all the code goes
- 	/src/drivers: 
ROS Lidar Drivers Provided by manufacturer. See README for more info on how to use
- 	/src/master_launcher: 
Node and launch files for organizing entire project together. Subscribes automatically or manually to the raw PointCloud2 topic and republishes it for use by other nodes. 
-	/src/my_pcl_tutorial: 
Partial implementation of [this](http://wiki.ros.org/pcl/Tutorials) tutorial. Useful as a skeleton for other code. Simply subscribes on a topic and echos what it hears to a publisher. See [link](http://www.pointclouds.org/documentation/tutorials/) for ways to add to the skeleton code.
-	/src/filters: 
A number of ROS nodes that subscribe to PointCloud2 topics filter it and republish it to another ROS topic. Each one has a default topic constant and the ability to change the topics via parameters specified after rosrun or in a launch file.
-	/src/detectives: 
A number of nodes that subscribe to ROS PointCloud2 topics interpret it in a in a viraty of ways such as detecting the road or a type of vehicle and then publish the data for visuilization or to be used my other more advanced nodes.
-	/src/SLAM: 
Put slam here when implemented.
-	/src/master_launcher: 
Main launch file. Launches multiple other launch files. Also cpp code to listen for likely topics and republish on a topic that the rest of the program expects. 
- /setIP: 
A script and two .txt files that are usefult for configuring a static or dynamic IP address that are needed when connecting to the lidars. This is useful when connecting back and forth between internet and an ethernet connected lidar. For details on how to use it see the comments in the setIP.sh file.
- /data:
Recorded point cloud data for offline testing in multiple formats
- /data/pcd:
Point Cloud Data collected from various sensors for offline testing in the .pcd format. To publish this data see the publish data section below.
- /data/rosbag:
Point Cloud Data collected from various sensors in the rosbag format. To publish this data see the publish data section below. 
rosbag
- /rviz: 
RVIZ configuration files for a number of useful configurations. Named appropriately. Also a script to easily transfer them between the default rviz configuration file directory.


# To download the project:
'''
git clone https://github.com/niko1499/Lidar_Utility.git
'''
# To compile the project:
cd ~/Lidar_Utility
'''
source devel/setup.bash	   *(OR ADD IT TO YOUR BASHRC)
'''

'''
catkin_make
'''

**NOTE IF catkin_make fails run this instead or delete the pandalidar driver**
'''
catkin_make --cmake-args -DCamera_Enable=ON
'''

# To Run the project
There are several ways to run different parts of the project


**First select the source of your lidar data and launch it:**

- rslidar: Below is the roslaunch command to launch the rslidar driver.
'''
roslaunch rslidar_pointcloud rs_lidar_16.launch
'''
- [pandalidar](https://github.com/HesaiTechnology/HesaiLidar-ros)
'''
roslaunch hesai_lidar pandora_ros.launch
'''
- .pcd file: Below is an example of how to use ROS to publis a .pcd as a ros PointCloud2 topic.

'''
rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/PointCloudData/pcd/velodyne1/2826laser.pcd 1
'''
'''
rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/PointCloudData/pcd/velodyne1/2321laser.pcd 1

'''
- OTHER

Other sources should also work with the Lidar_Utility as long as they publish PointCloud2
data as a ROS topic. See the next step for how to specify a subscription topic a launch time. 

**Second Launch the Lidar_Utility**	

This is the main launch file. It will launch multiple nodes and rviz. To launch individual nodes you can use rosrun or roslaunch after exploring the directors. Most nodes contain a launch directory.
'''
roslaunch master_launcher Lidar_Utility.launch
'''
To manually set your main subscriber simply use
'''
roslaunch master_launcher Lidar_Utility.launch _subscriber:="your_topic"
'''

The Lidar_Utility.launch will launch other launch files with the nameing convention node_name_core.launch these launch files contain important parameters and sometimes launch multiple instance with different setting of the same node. 

The main purpose of this launch file is to search for all the possib
# Parameters
There are three parameters that all the nodes in this project have. They are: subscriber, publisher, and mode. Others may exist but at a minimum all the nodes have these.

subscriber: sets subsccriber, if one isn't specified a default will be used as defined in the cpp file
publisher: sets publisher, if one isn't specified a default will be used as defined in the cpp file
mode: sets mode, if one isn't specified a default will be used as defined in the cpp file
For ease of use the mode can be set by a capital letter, lower case letter, or lowercase single word.
Modes are typically things like filtered vs unfiltered, radial vs statistical ect... Look at the cpp file for a node to see available modes. 

to set a parameter at launch: _parameter_name:="your_setting"
to set a parameter in a launch file see this [ROS documentation]()






