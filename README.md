## Lidar_Utility
## 6/4/18
## Nikolas Xarles Gamarra

# Description 
This "Lidar_Utility" is a coclection or ROS nodes that are useful for filtering and interpreting point cloud data. It depends on [ROS](http://wiki.ros.org/) and [PCL](http://pointclouds.org/documentation/). 

# Here is an overview of the directories
- /build: ROS generated excutables
- /devel: ROS generated scripts
- /src: Where all the code goes
- 	/master_launcher: Node and launch files for organizing entire project together.
			  Subscribes automatically or manually to the raw PointCloud2 topic and 
			  republishes it for use by other nodes. 
-	/my_pcl_tutorial: Partial implementation of this http://wiki.ros.org/pcl/Tutorials tutorial.
	                  useful as a skeleton for other code. Simply subscribes on a topic and echos
			  what it hears to a publisher. 
			  See http://www.pointclouds.org/documentation/tutorials/ for ways to add to
			  the skeleton code.
-	/rslidar: drivers for the rslidar 16 and 32 lidar sensors. To run you can use the provided
		  launch files: roslaunch rslidar_pointcloud rs_lidar_16.launch
-	/filters: A number of ROS nodes that subscribe to PointCloud2 topics filter it and republish 
		  it to another ROS topic. Each one has a default topic constant and the ability to
		  change the topics via parameters specified after rosrun or in a launch file.
-	/detectives: A number of nodes that subscribe to ROS PointCloud2 topics interpret it in a
		     in a viraty of ways such as detecting the road or a type of vehicle and then 
		     publish the data for visuilization or to be used my other more advanced nodes.
-	/SLAM: Put slam here when implemented.
-	/launch: A number of ROS launch files for easily launching multiple nodes and setting them
		 to communicate correctly with eachother. 
- /setIP: A script and two .txt files that are usefult for configuring a static or dynamic IP address.
        This is useful when connecting back and forth between internet and an ethernet connected lidar.
        For details on how to use it see the comments in the setIP.sh file.
- /pcd: Point Cloud Data collected from a Velodine sensor for offline testing.
      To publish this data to a ros topic you can use:
      rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/pcd/2826laser.pcd 1
- /rviz: RVIZ configuration files for a number of useful configurations. Named appropriately. 
       Also a script to easily transfer them between the default rviz configuration file directory.


# To compile the project:
$ cd ~/Lidar_Utility
$ source devel/setup.bash	*(OR ADD IT TO YOUR BASHRC)
$ catkin_make			*If you have trouble delete /build /devel and any CMake files

# To Run the project
There are several ways to run different parts of the project

**First select the source of your lidar data and launch it:**

- rslidar
$ roslaunch rslidar_pointcloud rs_lidar_16.launch

- pandalidar
$

- .pcd file
$ rosrun pcl_ros pcd_to_pointcloud ~/Lidar_Utility/pcd/2826laser.pcd 1

- OTHER
	Other sources should also work with the Lidar_Utility as long as they publish PointCloud2
	data as a ROS topic. See the next step for how to specify a subscription topic a launch time. 

**Second Launch the Lidar_Utility**	

This is the main launch file. It will launch multiple nodes and rviz. To launch individual nodes you can use rosrun or launch after exploring the director.

$ roslaunch master_launcher Lidar_Utility.launch

To manually set your main subscriber simply use

$ roslaunch master_launcher Lidar_Utility.launch _subscriber:="your_topic"





