read -p "Enter 1 to transfer files to github directory 2 to transfer files to rviz directory: " input
case $input in
	1)  
	echo transfering to github directory...
	cp ~/.rviz/* ~/catkin_ws/rviz
	;;
	2) 
	echo transfering to rviz directory...
	cp ~/catkin_ws/rviz/*.rviz ~/.rviz
	;;
	
	*)echo error
	exit
	;;
esac


#simple
#cp ~/.rviz/* ~/catkin_ws/rviz
echo done
