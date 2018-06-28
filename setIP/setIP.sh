#!/bin/bash
#This script for seting the IP address for the rslidar
#The script depends on two txt files being in the same folder (staticIP.txt and dynamicIP.txt) if lost they are included as a comment at the bottem of the file
#Nikolas Gamarra 5/28/18

#此脚本用于发送rslidar的IP地址
#脚本依赖于两个txt文件在同一个文件夹（staticIP.txt和dynamicIP.txt）中，
#如果丢失，它们作为注释包含在文件中

#To run the script type "sudo ./setIP.sh"
#If you have trouble with this make sure the file properties/permissions allow it to be excutable. 
#要运行脚本类型“sudo ./setIP.sh”
#如果你在这方面遇到麻烦，请确保文件属性/权限允许它是可改变的。

#location of ipconfig file
file='/etc/network/interfaces'

clear
ifconfig
tput setaf 1
echo 以上是当前的ip配置
echo Above is the current ip configuration
tput setaf 2
echo 输入0为动态 --- 1为RSLIDAR --- 2为PANAR 
read -p "Enter 0 for Dynamic --- 1 for rsLidar --- 2 for pandar: " input

tput setaf 7 
case $input in
	0) 
	echo \ 
	echo setting dynamic IP...
	sudo cp -f dynamicIP.txt /etc/network/interfaces
	;;
	1) 
	echo \ 
	echo setting static IP rsLidar...
	sudo cp -f rsLidar.txt /etc/network/interfaces
	;;
	2) 
	echo \ 
	echo setting static IP pandar40...
	sudo cp -f pandar40.txt /etc/network/interfaces
	;;
	
	*)echo error
	exit
	;;
esac

tput setaf 1
echo \ 
echo 网络设置现在是：
echo The network settings are now: 
tput sgr0
cat /etc/network/interfaces
tput setaf 2
echo 需要重新启动才能应用更改。 马上重启？[Y/N]
read -p "Reboot required to apply changes. Reboot now? [y/n]" inputTwo
tput setaf 7 
case $inputTwo in
	y|Y)
	sudo reboot
	;;
	n|N)
	exit
	;;
	
	*)echo error
	exit
	;;
esac

#这些是依赖文件。 删除：''评论并另存为.txt
#These are the dependent files. Remove :' ' comment and save as .txt
#-----------------staticIP.txt------------------------
:'
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

auto lo eth0
iface lo inet loopback
iface eth0 inet static
auto lo enp2s0
iface lo inet loopback
iface enp2s0 inet static
	address 192.168.1.102
	netmask 255.255.255.0
#	gateway 192.168.3.255

#See Appendix D of RS-Lidar-16UsersGuide
'
#----------------dynamicIP.txt-------------------------
: '
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d
'

