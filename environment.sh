#! /bin/bash
source catkin_ws/devel/setup.bash
source network_env.sh
echo "Remember to setup network"
echo "ROS_MASTER_URI="$ROS_MASTER_URI
echo "ROS_IP="$ROS_IP
echo ""
sudo pigpiod