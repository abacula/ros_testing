source /opt/ros/humble/setup.sh
source ~/ros_ex_ws/install/setup.sh
export ROS_DOMAIN_ID=1
export ROS_DISCOVERY_SERVER=192.168.1.30:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
