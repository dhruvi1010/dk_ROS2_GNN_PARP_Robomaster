source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=130
alias start_zenoh="ros2 run rmw_zenoh_cpp rmw_zenohd"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ISAAC_ROS_WS=/workspaces/isaac_ros-dev
export ROBOT=rm04
#export ZENOH_SESSION_CONFIG_URI=/workspaces/isaac_ros-dev/ZENOH_CONFIG.json5
export ISAAC_ROS_WS=/workspaces/isaac_ros-dev
source /workspaces/isaac_ros-dev/install/setup.bash

. install/setup.bash
sudo udevadm control --reload
sudo udevadm trigger
