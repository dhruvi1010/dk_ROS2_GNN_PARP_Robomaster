### NOTE : The main branch is tested and works only for ros-foxy to switch to ros-humble use the humble branch and eventually it will be merged when foxy is out of scope.  

## Run the nav stack

Launch the full nav stack with slam

    ros2 launch robomaster_nav2_bringup robomaster_nav_launch.py

Lauch the navstack with gazebo

    ros2 launch robomaster_nav2_bringup robomaster_nav_simulation_launch.py
    
Lauch the navstack with multibple robots in gazebo (**WIP**)

    ros2 launch robomaster_nav2_bringup robomaster_nav_multi_simulation_launch.py

Launch only rviz wen runing navstack on robomaster

    ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=ep04 use_namespace:=true rviz_config:="/home/su-aschmelt/projects/robomaster_nav2_ws/src/robomaster_nav2_packages/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz"

to use the Nav_trought_poses node is needed to hardwrite the robot_base_frame="rm04/base_footprint" in the .xml file of the behaivor tree 

the robots move in a stable path at 1,5 m/s