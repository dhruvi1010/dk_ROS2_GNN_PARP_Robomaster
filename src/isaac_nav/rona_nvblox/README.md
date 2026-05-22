# RONA NVBLOX

This package contains the launch and config files for using nvblox, cuvslam and also people segmentation.

## Commands

```bash
start_zenoh

ros2 launch robomaster_bringup minimal.launch.py

ros2 launch rona_nvblox realsense_without_seg.launch.py namespace:=ep02

ros2 launch rona_navigation waypoint_bringup_launch.py
```

## Commands for NvBlox, LiDAR and Navigation

```bash
start_cuvslam # to start docker 

enter_cuvslam # to enter docker

start_zenoh # to start zenoh if not running in system service

ros2 launch robomaster_bringup master.launch.py

ros2 launch rona_nvblox realsense_example.launch.py namespace:=ep02

ros2 launch rona_navigation waypoint_bringup_launch.py

ros2 launch ti_mmwave_ros2_pkg mmwave_datahdl_socket_launch_6g_demo.py namespace:=ep03

sudo chmod 666 /dev/ttyUSB4
```