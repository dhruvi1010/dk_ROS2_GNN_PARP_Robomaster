# comms_modem_monitor

This package provides a ROS 2 node that wraps the modem monitor logic from the standalone script and publishes RSRP, RSRQ, and SINR on a custom message at the topic `<robot_namespace>/modem_link`.

## Install on the robot

Run these commands on the robot inside the ROS 2 workspace that contains this package:

```bash
cd ~/ros2_ws/src
git clone <your-repo-url>  # if needed
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select comms_modem_monitor
source install/setup.bash
```

If the package is already present in the workspace, you can rebuild it directly:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select comms_modem_monitor
source install/setup.bash
```

## Run

Start the node with:

```bash
ros2 launch comms_modem_monitor modem_monitor.launch.py
```

The node publishes to `<robot_namespace>/modem_link`. With `ROBOT=rm03`, the topic becomes `/rm03/modem_link`. The node uses `ROBOT` as the namespace source.

## Optional: run with a specific namespace

```bash
export ROBOT=rm03
ros2 launch comms_modem_monitor modem_monitor.launch.py
```

## Message topic

The published message type is `comms_modem_monitor/ModemLink` and the topic is `/rm03/modem_link` when `ROBOT=rm03` is set. The available fields are:

- `rsrp`
- `rsrq`
- `sinr`
