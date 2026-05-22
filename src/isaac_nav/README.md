# Robomaster Navigation RoNa
In this repository, you will find the necessary packages to use navigation on the Robomaster using waypoints, as well all the necesary launch files to run the Applications that are orhrestrated in the other Package (add link to the other repository)
<p align="center">
  <table>
    <tr>
      <th>Version</th>
      <th>ROS</th>
      <th>Status</th>
    </tr>
    <tr>
      <td>Ubuntu 22.04</td>
      <td>Humble</td>
      <td>Tested ✅</td>
    </tr>
    <tr>
      <td>Ubuntu 20.04</td>
      <td>Galactic</td>
      <td>Tested ✅</td>
    </tr>
  </table>
</p>


## Table of Contents

- [Overview](#overview)
- [Workspace Layout](#workspace-layout)
- [Packages](#packages)
- [Installation](#installation)
- [1. Robomaster Base Setup](#1-robomaster-base-setup)
- [2. Install Docker](#2-install-docker)
- [3. Create Docker Aliases](#3-create-docker-aliases)
- [4. Install Python Dependencies](#4-install-python-dependencies)
- [5. Install ROS Middleware (RMW Zenoh DDS)](#5-install-ros-middleware-rmw-zenoh-dds)
- [6. 5G Modem Setup](#6-5g-modem-setup)
- [6.1 Configure the 5G Modem](#61-configure-the-5g-modem)
- [6.2 Disable Other Network Routes](#62-disable-other-network-routes)
- [6.3 Recovering Lost 5G Connection](#63-recovering-lost-5g-connection)
- [Usage](#usage)
- [1. Environment Setup](#1-environment-setup)
- [2. Device IPs](#2-device-ips)
- [3. Launch Commands](#3-launch-commands)
- [4. Camera and Segmentation](#4-camera-and-segmentation)
- [5. Kubernetes Deployment](#5-kubernetes-deployment)
- [5.1 K3s Quick Commands](#51-k3s-quick-commands)
- [6. Deployment File Example](#6-deployment-file-example)
- [7. Important Locations](#7-important-locations)
- [8. Steps to Push a Docker Image](#8-steps-to-push-a-docker-image)
- [9. Known Issues](#9-known-issues)

## Overview

The current Robomaster setup uses a Docker image. All RoNa packages are built inside this Docker environment, which shares a folder with the local host on the robot Orin board.

## Workspace Layout

Shared workspace:

```bash
~/isaac_ros-dev/
```

Source packages:

```bash
~/isaac_ros-dev/src/
```

After modifying source code, rebuild inside the Docker image so changes are applied correctly.

## Packages

| Package | Description |
|---|---|
| `rona_navigation` | Navigation launch files for Vicon, SLAM, and AMCL; includes planner and controller configuration. |
| `rona_nvblox` | Visual SLAM and camera-related applications. |
| `rona_physical` | Nodes to send physical waypoints to the Robomaster. |
| `rona_comm` / `rona_msgs` | Custom messages and services for consistent communication across RoNa packages. |
| `rona_people_segmentation` | Image segmentation and object classification integrated with `rona_nvblox` for dynamic costmaps. |
| `ros_robomaster_description` | TF tree definitions for Robomaster localization and navigation. |

## Installation

### 1. Robomaster Base Setup

Follow the installation steps from the `robomaster-setup` package to install required ROS packages for base control.

If the robot moves erratically or this is the first setup, calibrate using the Robomaster App:

1. Set the Robomaster computer to cell phone connection mode using the selector switch.
2. Connect your phone to the Robomaster controller Wi-Fi (password is on the controller).
3. Open the Robomaster App and go to Settings (top-right corner).
4. Navigate to System -> Motor Addressing and follow the app instructions.

### 2. Install Docker

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add Docker repository:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo \"${UBUNTU_CODENAME:-$VERSION_CODENAME}\") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

# Install Docker engine and plugins:
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Allow Docker commands without `sudo`:

```bash
sudo usermod -aG docker ${USER}
su - ${USER}
```

### 3. Create Docker Aliases

Add these aliases to `~/.bashrc`:

```bash
# Start container
alias start_cuvslam='docker run -it --rm --privileged --network host --ipc=host -v $ISAAC_ROS_WS:/workspaces/isaac_ros-dev -v /etc/localtime:/etc/localtime:ro --name "cuvslam" --runtime nvidia --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh --workdir /workspaces/isaac_ros-dev emmagon/kubernetes_trials:cuvslamv7 /bin/bash'

# Enter existing container
alias enter_cuvslam="docker exec -it -u admin cuvslam bash"
```

Reload shell:

```bash
source ~/.bashrc
```

### 4. Install Python Dependencies

Inside Docker:

```bash
pip install paho-mqtt
pip install transforms3d
```

### 5. Install ROS Middleware (RMW Zenoh DDS)

ROS communication uses `rmw_zenoh_dds`, built from source.

On the robot:

```bash
mkdir -p ~/ws_rmw_zenoh/src
cd ~/ws_rmw_zenoh/src
git clone https://github.com/ros2/rmw_zenoh.git -b humble

cd ~/ws_rmw_zenoh
rosdep install --from-paths src --ignore-src --rosdistro humble -y
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Inside Docker:

```bash
cd /workspaces/isaac_ros-dev/src
git clone https://github.com/ros2/rmw_zenoh.git -b humble

cd /workspaces/isaac_ros-dev
rosdep install --from-paths src/rmw_zenoh --ignore-src --rosdistro humble -y
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select rmw_zenoh_cpp zenoh_cpp_vendor zenoh_security_tools
```

### 6. 5G Modem Setup

#### 6.1 Configure the 5G Modem

```bash
# Set modem to use 5G mode
sudo mmcli -m 0 --set-allowed-modes=5g

# Create a GSM connection for the modem
sudo nmcli c add type gsm ifname cdc-wdm0 con-name fiveG-iml apn default

# Bring up connection
sudo nmcli con up fiveG-iml

# Verify modem status
mmcli -m 0

# Save the 5G Modem Logs
sudo journalctl -u ModemManager -f | grep modem0 >> ~/<NAME>.log
```

#### 6.2 Disable Other Network Routes

To force traffic through 5G:

1. Open Network Manager TUI:

```bash
sudo nmtui
```

2. Go to `Edit a connection -> Wi-Fi -> IPv4 Configuration -> Show Advanced Options`.
3. Under Routing, enable `Never use this network for default route`.
4. Save and reboot:

```bash
sudo reboot
```

#### 6.3 Recovering Lost 5G Connection

```bash
sudo mmcli -r -m 0
```

## Usage

### 1. Environment Setup

All devices must use Zenoh RMW.

If you need write permissions in a mounted workspace:

```bash
sudo chown -R $(whoami):$(whoami) .
```

Add to `~/.bashrc` or `~/.bash_profile`:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /home/robot_2/ws_rmw_zenoh/install/setup.bash

export ROS_DOMAIN_ID=130
export ROBOT=rm02
export ISAAC_ROS_WS=/home/robot_2/isaac_ros-dev/
alias start_zenoh="ros2 run rmw_zenoh_cpp rmw_zenohd"
export ZENOH_ROUTER_CONFIG_URI=/opt/robomaster/MY_ZENOH_ROUTER_CONFIG.json5
```

Reload shell:

```bash
source ~/.bashrc
```

### 2. Device IPs

| Device | IP Address | Password |
|---|---|---|
| robot_1 | 172.16.30.101 | robomaster |
| robot_2 | 172.16.30.102 | robomaster |
| robot_3 | 172.16.30.103 | robomaster |
| robot_4 | 172.16.30.104 | robomaster |
| robot_5 | 172.16.30.105 | robomaster |
| robot_6 | 172.16.30.106 | robomaster |
| isaac-sim | 172.16.80.65 | isaac-sim |
| frido | 10.64.82.149 | pcb-lab01 |
| kybernetes | 172.16.3.61 | cluster |

SSH example:

```bash
ssh robot_3@rm03.local
# password: robomaster
```

### 3. Launch Commands

Vicon navigation:

```bash
ros2 launch rona_navigation vicon_bringup_launch.py
```

RViz on monitor:

```bash
source ~/ros2_ws/install/setup.bash && ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=rm02 use_namespace:=true rviz_config:=/home/isaac-sim/ros2_ws/src/robomaster_nav2_bringup/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz
```

SLAM navigation:

```bash
ros2 launch rona_navigation waypoint_bringup_launch.py
```

Run SLAM Toolbox:

```bash
ros2 launch rona_mapping online_async_multirobot_launch.py namespace:=rm0#
```

Activate SLAM and navigation (replace `#` with robot number):

```bash
ros2 lifecycle set /rm#/rona_mapping activate
ros2 service call /rm#/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

Save generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f mymap --ros-args -r map:=/rm0#/map
```

### 4. Camera and Segmentation

Activate camera nodes:

```bash
ros2 launch rona_nvblox only_realsense.launch.py namespace:=rm03
```

Run people segmentation:

```bash
ros2 launch rona_people_segmentation people_segmentation.launch.py namespace:=rm03
```

Activate segmentation lifecycle:

```bash
ros2 lifecycle set /rm02/rona_people_segmentation configure
ros2 lifecycle set /rm02/rona_people_segmentation activate
```

### 5. Kubernetes

### 5.1 Intalaton of the cluster 
in the controler PC run the following script, cpy and paste it in a file call k3s-install.sh 
```bash
#!/bin/bash

curl -sfL https://get.k3s.io |  sh -s server \
  --cluster-init \
  --token "1234" \
  --write-kubeconfig-mode 644 \
  --disable traefik \
  --data-dir=~/etcd-backups \
  --etcd-snapshot-retention=72 \
  --etcd-snapshot-dir=~/etcd-snapshots \
  --etcd-snapshot-schedule-cron="*/3 * * * *"
```
and then run 
```bash
chmod +x k3s-install.sh
sudo ./k3s-install.sh
```
in each worker run the following script just change the <SERVER_IP> for the ral IP of the cluster computer

```bash
#!/bin/bash

curl -sfL https://get.k3s.io | sh -s - agent \
  --server https://<SERVER_IP>:6443 \
  --token "1234" \
  --data-dir ~/etcd-backups

```

## 5.1.1 Deployment

Connect to Kybernetes cluster:

```bash
ssh kybernetes@172.16.3.61
# password: cluster
```

Deployment directory:

```bash
~/Edge_cloud_ros2/deployment
```

Apply deployment:

```bash
kubectl apply -f <FILE_NAME>
```

Useful aliases:

```bash
alias pod="kubectl get pod"
alias nod="kubectl get node"
alias deld="kubectl delete deployment"
alias deplo="kubectl get deployment"
```

View logs:

```bash
kubectl logs <POD_ID> <CONTAINER_NAME>
```

#### 5.1 K3s Quick Commands

Most useful day-to-day commands:

```bash
# Nodes and roles
kubectl get nodes
kubectl get nodes --show-labels
kubectl describe node <NODE_NAME>

# Pods and deployments
kubectl get pods -A
kubectl get deploy -A
kubectl describe pod <POD_NAME> -n <NAMESPACE>

# Watch resources live
kubectl get pods -A -w

# K3s services on a node
sudo systemctl status k3s
sudo systemctl status k3s-agent
```

Change a node role name (label-based):

```bash
# Add a role label (example: worker)
kubectl label node <NODE_NAME> node-role.kubernetes.io/worker=true

# Change role: remove old role label and add new one
kubectl label node <NODE_NAME> node-role.kubernetes.io/worker-
kubectl label node <NODE_NAME> node-role.kubernetes.io/edge=true
```

Verify role labels:

```bash
kubectl get nodes --show-labels
```

### 6. Deployment File Example

People segmentation container snippet:

```yaml
- name: people
  image: emmagon/kubernetes_trials:cuvslamv7
  imagePullPolicy: IfNotPresent
  securityContext:
    privileged: true
  env:
    - name: ROS_DOMAIN_ID
      value: "130"
    - name: RMW_IMPLEMENTATION
      value: "rmw_zenoh_cpp"
    - name: ROBOT
      value: "rm03"
  command: ["/usr/local/bin/scripts/workspace-entrypoint.sh"]
  args:
    - /bin/bash
    - -lc
    - |
      source /opt/ros/humble/setup.bash &&
      source /workspaces/isaac_ros-dev/install/setup.bash &&
      ros2 launch rona_people_segmentation people_segmentation.launch.py namespace:=rm03
  volumeMounts:
    - name: isaac-ros-workspace
      mountPath: /workspaces/isaac_ros-dev
    - name: usb-devices
      mountPath: /dev/bus/usb
```

You can modify:

- `ROS_DOMAIN_ID`
- `RMW_IMPLEMENTATION`
- `ROBOT`
- ROS 2 launch command

Each robot can run up to 7 containers per deployment, but only one deployment should be active at a time.

### 7. Important Locations

| Location | Description |
|---|---|
| `rona_navigation/config/` | Navigation controller configuration files. |
| `rona_navigation/maps/` | Navigation maps for different halls. |
| `rona_navigation/launch/rviz/` | RViz and launch files. Modify `navigation.launch.py` to change map selection. |
| `rona_physical/scripts/` | Waypoint sender scripts for robot movement. |
| `orchestrator_kub/msg/` | Custom message definitions for device status. |
| `orchestrator_kub/scripts/` | Fake RSRP and orchestrator logic for offload/communication criteria. |

### 8. Steps to Push a Docker Image

1. Start with a container:

```bash
docker run -it myimage bash
```

2. Commit changes:

```bash
docker ps
docker commit <container_id> myusername/myimage:newtag
```

Example:

```bash
docker commit 3e1c0e2345d1 myusername/modified-image:v1
```

3. Log in to registry:

```bash
docker login
```

4. Push image:

```bash
docker push myusername/modified-image:v1
```

### 9. Known Issues

1. Controller parameter tuning:
   using MQTT reaches about 1.3 m/s; using DWB controller reaches about 2.8 m/s.
2. Navigation through poses may fail:
   installing Navigation2 from source may be required for goal handling changes.
3. Node communication overload:
   multi-robot ROS traffic can overload communication.
4. 5G modem instability:
   connection may stop without a clear failure message; restart modem.
5. USB scan issues:

```bash
sudo usbreset /dev/bus/usb/001/005
```
