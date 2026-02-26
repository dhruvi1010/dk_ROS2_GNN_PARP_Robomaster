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

# Robomaster Navigation RoNa
In this repository, you will find the necessary packages to use navigation **based on Vicon Localization, SLAM, and AMCL**.  
All the files can be found in the following packages:

<p align="center">
  <table style="border:1px solid #444;border-collapse:collapse;">
    <tr style="background-color:#2d2d2d;color:#fff;">
      <th style="padding:8px 16px;">Package</th>
      <th style="padding:8px 16px;">Description</th>
    </tr>
    <tr style="background-color:#1e1e1e;color:#fff;">
      <td style="padding:8px 16px;"><code>rona_navigation</code></td>
      <td style="padding:8px 16px;">Contains all the launchers related to navigation, with different localization approaches (Vicon, SLAM, AMCL), and configuration files for path planning and robot controllers.</td>
    </tr>
    <tr style="background-color:#1e1e1e;color:#fff;">
      <td style="padding:8px 16px;"><code>rona_nvblox</code></td>
      <td style="padding:8px 16px;">Includes packages related to visual SLAM and camera-based applications.</td>
    </tr>
    <tr style="background-color:#1e1e1e;color:#fff;">
      <td style="padding:8px 16px;"><code>rona_physical</code></td>
      <td style="padding:8px 16px;">Contains the nodes used to send physical waypoints to the Robomaster.</td>
    </tr>
    <tr style="background-color:#1e1e1e;color:#fff;">
      <td style="padding:8px 16px;"><code>rona_comm</code> / <code>rona_msgs</code></td>
      <td style="padding:8px 16px;">Provide custom messages and services to maintain consistent communication syntax across all RONA packages.</td>
    </tr>
    <tr style="background-color:#1e1e1e;color:#fff;">
      <td style="padding:8px 16px;"><code>rona_people_segmentation</code></td>
      <td style="padding:8px 16px;">Implements image segmentation and object classification to define which objects are visualized as segmented, integrating with <code>rona_nvblox</code> to generate dynamic costmaps.</td>
    </tr>
    <tr style="background-color:#1e1e1e;color:#fff;">
      <td style="padding:8px 16px;"><code>ros_robomaster_description</code></td>
      <td style="padding:8px 16px;">Defines the transformation tree (TF) of the Robomaster robots, publishing the TF frames required for localization and navigation.</td>
    </tr>
  </table>
</p>



## Table of Contents

- [Introduction](#introduction)
- [5GModem](#5G)
- [Installation](#installation)
- [Usage](#usage)
- [Important_plcaes](#important_places)


## Introduction

The current setup on the **Robomaster** is based on a **Docker image**.  
All the RONA packages will be built inside this Docker environment, which shares a folder with the local host on the **Orin board** of the robot.

The shared workspace is located at:
```
~/isaac_ros-dev/
```

All source packages should be placed inside:
```
~/isaac_ros-dev/src/
```

After making any modifications, make sure to **rebuild inside the Docker image** to properly apply the changes.

---

## Installation

### 1. Robomaster Base Setup

Follow the installation steps provided in the package **`robomaster-setup`**, which installs the required ROS packages to control the Robomaster base.

> 🧭 **Base Calibration**
>
> If the robot moves erratically or this is the first setup, calibrate the base using the **Robomaster App** (available on the App Store) and follow these steps:

1. Set the Robomaster computer to **cell phone connection mode** using the selector switch.  
2. Connect your cell phone to the **Robomaster controller Wi-Fi** (the password can be found on the controller).  
3. Once connected, open the Robomaster App → **Settings (top-right corner)**.  
4. Navigate to **System → Motor Addressing**, and follow the instructions in the App.

---

### 2. Install Docker

Run the following commands to install Docker on your system:

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the Docker repository:
echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc]   https://download.docker.com/linux/ubuntu   $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" |   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

# Install Docker Engine and required plugins:
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Allow Docker commands without using `sudo`:

```bash
sudo usermod -aG docker ${USER}
su - ${USER}
```

---

### 3. Create Docker Aliases

To simplify access to the container, add these aliases to your `~/.bashrc` file:

```bash
# Start container
alias start_cuvslam='docker run -it --rm   --privileged   --network host   --ipc=host   -v $ISAAC_ROS_WS:/workspaces/isaac_ros-dev   -v /etc/localtime:/etc/localtime:ro   --name "cuvslam"   --runtime nvidia   --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh   --workdir /workspaces/isaac_ros-dev   emmagon/kubernetes_trials:cuvslamv7 /bin/bash'

# Enter existing container
alias enter_cuvslam="docker exec -it -u admin cuvslam bash"
```

Reload your shell or source the `.bashrc` file to enable the aliases:

```bash
source ~/.bashrc
```

---

### 4. Install Python Dependencies

Inside the Docker container, install the required Python packages:

```bash
pip install paho-mqtt
pip install transforms3d
```

---

### 5. Install ROS Middleware (RMW Zenoh DDS)

The ROS communication layer is based on **`rmw_zenoh_dds`**, which needs to be built from source.  

#### 🦾 On the Robot:
```bash
mkdir -p ~/ws_rmw_zenoh/src
cd ~/ws_rmw_zenoh/src
git clone https://github.com/ros2/rmw_zenoh.git -b humble

cd ~/ws_rmw_zenoh
rosdep install --from-paths src --ignore-src --rosdistro humble -y
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 🧱 Inside the Docker Image:
```bash
cd /workspaces/isaac_ros-dev/src
git clone https://github.com/ros2/rmw_zenoh.git -b humble

cd /workspaces/isaac_ros-dev
rosdep install --from-paths src/rmw_zenoh --ignore-src --rosdistro humble -y
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release   --packages-select rmw_zenoh_cpp zenoh_cpp_vendor zenoh_security_tools
```
---
## 6. 5G Modem Setup

To connect the robot to the **campus 5G network**, follow these steps:

### 6.1. Configure the 5G Modem

Run the following commands to set up the 5G connection:

```bash
# Set modem to use 5G mode
sudo mmcli -m 0 --set-allowed-modes=5g

# Create a new GSM connection for the 5G modem
sudo nmcli c add type gsm ifname cdc-wdm0 con-name fiveG-iml apn default

# Bring up the 5G connection
sudo nmcli con up fiveG-iml

# Verify modem status
mmcli -m 0
```

---

### 6.2. Disable Other Network Routes

To ensure that **only the 5G network** is used for internet traffic, disable routing through Wi-Fi:

1. Open the Network Manager Text UI:
   ```bash
   sudo nmtui
   ```
2. Navigate to:  
   **Edit a connection → Wi-Fi → IPv4 Configuration → Show Advanced Options**
3. Under **Routing**, check the box:  
   ```
   Never use this network for default route
   ```
4. Save and exit, then reboot the device:
   ```bash
   sudo reboot
   ```

---

### 6.3. Recovering Lost 5G Connection

If the device loses its 5G connection, restart the modem with:

```bash
sudo mmcli -r -m 0
```

---

✅ **Result:**  
Once the build process completes, your Docker environment will include all the RONA packages and the Zenoh-based ROS 2 communication layer, fully ready for **navigation and Vicon-based localization**.



## Usage

## 1. Environment Setup

All devices (robots and computers) must use **Zenoh RMW** for communication.

Add the following to your `~/.bashrc` or `~/.bash_profile`:

If you need to change some code and it doesn't allow you to make any changes run the following command
```bash
 sudo chown -R $(whoami):$(whoami) . 
```

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /home/robot_2/ws_rmw_zenoh/install/setup.bash

export ROS_DOMAIN_ID=130
export ROBOT=rm02
export ISAAC_ROS_WS=/home/robot_2/isaac_ros-dev/
alias start_zenoh="ros2 run rmw_zenoh_cpp rmw_zenohd"
export ZENOH_ROUTER_CONFIG_URI=/opt/robomaster/MY_ZENOH_ROUTER_CONFIG.json5
```

Then reload:
```bash
source ~/.bashrc
```

---

## 2. Device IPs

| Device | IP Address | Password | 
|---------|-------------|-------------|
| robot_1 | 172.16.30.101 | robomaster |
| robot_2 | 172.16.30.102 | robomaster |
| robot_3 | 172.16.30.103 |robomaster |
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

---

## 3. Launch Commands

### 🧭 Vicon Navigation
```bash
ros2 launch rona_navigation vicon_bringup_launch.py
```
### 🧭 Rviz in the monitor
```bash
source ~/ros2_ws/install/setup.bash && ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=rm02 use_namespace:=true rviz_config:=/home/isaac-sim/ros2_ws/src/robomaster_nav2_bringup/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz
```
### 🗺️ SLAM Navigation
```bash
ros2 launch rona_navigation waypoint_bringup_launch.py
```

Then run SLAM Toolbox:
```bash
ros2 launch rona_mapping online_async_multirobot_launch.py namespace:=rm0#
```

Activate SLAM and Navigation (chnage # for the number of robot):
```bash
ros2 lifecycle set /rm#/rona_mapping activate
ros2 service call /rm#/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```
Save the map created with the SLAM
```bash
ros2 run nav2_map_server map_saver_cli -f mymap --ros-args -r map:=/rm0#/map
```
---

## 4. Camera and Segmentation

### 🎥 Activate Camera Nodes
```bash
ros2 launch rona_nvblox only_realsense.launch.py namespace:=rm03
```

### 🧍 People Segmentation
```bash
ros2 launch rona_people_segmentation people_segmentation.launch.py namespace:=rm03
```

Activate segmentation:
```bash
ros2 lifecycle set /rm02/rona_people_segmentation configure
ros2 lifecycle set /rm02/rona_people_segmentation activate
```

---

## 5. Kubernetes Deployment

For multi-robot orchestration via **Kybernetes**, connect to the cluster:

```bash
ssh kybernetes@172.16.3.61
# password: cluster
```

Navigate to:
```
~/Edge_cloud_ros2/deployment
```

Deploy containers:
```bash
kubectl apply -f <FILE_NAME>
```

### Useful Aliases
```bash
alias pod="kubectl get pod"
alias nod="kubectl get node"
alias deld="kubectl delete deployment"
alias deplo="kubectl get deployment"
```

View container logs:
```bash
kubectl logs <POD_ID> <CONTAINER_NAME>
```

---

## 6. Deployment File Example

Example snippet for **People Segmentation container**:

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
- ROS2 launch command

Each robot can run up to **7 containers** per deployment, but only **one deployment** should be active at a time.

---

## 7. Important Locations

| Location | Description |
|-----------|-------------|
| `rona_navigation/config/` | Contains configuration files for navigation controllers (you can modify the robot’s control parameters here). |
| `rona_navigation/maps/` | Stores all navigation maps for the different halls. |
| `rona_navigation/launch/rviz/` | Contains RViz and launch files — modify `navigation.launch.py` to change the map used during navigation. |
| `rona_physical/scripts/` | Includes waypoint sender scripts that control robot movement. |
| `orchestrator_kub/msg/` | Defines the custom message used to report the device status. |
| `orchestrator_kub/scripts/` | Contains the **fake RSRP** script (for network distribution simulation) and the **orchestrator** code to adjust offload criteria and communication parameters. |

---
## ✅ Steps to Push a Docker Image

### 1. Start with a container

Suppose you started a container and made changes inside it:

```bash
docker run -it myimage bash
```

Install software or modify files inside the container as needed.

---

### 2. Commit the changes

Save your modified container as a new image:

```bash
docker ps
docker commit <container_id> myusername/myimage:newtag
```

**Example:**

```bash
docker commit 3e1c0e2345d1 myusername/modified-image:v1
```

---

### 3. Log in to Docker Hub (or other registry)

```bash
docker login
```

---

### 4. Push the image

```bash
docker push myusername/modified-image:v1
```

---

## 8. Issues

1. **Controller parameter tuning**  
   - Using MQTT: 1.3 m/s  
   - Using DWB controller: 2.8 m/s  

2. **Navigation through poses not working**  
   - Install Navigation2 from source to modify goal handling.  

3. **Node communication overload**  
   - Communication between multiple robots can become overloaded under heavy ROS traffic.

4. **Error in the 5G Modem**
    - The Connection stops without fail message, to restart the communication we need to restart the modem.

5. **Issues with usb Scan**
    - Scan stops working, use sudo usbreset /dev/bus/usb/001/005

---

