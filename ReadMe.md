# On‑Robot Dev Guide — `isaac_ros-dev` (Docker Workspace)

This README describes how to place our packages inside the **Docker‑mounted** workspace `~/dev/isaac_ros-dev`, build them in the correct order, and hook them into the navigation stack (Isaac/“rona”). It’s **separate** from the main ROS_GNN_ws docs and is intended to live **on the robot**.

> Convention: the host path `~/dev/isaac_ros-dev` is bind‑mounted into the container. All commands below run **inside the container shell** unless noted.

---

## 0) Workspace Layout (target)
We aim for this structure under `~/dev/isaac_ros-dev/src`:

```
isaac_ros-dev/
├─ src/
│  ├─ mmwave_ti_ros/                  # TI mmWave radar stack
│  │   ├─ serial/
│  │   ├─ ti_mmwave_ros2_interfaces/
│  │   └─ ti_mmwave_ros2_pkg/
│  ├─ gnn_interfaces/                 # custom message types
│  ├─ gnn_objects_layer/              # custom Nav2 costmap layer (uses TrackedPolygon)
│  ├─ isaac_nav/                      # Isaac/rona navigation bundle (launch + configs)
│  │   └─ rona_navigation/            # ← copy our nav package here (launch/config overlays)
│  └─ (other isaac/isaac_ros packages as needed)
├─ build/  (colcon)
├─ install/ (colcon)
└─ log/     (colcon)
```

If `isaac_nav/` already exists, **place `rona_navigation/` *inside* it**, not alongside.

---

## 1) Copy/Sync Packages Into the Workspace(Or copy manually)

> Do this on the **host** or inside the container (adjust paths accordingly). `rsync` preserves perms/symlinks; `cp -r` also works.

```bash
# From wherever your sources live → into the mounted workspace
# Radar stack
rsync -aH --delete /path/to/mmwave_ti_ros/   ~/dev/isaac_ros-dev/src/mmwave_ti_ros/

# GNN packages
rsync -aH --delete /path/to/gnn_interfaces/  ~/dev/isaac_ros-dev/src/gnn_interfaces/
rsync -aH --delete /path/to/gnn_objects_layer/ ~/dev/isaac_ros-dev/src/gnn_objects_layer/

# Navigation overlays: copy rona_navigation *inside* isaac_nav/
mkdir -p ~/dev/isaac_ros-dev/src/isaac_nav/
rsync -aH --delete /path/to/rona_navigation/ ~/dev/isaac_ros-dev/src/isaac_nav/rona_navigation/
```

> If you have a zip/tarball: `unzip mmwave_ti_ros.zip -d ~/dev/isaac_ros-dev/src/` etc.

---

## 2) Build Order (strict)

Build in this exact sequence; **source after each step** so downstream packages find headers/messages.

### 2.1 TI mmWave Radar
```bash
cd ~/dev/isaac_ros-dev

colcon build --symlink-install --packages-select serial
source install/local_setup.bash

colcon build --symlink-install --packages-select ti_mmwave_ros2_interfaces
source install/local_setup.bash

colcon build --symlink-install --packages-select ti_mmwave_ros2_pkg
source install/local_setup.bash
```
This brings up radar topics (PointCloud2 + optional LaserScan).

### 2.2 GNN Interfaces (custom msgs)
```bash
colcon build --symlink-install --packages-select gnn_interfaces
source install/setup.bash
```

### 2.3 GNN Objects Layer (Nav2 costmap plugin)
```bash
colcon build --symlink-install --packages-select gnn_objects_layer
source install/setup.bash
```

### 2.4 Navigation Bundle (Isaac/“Rona”)
```bash
colcon build --symlink-install --packages-select rona_navigation
source install/setup.bash
```
Make sure all other required packages fro the Nav_stack are built.

---

## 3) Runtime — Launch & Control

> Open **two terminals** inside the container (or use `tmux`). Always source the overlay first:
```bash
source install/setup.bash
```

### 3.0 Pre-reqs
- Make sure to check if zenoh and robomaster bringup nodes are running with proper ROS_DOMAIN_ID and discoverable topics across devices.

### 3.1 Bring up the Radar
- Ensure `/dev/ttyTI_cmd` and `/dev/ttyTI_data` exist (udev rules section below).  
- Launch the radar node:
```bash
ros2 launch ti_mmwave_ros2_pkg mmwave_datahdl_socket_launch_6g_demo.py namespace:=$ROBOT
```
- Verify topics:
```bash
ros2 topic list | grep -E "(radar|mmwave|scan|pcl)"
ros2 topic echo /ti_mmwave/radar_scan_pcl --once
```

### 3.2 Start Nav2 + Our Costmap Layer
- Use the **rona_navigation** launch under `isaac_nav`. Replace with your actual launch:
```bash
ros2 launch rona_navigation waypoint_gnn_bringup_launch.py  
```
- Check that the layer loads:
```bash
ros2 pkg plugins --ros-plugins costmap_2d | grep -i gnn
```
- Inspect node logs for plugin loading messages.

### 3.3 Provide/Verify `TrackedPolygon` (if another node publishes)
Confirm the required topic is visible:
```bash
ros2 topic echo /tracked_polygons --once
```

---



## 4) Navigation Bringup (rona_navigation)

### Launch: `waypoint_gnn_bringup_launch.py`

- Wraps Nav2 bringup + filters, supports per-robot namespaces.
- Uses `<robot_namespace>` placeholders inside YAML → replaced with `prefix/`.

#### Arguments
- `prefix` — robot namespace (default `$ROBOT`)
- `params_file` — Nav2 params file (`gnn_robomaster_nav2_radar.yaml`)
- `map` — map yaml
- `autostart` (default: true)
- `use_rviz` (default: false)
- many others (see launch file)

#### Typical Multi-Robot Example
Robot rm03:
```bash
ros2 launch ti_mmwave_ros2_pkg mmwave_datahdl_socket_launch_6g_demo.py namespace:=$ROBOT
ros2 launch rona_navigation waypoint_gnn_bringup_launch.py prefix:=rm03
```
Robot rm04:
```bash
ros2 launch ti_mmwave_ros2_pkg mmwave_datahdl_socket_launch_6g_demo.py namespace:=$ROBOT
ros2 launch rona_navigation waypoint_gnn_bringup_launch.py prefix:=rm04
```

---

## 5) Nav2 Parameters — `gnn_robomaster_nav2_radar.yaml`

### AMCL
- Namespaced frames: `<robot_namespace>base_footprint`, `<robot_namespace>odom`
- `scan_topic: scan`
- `set_initial_pose: true`

### BT Navigator
- Uses `navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml`
- Groot monitoring enabled (ports 1666/1667)

### Controller Server (DWB)
- `controller_frequency: 20.0`
- Vel limits tuned (`max_vel_x: 0.6`)
- Critics: RotateToGoal, PathAlign, GoalAlign, etc.

### Local Costmap
- `global_frame: <robot_namespace>odom`
- Plugins: `static_layer`, `gnn_costmap_layer`, `inflation_layer`
- GNN layer:
  - `enabled: True`
  - `topic: "/tracked_polygons"`
  - `label_decay_times: [5.0, 20.0, 5.0, 10.0, 20.0]` # Other WS Robot Boundary Forklift
  - `label_inflation_radii: [0.0, 0.8, 0.3, 0.5, 0.7]`

### Global Costmap
- `global_frame: map`
- GNN layer disabled by default (`enabled: False`)
- Inflation tuned (`inflation_radius: 0.25`, `cost_scaling_factor: 2.5`)

---

## 6) GNN Objects Layer — Delay & Decay

- **Skip polygon** if `delay > 1.0s`
- **Warn** if `0.3s < delay <= 1.0s`
- **Decay time**: per-label via `label_decay_times`, else `decay_time` (default 5s)
- **Inflation**: rasterized lethal inside polygon, graded costs outward up to radius
- **CSV Logs** written under:
  - `/workspaces/isaac_ros-dev/gnn_logs/gnn_costmap/gnn_costmap_log_*.csv`
  - `/workspaces/isaac_ros-dev/gnn_logs/gnn_costmap/gnn_costmap_first_use_*.csv`



---

## 7) Troubleshooting

**“Package X not found” / message type missing**  
→ You probably opened a new shell without sourcing. Run: `source ~/dev/isaac_ros-dev/install/local_setup.bash`  
→ Or you built out of order; repeat the build in the order above.

**Plugin not loading at runtime**  
→ make sure you have sudo permission on the terminal where you will launch the nav launch file 
```bash
sudo chown -R $(whoami):$(whoami) .
```
→ Check the plugin name in the Nav2 YAML matches the class exported by `gnn_objects_layer`.  
→ Inspect node logs for “Failed to create bond for…” or “could not create class”; then run `ros2 pkg plugins --ros-plugins costmap_2d` to confirm it’s installed.

**Radar node present but no points**  
→ Verify serial permissions and check if device mapping service is running or not otherwise follow these steps for static USB port configuration.

🔧 Update your rule file:
```bash
sudo apt install -y nano
sudo nano /etc/udev/rules.d/99-ti-mmwave.rules
```
Copy and Paste (replace existing content):
```bash
# TI mmWave Command Port (Enhanced Com Port = interface 00)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="ttyTI_cmd"

# TI mmWave Data Port (Standard Com Port = interface 01)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="ttyTI_data"
```
✅ Reload Rules & Trigger
```bash
sudo udevadm control --reload
sudo udevadm trigger
```
Verify
```bash
ls -l /dev/ttyTI_*
```
You should now see e.g.:
```
/dev/ttyTI_cmd -> ttyUSB4
/dev/ttyTI_data -> ttyUSB5
```

**Clock/time issues**  
→ Ensure NTP is working (chrony or systemd‑timesyncd). Incorrect time breaks TLS (apt/pip) and ROS timestamping.

---
