

---

# PARP Branch — Integration Guide (RoNa `isaac_nav`)

Perception-Aware Routing (PARP) for the RoboMaster EP fleet. This is **not a new
repository** — it is the `parp_dev` branch of the existing **RoNa `isaac_nav`** repo. It
adds custom Nav2 costmap-risk layers (L1 comms · L2 observability · L3 safety) and a
route-cost / PUC observer nodes.

This guide assumes the **base RoNa setup is already installed and working** on the target
robot (Docker `cuvslam` image, RMW Zenoh, 5G/Vicon, `ROBOT` env) per the main
[`README.md`](https://gitlab.cc-asp.fraunhofer.de/robomaster/isaac_nav) on the `humble`
branch. It only covers what is **extra** to bring PARP up on a sibling robot (rm04, rm05…).

<p align="center">
  <table>
    <tr><th>OS</th><th>ROS</th><th>Branch</th><th>Status</th></tr>
    <tr><td>Ubuntu 22.04</td><td>Humble</td><td><code>parp_dev</code></td><td>L1+L2+L3+route_cost tested on rm03 ✅</td></tr>
  </table>
</p>

---

## Table of Contents

- [Repository & Branch](#repository--branch)
- [What the PARP Branch Adds](#what-the-parp-branch-adds)

- [Prerequisites (assumed already working)](#prerequisites-assumed-already-working)
- [1. Get the PARP Branch onto a Robot](#1-get-the-parp-branch-onto-a-robot)
- [2. Build](#2-build)
- [3. Per-Robot Configuration](#3-per-robot-configuration)
- [4. Launch Commands](#4-launch-commands)
- [5. Verify](#5-verify)
- [6. Push Workflow (publishing your working code)](#6-push-workflow-publishing-your-working-code)
- [7. Known Issues / Multi-Robot Gotchas](#7-known-issues--multi-robot-gotchas)

---

## Repository & Branch

```
repo   : git@gitlab.cc-asp.fraunhofer.de:robomaster/isaac_nav.git
base   : humble        (origin/HEAD)
branch : parp_dev      ← all PARP work lives here
path   : <workspace>/isaac_ros-dev/src/isaac_nav/
```

PARP is developed on `parp_dev`, which branches off `humble`. Everything else in the
workspace (Isaac ROS, Nav2, `rmw_zenoh`, drivers) is unchanged from the standard robot
image — only `isaac_nav` switches branch.

---

## What the PARP Branch Adds

On top of `humble`, `parp_dev` introduces the perception-aware navigation stack:

| Package | Type | Role |
|---|---|---|
| `gnn_interfaces` | msgs | `TrackedPolygon` (shared GNN detections from the 5G edge) |
| `gnn_objects_layer` | `nav2_costmap_2d::Layer` | `gnn_costmap_layer` — hard inflation in the **local** costmap |
| `perception_aware_nav2/perception_aware_nav2_msgs` | msgs | `LinkStats`, + `RouteCost`/`RoutePUC`/`RoutePUCComponents` |
| `perception_aware_nav2/comms_monitor_pynode` | `ament_python` | L1 link-quality publisher → `comms/link_stats` |
| `perception_aware_nav2/comms_script` | helpers | L1 edge echo / probe scripts |
| `perception_aware_nav2/nav2_comms_risk_layer` | layer | **L1** comms radio-shadow halo (global costmap) |
| `perception_aware_nav2/nav2_o11y_risk_layer` | layer | **L2** observability halos from `/tracked_polygons` |
| `perception_aware_nav2/nav2_safety_risk_layer` | layer | **L3** predicted-trajectory smear (dynamic labels 2,4) |
| `perception_aware_nav2/route_cost_puc_pynode` | `ament_python` | `J(π)` / PUC observer + CSV logger |
| `rona_navigation/config/*` | nav2 YAML | `parp_`, `o11y_parp_`, `safety_parp_` configs + `route_cost_puc_params.yaml` |
| `rona_navigation/launch/rviz/*` | bringup | `parp_`, `o11y_parp_`, `route_cost_puc_parp_`, `safety_parp_`, `safety_route_cost_puc_parp_` launchers |



## Prerequisites (assumed already working)

These come from the **base RoNa README** and must already pass on the target robot. Do
**not** redo them for PARP:

- Docker `cuvslam` image present; `start_cuvslam` / `enter_cuvslam` aliases work.
- `rmw_zenoh` built; Zenoh router runs (`start_zenoh` / `zenoh_start.service`).
- `~/.bashrc` exports (per-robot — note `ROBOT`):
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_DOMAIN_ID=130
  export ROBOT=rm04                         # ← this robot's id
  export ISAAC_ROS_WS=/home/robot_4/isaac_ros-dev/
  source ~/ws_rmw_zenoh/install/setup.bash
  ```
- Vicon tracks this robot → `/rm04/vicon_pose`; RoboMaster driver publishes `/rm04/odom`,
  `/rm04/battery_state`.
- **5G edge** GNN running and this robot included in `multi_robot_inference.launch.py` so
  `/tracked_polygons` carries its detections (contributor id = robot number: rm04→4).

| Device | IP | (from base README) |
|---|---|---|
| robot_3 (rm03) | 172.16.30.103 | PARP reference robot |
| robot_4 (rm04) | 172.16.30.104 | target |
| robot_5 (rm05) | 172.16.30.105 | target |
| 5G edge (kybernetes/flw) | 172.16.3.61 / 172.16.3.62 | GNN + edge echo |

---

> ⚠️ `isaac_ros-dev` on rm03 is bind-mounted into the live `cuvslam` container and is
> fragile — run these git commands yourself; verify `git status` before committing so no
> unintended local edits ride along.

## 1. Get the PARP Branch onto a Robot

Inside the robot's workspace, switch the **`isaac_nav` repo** to `parp_dev` (the rest of
the workspace stays put):

```bash
cd $ISAAC_ROS_WS/src/isaac_nav        # e.g. /home/robot_4/isaac_ros-dev/src/isaac_nav

git fetch origin
git checkout parp_dev                  # or: git switch parp_dev
git pull origin parp_dev
```

If the robot already runs a different `isaac_nav` branch for other projects, stash/commit
local work first. The PARP stack is additive (new packages), so a clean `parp_dev`
checkout does not disturb the base nav packages.


---

## 2. Build

Build interfaces first, then the layers and nodes (inside the `cuvslam` container):

```bash
enter_cuvslam
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

# 1) messages / interfaces first
colcon build --packages-select perception_aware_nav2_msgs gnn_interfaces
source install/setup.bash

# 2) layers + nodes + bringup
colcon build --packages-select \
  gnn_objects_layer \
  nav2_comms_risk_layer nav2_o11y_risk_layer nav2_safety_risk_layer \
  comms_monitor_pynode route_cost_puc_pynode \
  rona_navigation
source install/setup.bash

```


## 3. Per-Robot Configuration

The stack namespaces itself from the `ROBOT` env var: `PushRosNamespace($ROBOT)` plus a
`<robot_namespace>` placeholder in the nav2 YAMLs that `ReplaceString` rewrites at launch.
So the **three nav2 configs and all frames port automatically** — `export ROBOT=rm04` is
enough for the layers.



---

## 4. Launch Commands

All launchers accept `prefix:=rm0#` (defaults to `$ROBOT`; the `safety_route_cost` one
defaults to `default`, so always pass it). Replace `#` with the robot number.

```bash
enter_cuvslam
source /workspaces/isaac_ros-dev/install/setup.bash

# L1 only
ros2 launch rona_navigation parp_bringup_launch.py                  prefix:=rm04

# L1 + L2  
ros2 launch rona_navigation o11y_parp_bringup_launch.py             prefix:=rm04

# L1 + L2 + L3  (layers only)
ros2 launch rona_navigation safety_parp_bringup_launch.py           prefix:=rm04

# Full PARP: L1+L2+L3 + route-cost + CSV + rosbag 
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    prefix:=rm04 \
    run_id:=rm04_$(date -u +%Y%m%d_%H%M%S) \
    record_rosbag:=true
```

---

## 5. Verify

```bash
# lifecycle + plugin ladder, now under /rm04/
ros2 lifecycle get /rm04/global_costmap/global_costmap          # active [3]
ros2 param get  /rm04/global_costmap/global_costmap plugins
# ["static_layer","gnn_costmap_layer","comms_risk_layer","o11y_risk_layer","safety_risk_layer","inflation_layer"]

ros2 topic hz   /rm04/comms/link_stats                          # ~5 Hz (L1)
ros2 topic echo /rm04/route_cost --field robot_id --once        # → rm04
ros2 topic echo /tracked_polygons gnn_interfaces/msg/TrackedPolygon --once   # shared edge stream
```

Then run the layer behaviour checks from
[op2_ph2_L3_safety_checklist.md](op2_ph2_L3_safety_checklist.md) §3 (R1–R9) with `/rm03/`
→ `/rm04/`. L3 acceptance is single-robot, so rm04 validates on its own.

---

## 6. Push Workflow (publishing your working code)



On every other robot afterwards:

```bash
cd $ISAAC_ROS_WS/src/isaac_nav && git checkout parp_dev && git pull origin parp_dev
# then rebuild (§2)
```



---

## 7. Known Issues / Multi-Robot Gotchas

- **`/tracked_polygons` is absolute and fleet-shared** — every robot's L2/L3/route-cost
  subscribe to the same edge topic; each paints into its own namespaced costmap. The
  growing subscription count is expected, not a leak.
- **`safety_route_cost_…` launcher defaults `ROBOT` to `default`** — always pass
  `prefix:=rm0#` or it namespaces under `/default/`.
- **rm04 doubles as the B0 baseline** on the legacy `humble` launcher — switching it to
  `parp_dev` permanently removes that free ablation (see
  [op2_ph4_integration.md](op2_ph4_integration.md)).
- **Build order matters** — `perception_aware_nav2_msgs` and `gnn_interfaces` must build
  before the layers/nodes, or pluginlib discovery and Python imports fail.
- **Two `perception_aware_nav` trees exist** — `perception_aware_nav/` (v1, older
  `perception_aware_msgs`) and `perception_aware_nav2/` (current). PARP uses the **`_nav2`**
  packages; ignore the v1 tree.