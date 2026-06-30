# PARP Data Recording: Plain-English README

**Who this is for:** anyone (developer or non-developer) who needs to *record robot
data* during a PARP experiment, find the recorded files afterwards, or understand what
each script does.

For every recording tool this README explains **WHAT** it records, **WHERE** the files
land, **HOW** to run it, and **WHY** it exists.

---

## 1. The 90-second background (read this first)

The robot is a **RoboMaster** (e.g. `rm03`, `rm04`, `cr01`). While it drives, dozens of
little data streams flow through the system: its position, its plan, detected
obstacles, radio signal strength, and so on. In ROS 2 each stream is a **topic**
(a labelled pipe of live data).

To analyse a trial *afterwards* we **record** topics into a **rosbag** (the robot's
"black-box flight recorder"). Alongside the bag we also write a **CSV**, a light,
spreadsheet-ready table of the PARP cost metrics, one row per planner cycle.

| Word | Plain meaning |
|---|---|
| **Topic** | A named live data stream, e.g. `/rm03/plan` (the planned path). |
| **rosbag / bag** | The recorded file holding those streams. Ours use the **MCAP** format. |
| **CSV** | A spreadsheet of the PARP metrics, written *next to* each bag for quick analysis & model training. |
| **Container** | The sealed software box (**cuvslam**) the robot's code runs in. Enter it with `enter_cuvslam`. |
| **nav2** | The standard robot navigation "brain" (planning + driving). |
| **L1 / L2 / L3** | The three PARP perception-risk layers stacked on nav2 (see below). |
| **Relay** | A tiny helper that copies one topic to a clean one so it records reliably (see §3.3). |
| **`test_bag_name`** | The label *you* choose for a trial, e.g. `all_layer_rm03_rm04`. It names the bag/CSV folder and fills `RouteCost.trial_id`. |
| **namespace / `<host>`** | The `/rm03` prefix on a robot's topics. Auto-derived from `$ROBOT` → hostname. |

### The three PARP layers (rule of thumb)

| Layer | Scores | One word |
|---|---|---|
| **L1** Communication risk | radio-link quality / connectivity | **comms** |
| **L2** Observability risk | how well the robot can sense / see | **see** |
| **L3** Safety | protective halo along the planned path | **safe** |

> **Mnemonic:** *"Can I talk → can I see → am I safe?"* = L1 → L2 → L3.
> **"All layers"** = L1 + L2 + L3 active together.

### `test_bag_name` convention

Name a trial after *which layers* and *which robots* it covers:

| Example | Meaning |
|---|---|
| `all_layer_rm03_rm04` | all three layers on, robots rm03 + rm04 |
| `all_layer_all_robot` | all three layers on, every robot |
| `S1_diagonal` | a named scenario/ablation |

**Golden rule before *any* command below:**

```bash
enter_cuvslam              # step inside the container
source install/setup.bash  # make the robot's commands available
```

---

## 2. The map: which tool does what

There are **two** recording tools you use day to day, plus one plumbing helper.

| # | Tool | One-line job | Bag | CSV |
|---|---|---|---|---|
| 1 | [safety_route_cost_puc_parp_bringup_launch.py](../rona_navigation/launch/rviz/safety_route_cost_puc_parp_bringup_launch.py) | **All-layers trial**: start the whole stack (L1+L2+L3) **and** record, in one command. | ✅ | ✅ |
| 2 | [PARP_bag_csv_recorder.py](../PARP_Recorder/PARP_bag_csv_recorder.py) | **Separate recorder**: record on top of an already-running stack; relay + bag + CSV + modem_link in one process. | ✅ | ✅ |
| 3 | `tracked_polygons_relay_node.py` (in `dk_ros2_bags/`) | **Relay only**: copies one tricky topic so it records cleanly. Records nothing itself. | ❌ | ❌ |

> Both tools now write the **same 34-column CSV** (see §4) and use the **same
> `test_bag_name`** naming. Tool 1 is the everyday button; tool 2 is the portable,
> robust fallback you run by hand.

---

## 3. Each tool in detail

### 3.1 `safety_route_cost_puc_parp_bringup_launch.py`: the all-layers "everything on" button

**File:** [../rona_navigation/launch/rviz/safety_route_cost_puc_parp_bringup_launch.py](../rona_navigation/launch/rviz/safety_route_cost_puc_parp_bringup_launch.py)

**WHAT it does:** one command brings up the *complete* robot brain and (optionally)
records it:

- nav2 navigation stack,
- all three PARP layers (**L1 + L2 + L3**) via `safety_parp_gnn_robomaster_nav2_radar.yaml`,
- the `route_cost_puc` node (computes route cost J(π) + PUC, the thesis metrics),
- a **CSV logger** (`route_cost_csv_logger`) writing the 34-column metrics table,
- a **rosbag recorder** capturing ~27 key topics.

**WHAT it records (the ~27 topics):** plans (`plan`, `local_plan`,
`transformed_global_plan`, `received_global_plan`), costmaps (global/local + raw +
updates), comms + energy (`comms/link_stats`, `fake_rsrp`, `battery_state`), the PARP
core (`route_cost`, `route_puc`, `route_puc_components`), pose/actuation (`odom`,
`odom_vicon`, `vicon_pose`, `cmd_vel`, `cmd_wheel_speed`, `scan_filtered`), and
`tf`, `tf_static`, `behavior_tree_log`, `navigate_to_pose/feedback|result`, `diagnostics`.

> **Perception note (important):** the launch is set up to record **raw
> `/tracked_polygons`** directly. This is safe now that the redundant rviz Polygon
> display was removed (see §3.3). At the moment that line is **commented out** (per
> supervisor preference), so the launch bag currently contains **no** polygon topic.
> To include it, un-comment `'/tracked_polygons'` in the recorder's topic list. The
> obstacle data still flows to `route_cost` either way, so `n_obstacles` / `obs_term`
> in the CSV are unaffected.

**WHERE it lands:**
- Bag: `/workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<test_bag_name>_<ts>_bag/`
- CSV: `/workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<test_bag_name>_<ts>.csv`

**HOW to run it:**
```bash
# Record an all-layers trial (bag + CSV):
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=all_layer_rm03_rm04 record_rosbag:=true

# Run the stack but DON'T record a bag (the CSV is still written):
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=all_layer_rm03_rm04

# Drive to the goal, then Ctrl-C once to finalise.
```

**WHY it exists:** the main data-collection tool for an all-layers trial: one command,
one reproducible, fully-captured run.

---

### 3.2 `PARP_bag_csv_recorder.py`: the separate, portable recorder (with relay fallback)

**File:** [../PARP_Recorder/PARP_bag_csv_recorder.py](../PARP_Recorder/PARP_bag_csv_recorder.py)

**WHAT it does:** a single self-contained process doing three jobs (one Ctrl-C stops all):
1. **Relay**: republishes `/tracked_polygons` → `/tracked_polygons_logged` (single-type),
   so polygons record cleanly **even if a stray rviz still dirties the bus** (the fallback).
2. **Bag**: `ros2 bag record -s mcap` of ~29 curated topics (incl. `/tracked_polygons_logged`
   and `/<host>/modem_link`).
3. **CSV**: the same 34-column metrics table as the launch (§4).

You run it **on top of an already-running stack**: it does *not* start nav2.

**Portable to any robot:** namespace and output paths auto-derive from `$ROBOT` →
hostname (rm03, rm04, cr01, flw, …). Per-robot topics get the `/<host>/` prefix;
host-independent ones (`/tf`, `/navigate_to_pose/*`, the relayed polygons) stay global.

**WHERE it lands:** `/workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<label>_<ts>_bag/`
plus `…/<label>_<ts>.csv` (bag and CSV share the same `<label>_<ts>` stem).

**HOW to run it** (naming works exactly like the launch's `test_bag_name:=`):
```bash
cd /workspaces/isaac_ros-dev

python3 PARP_Recorder/PARP_bag_csv_recorder.py test_bag_name:=S1_diagonal   # launch-style name
python3 PARP_Recorder/PARP_bag_csv_recorder.py S1_diagonal                  # bare positional also works
python3 PARP_Recorder/PARP_bag_csv_recorder.py                             # → "default_run"
```

> Don't also pass `record_rosbag:=true` to the launch while running this: you'd get
> two relays and two bags. Bring the stack up *without* recording, then run this.

**WHY it exists:** the robust fallback. When you want polygons guaranteed clean
regardless of network hygiene, or you need an extra capture on a stack that's already
up, this is the tool. It also adds `/<host>/modem_link` (the modem link topic), which
the launch bag does not.

---

### 3.3 The polygon dual-type story, the relay, and the rviz fix

`/tracked_polygons` was historically **dual-type**: the GNN publishes it as
`gnn_interfaces/TrackedPolygon`, but an rviz **Polygon display** *subscribed* to it as
`geometry_msgs/PolygonStamped`. That confused `ros2 topic echo` and `ros2 bag record`
(0 / wrong-type captures). Your own typed code was never affected.

**Two fixes are now in place:**
- **Root cause removed:** the redundant rviz Polygon display on `/tracked_polygons` was
  deleted (the correct `MarkerArray` on `/tracked_polygon_markers` does the
  visualization). With no `PolygonStamped` subscriber, `/tracked_polygons` is now
  single-type, so the **launch records it raw**. *(Takes effect once every rviz is
  restarted with the fixed config.)*
- **Fallback kept:** `PARP_bag_csv_recorder.py` still relays to
  `/tracked_polygons_logged`, so it stays safe even if a stray old-config rviz reappears.

The standalone relay helper `tracked_polygons_relay_node.py` (in the container's
`dk_ros2_bags/`) is only needed if you ever want to publish for   `/tracked_polygons_logged`
by hand for 'route_cost_puc_parp_bringup_launch.py':
```bash
python3 /workspaces/isaac_ros-dev/dk_ros2_bags/tracked_polygons_relay_node.py
```

---


> ⚠️ **One-time build** so the columns fill with data (the message gained fields):
> ```bash
> cd /workspaces/isaac_ros-dev
> colcon build --packages-select perception_aware_nav2_msgs && source install/setup.bash
> colcon build --packages-select route_cost_puc_pynode && source install/setup.bash
> ```
> Run on every robot that publishes, logs, echoes, or replays `RouteCost`.

---

## 4. Where do the files go? (the path twist)

You run inside the container, but files live on the host (a bind-mount: same files,
two doorways):

| Where you are | Path you use |
|---|---|
| **Inside the container** (`enter_cuvslam`) | `/workspaces/isaac_ros-dev/dk_ros2_bags/...` |
| **On the host** | `/home/robot_3/isaac_ros-dev/dk_ros2_bags/...` |

> The host path doesn't exist inside the container; `ros2 bag info` only works on the
> container path while you're inside. When in doubt, use the container path.

Folder patterns (both tools): `dk_ros2_bags/<host>/<name>_<ts>_bag/` + `…_<ts>.csv`.

> **Namespace gotcha:** the per-robot topics are `/<host>/…` where `<host>` = `$ROBOT`
> → hostname. The stack and the recorder must resolve to the **same** value, or the
> recorder/CSV look at the wrong robot and capture nothing. Simplest rule: set each
> robot's hostname to its id and **don't** export `ROBOT`. Only set `ROBOT` (consistently
> in every terminal) for deliberate identity-faking.

---

## 5. "Which one do I run?" (decision guide)

```
Starting a fresh, scripted trial (and want the stack too)?
├─ Yes  → §3.1  safety_..._bringup_launch.py   (records raw /tracked_polygons; bag + CSV)
└─ No: the stack is already running, OR I want guaranteed-clean polygons / modem_link
        → §3.2  PARP_bag_csv_recorder.py        (relay fallback; bag + CSV)
```

---

## 6. Ready-to-paste recipes

**Recipe A: one full all-layers trial (most common):**
```bash
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=all_layer_rm03_rm04 record_rosbag:=true
# Drive to goal, Ctrl-C. → dk_ros2_bags/<host>/all_layer_rm03_rm04_<ts>_bag + .csv
```

**Recipe B: stack up once, then record separately (clean-polygon fallback):**
```bash
# Terminal 1: stack only (no bag):
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=S1_diagonal

# Terminal 2: separate recorder (relay + bag + CSV + modem_link):
cd /workspaces/isaac_ros-dev
python3 PARP_Recorder/PARP_bag_csv_recorder.py test_bag_name:=S1_diagonal
# Ctrl-C to save.
```

> Use the **same** `test_bag_name` in both so the filenames and the CSV `trial_id`
> column match (the `trial_id` comes from the running `route_cost` node).

---

## 7. Did it work? Verify after Ctrl-C

```bash
ros2 bag info /workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<name>_<ts>_bag
head -1 /workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<name>_<ts>.csv   # 34-column header
```

Healthy signs:
- `Storage id: mcap`, ~27 topics (launch) / ~29 topics (separate recorder).
- Polygons present: **either** `/tracked_polygons` (launch, raw) **or**
  `/tracked_polygons_logged` (recorder, relayed) listed as
  `gnn_interfaces/msg/TrackedPolygon` with **count > 0**.
- CSV: `n_obstacles > 0` on some rows ⇒ perception is getting through.

---

## 8. Common problems & fixes

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2 topic echo /tracked_polygons` → "more than one type" | A stray rviz still has the old Polygon display | Restart that rviz with the fixed config, or use the relay (recorder). |
| Bag `/tracked_polygons` has **0 messages** | Dual-type at record time (dirty network), or GNN not publishing | Use `PARP_bag_csv_recorder.py` (relay fallback); confirm `ros2 topic hz /tracked_polygons`. |
| `obs_term` / `n_obstacles` always 0 | GNN not publishing, polygons not in `map` frame, or none within ~1 m of path | Check `ros2 topic echo /tracked_polygons --field header.frame_id` (must be `map`). |
| CSV new columns blank or `AttributeError` | `RouteCost.msg` not rebuilt | Run the §4 `colcon build` on that robot. |
| Recorder/CSV capture nothing per-robot | `$ROBOT` ≠ the stack's namespace | Match `ROBOT` (or hostname) to the namespace the stack uses (see §5). |
| `ros2 bag info` "path does not exist" | Used host path inside the container | Use the `/workspaces/isaac_ros-dev/...` container path. |
| Commands "not found" | Env not sourced | `enter_cuvslam` then `source install/setup.bash`. |

---

## 9. One-paragraph summary

For a real trial, run **one** launch,
`safety_route_cost_puc_parp_bringup_launch.py` (all layers: **L1 comms + L2 see + L3
safe**) with `test_bag_name:=<label> record_rosbag:=true`, then drive to the goal, Ctrl-C.
You get an MCAP **bag** + a 34-column **CSV** under
`dk_ros2_bags/<robot>/<label>_<ts>_*`. The launch records **raw** `/tracked_polygons`
(clean now that the rviz wart is gone). When you need guaranteed-clean polygons, the
extra `modem_link` topic, or a capture on a stack that's already up, run the separate
**`PARP_bag_csv_recorder.py`** (same naming, same CSV): it relays
`/tracked_polygons → /tracked_polygons_logged` as a fallback. Both write identical CSVs;
the new `n_obstacles / nearest_obstacle_m / rsrp_dbm / jitter_ms / min_ttc_s` columns
explain *why* each cost term has its value and double as model-training features.
