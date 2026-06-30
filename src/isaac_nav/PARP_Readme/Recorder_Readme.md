# PARP Data Recording — Plain-English README

**To whom this is for:** anyone — developer or non developer — who needs to *record robot data*
during a PARP experiment, find the recorded files afterwards, or understand what
each script in this folder actually does.

This README explains, for every recording-related file:

- **WHAT** it records (which data),
- **WHERE** the recording lands (which folder),
- **HOW** to run it (copy-paste commands), and
- **WHY** it exists (what problem it solves).


---

## 1. The 90-second background (read this first)

The robot is a **RoboMaster** named **`rm03`**. While it drives around, dozens of
little data streams flow through the system — its position, its plan, its 
detected obstacles, its radio signal strength, and so on. In ROS 2 (the robot's
operating-system layer) each stream is called a **topic** (think: a labelled pipe
of live data).

To analyse an experiment *after* it happened, we **record** these topics into a
file called a **rosbag** (a "black box flight recorder" for the robot). We can
replay a rosbag later, or run analysis scripts on it.

A few words you'll keep seeing:

| Word | Plain meaning |
|---|---|
| **Topic** | A named live data stream, e.g. `/rm03/plan` (the robot's planned path). |
| **rosbag / bag** | The recorded file holding those streams. Our bags use the **MCAP** format. |
| **MCAP** | A modern, efficient bag file format (the default here). `sqlite3` is the older alternative. |
| **CSV** | A spreadsheet-friendly table. A second, lighter log written *next to* each bag for quick analysis. |
| **Container** | A sealed software box (called **cuvslam**) the robot's code runs inside. You enter it with `enter_cuvslam`. |
| **nav2** | The standard robot navigation "brain" (planning + driving). |
| **L1 / L2 / L3** | The three custom PARP perception-risk layers stacked on nav2. See the rule of thumb below. |
| **Relay** | A tiny helper that copies one topic to another so it can be recorded cleanly (explained in §3.4). |
| **`test_bag_name`** | A label *you* choose for a trial, e.g. `all_layer_rm03_rm04`. It becomes part of the bag/CSV folder name (and fills `RouteCost.trial_id`). On the catch-all recorder it's the positional name argument. **Not** the 5G `inference_node` `run_id`. |

### Rule of thumb — the three PARP layers

Read them **bottom-up** the stack. Each layer scores a different kind of risk:

| Layer | Full name (ROS package) | What it scores | One word |
|---|---|---|---|
| **L1** | **Communication risk** (`nav2_comms_risk_layer`) | radio-link quality / connectivity | **comms** |
| **L2** | **Observability risk** (`nav2_o11y_risk_layer`) | how well the robot can sense / see | **see** |
| **L3** | **Safety** (`nav2_safety_risk_layer`) | protective safety halo along the planned path | **safe** |

> **Mnemonic:** *"Can I talk → can I see → am I safe?"* = L1 → L2 → L3.
> Spell it **C-O-S** (Comms → Observability → Safety) going up the stack.
> **"All layers"** means **L1 + L2 + L3** are active together.

### `test_bag_name` naming convention

Name a trial after *which layers* and *which robots* it covers — the `test_bag_name`
becomes part of the folder name, so keep it short and descriptive:

| Example `test_bag_name` | Meaning |
|---|---|
| `all_layer_rm03_rm04` | all three layers on, robots **rm03 + rm04** |
| `all_layer_all_robot` | all three layers on, **every robot** |
| `all_layer_rm03` | all three layers on, **single robot rm03** |

**Golden rule before *any* command below:**

```bash
enter_cuvslam              # step inside the container
source install/setup.bash  # make the robot's commands available
```

Every command in this README assumes you have done those two lines first.

---

## 2. The map — which file does what

There are **four** recording-related code files. One is the everyday "big button"
launch file; three live in this `dk_ros2_bags/` folder and are smaller helpers.

| # | File | One-line job | Records data? |
|---|---|---|---|
| 1 | [safety_route_cost_puc_parp_bringup_launch.py](../robo_flw_dk/ROS2_GNN_Robomaster/src/isaac_nav/rona_navigation/launch/rviz/safety_route_cost_puc_parp_bringup_launch.py) | **All-layers trial**: start the *whole* robot stack with **L1 + L2 + L3** + record a bag + write a CSV, all in one command. | ✅ Yes |
| 2 | [relay_tracked_polygons.py](../dk_ros2_bags/relay_tracked_polygons.py) | **L3 sub-test recorder**: capture a short 30–60 s window *while the stack is already running*. Relay + recorder in one. | ✅ Yes |
| 3 | [bag_record_all.py](../dk_ros2_bags/bag_record_all.py) | **Catch-all**: dump *every visible topic* for debugging a weird event. | ✅ Yes |
| 4 | [tracked_polygons_relay_node.py](../dk_ros2_bags/tracked_polygons_relay_node.py) | **Relay only**: a plumbing helper. Copies one tricky topic so it *can* be recorded. Records nothing itself. | ❌ No |

> **Naming note:** older cheat sheets call file #3 `bag_record_all_launch.py` /
> `bag_record_all.launch.py` and start it with `ros2 launch`. It is now a plain
> Python script — **`bag_record_all.py`** — run it with `python3` (see §3.3).

---

## 3. Each file in detail

### 3.1 `safety_route_cost_puc_parp_bringup_launch.py` — the all-layers "everything on" button

**File:** [robo_flw_dk/.../safety_route_cost_puc_parp_bringup_launch.py](../robo_flw_dk/ROS2_GNN_Robomaster/src/isaac_nav/rona_navigation/launch/rviz/safety_route_cost_puc_parp_bringup_launch.py)

**WHAT it does:** One command brings up the *complete* robot brain and (optionally)
records it:

- nav2 navigation stack,
- all three PARP layers — **L1 (Communication risk) + L2 (Observability risk) + L3
  (Safety)** — by loading the `safety_parp_gnn_robomaster_nav2_radar.yaml` parameter file,
- the `route_cost_puc` node (computes route cost + PUC, the thesis metrics),
- a **CSV logger** that writes those metrics to a spreadsheet,
- a **relay** helper (only when recording — see §3.4),
- a **rosbag recorder** capturing the trial's key topics.

**WHAT it records (the ~18 key topics):**

```
/rm03/plan                       the global planned path
/rm03/local_plan                 the short-range path
/rm03/global_costmap/costmap     the map of "where it's costly to drive"
/rm03/global_costmap/costmap_raw
/rm03/local_costmap/costmap
/tracked_polygons_logged         camera/GNN-detected obstacle shapes (via relay)
/rm03/comms/link_stats           radio link quality
/rm03/battery_state              battery
/rm03/route_cost                 PARP metric: route cost
/rm03/route_puc                  PARP metric: PUC
/rm03/route_puc_components       PARP metric: PUC breakdown
/rm03/odom                       robot's own position estimate
/rm03/vicon_pose                 ground-truth position (motion-capture)
/tf, /tf_static                  coordinate-frame transforms
/navigate_to_pose/feedback       progress toward the goal
/navigate_to_pose/result         did it reach the goal
```

**WHERE it lands:**

- Bag: `/workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<test_bag_name>_<timestamp>_bag/`
- CSV: `/workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<test_bag_name>_<timestamp>.csv`

(`<host>` is the robot name, e.g. `rm03`. See §4 for the host-vs-container path twist.)

**HOW to run it:**

```bash
# Record an all-layers trial:
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=all_layer_rm03_rm04 record_rosbag:=true

# Run the stack but DON'T record (CSV still written):
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=all_layer_rm03_rm04

# Drive the robot to its goal, then press Ctrl-C once to stop & store the bag.
```

**WHY it exists:** This is the main data-collection tool for the **all-layers
condition** (L1 + L2 + L3 on). One command = one reproducible, fully-captured trial.

---

### 3.2 `relay_tracked_polygons.py` — the L3 sub-test mini-recorder

**File:** [dk_ros2_bags/relay_tracked_polygons.py](../dk_ros2_bags/relay_tracked_polygons.py)

**WHAT it does:** A small Python script that does **two jobs in one process**:

1. runs the polygon relay (§3.4) internally, and
2. immediately starts `ros2 bag record` on a fixed list of ~19 topics.

You run it **on top of an already-running stack** — you do *not* restart nav2. Press
Ctrl-C once and it cleanly closes both the relay and the bag.

**WHAT it records:** A curated L3 topic set defined in the script's `TOPICS` list —
`/tracked_polygons_logged`, both costmaps + raw, global/local/transformed plans,
`odom`, `odom_vicon`, `vicon_pose`, `comms/link_stats`, `fake_rsrp`, `cmd_vel`,
`scan_filtered`, `tf`, `tf_static`, and the navigate-to-pose feedback/result.

**WHERE it lands:**
`/workspaces/isaac_ros-dev/dk_ros2_bags/L3_real_<sub>_<timestamp>_bag/`
where `<sub>` is the sub-test name you pass (defaults to `Scenario_1`).

**HOW to run it:**

```bash
cd /workspaces/isaac_ros-dev
python3 dk_ros2_bags/relay_tracked_polygons.py Scenario_1
# (no argument → defaults to sub-test "Scenario_1")
# Drive the short maneuver, then Ctrl-C to save.
```

**WHY it exists:** During the L3 (Safety) acceptance tests (R1–R9), you want to capture
many short 30–60 second windows back-to-back **without** tearing down and rebuilding the
whole stack each time. This script makes each capture a one-liner.

---

### 3.3 `bag_record_all.py` — the catch-all "record everything"

**File:** [dk_ros2_bags/bag_record_all.py](../dk_ros2_bags/bag_record_all.py)

**WHAT it does:** Records **every topic currently visible** on the system (the `-a`
"all" flag), into one MCAP bag. It does **not** start the robot stack and it does
**not** start the relay.

**WHERE it lands:**
`/workspaces/isaac_ros-dev/dk_ros2_bags/<test_bag_name>_<timestamp>_bag/`
— a **flat folder, no per-host sub-folder** (this differs from the all-layers trial,
which nests under `<host>/`). Default name is `$ROBOT` or the hostname (e.g. `rm03`).

**HOW to run it** (it's a plain Python script — run it with `python3`, either from
inside the folder or via its full path; the name is a **positional** argument and
`--bag-dir` / `--storage` are flags):

```bash
# This recorder can be used for multiple purposes — just pass the arguments.

cd /workspaces/isaac_ros-dev/dk_ros2_bags
python3 bag_record_all.py

# Give it a custom label:
python3 bag_record_all.py scenario_1

# Use the older sqlite3 format instead of MCAP:
python3 bag_record_all.py --storage sqlite3

# Send it to a different output folder:
python3 bag_record_all.py quick_test --bag-dir /tmp/triage
```

**WHY it exists:** When something strange happens and you don't yet know *which* topic
holds the clue, record them *all* and sort it out later. It's a debugging/triage net,
not a tidy experiment recorder. Because it doesn't include the relay, start the relay
separately (§3.4) if you need the obstacle polygons.

---

### 3.4 `tracked_polygons_relay_node.py` — the relay (plumbing only)

**File:** [dk_ros2_bags/tracked_polygons_relay_node.py](../dk_ros2_bags/tracked_polygons_relay_node.py)

**WHAT it does:** Copies messages from `/tracked_polygons` to a new topic
`/tracked_polygons_logged`. **It records nothing** — it's pure plumbing.

**WHERE it lands:** Nowhere on disk. It just publishes the cleaned-up topic onto the
live system so a recorder can pick it up.

**HOW to run it** (only needed as a side-helper for catch-all recording):

```bash
python3 /workspaces/isaac_ros-dev/dk_ros2_bags/tracked_polygons_relay_node.py
```

**WHY it exists (the interesting bit):** The `/tracked_polygons` topic is *dual-type* —
the GNN obstacle detector publishes it as `gnn_interfaces/TrackedPolygon`, but foreign
RViz viewer subscribers also advertise it as `geometry_msgs/PolygonStamped`. When
`ros2 bag record` sees one topic with two different message types, it gets confused and
records **zero messages, or the wrong type**. The relay republishes *only* the correct
type onto a brand-new, single-type topic (`/tracked_polygons_logged`) that records
cleanly. The all-layers launch and the L3 sub-test recorder start this relay for you
automatically; the catch-all does not.

---

## 4. Where do the files actually go? (the path twist)

You run everything **inside the cuvslam container**, but the files are physically
stored on the **host** machine. They are the same files seen through two different
doorways (a "bind-mount"):

| Where you are | Path you use |
|---|---|
| **Inside the container** (after `enter_cuvslam`) | `/workspaces/isaac_ros-dev/dk_ros2_bags/...` |
| **On the host** (a normal terminal on the robot) | `/home/robot_3/isaac_ros-dev/dk_ros2_bags/...` |

> The host path `/home/robot_3/...` **does not exist inside the container**, and
> `ros2 bag info` only works on the container path while you're inside the container.
> When in doubt, goldern rule is: first always use the container path.

Folder-name patterns:

- **All-layers trials:** `dk_ros2_bags/<host>/<test_bag_name>_<timestamp>_bag/` + a `.csv` beside it.
- **L3 sub-test:** `dk_ros2_bags/L3_real_<sub>_<timestamp>_bag/`.
- **Catch-all:** `dk_ros2_bags/<test_bag_name>_<timestamp>_bag/` (flat, no host sub-folder).

---

## 5. "Which one do I run?" — a 10-second decision guide

```
Do you want to start a fresh, scripted experiment trial?
│
├─ Yes — full stack, all layers (L1+L2+L3)  → §3.1  safety_..._bringup_launch.py
│
└─ No — the stack is already running.
   │
   ├─ I want a short L3 sub-test window  → §3.2  relay_tracked_polygons.py
   ├─ Something weird happened, grab     → §3.3  bag_record_all.py
   │  EVERYTHING                            (+ §3.4 relay if you need polygons)
   └─ I only need the polygon relay      → §3.4  tracked_polygons_relay_node.py
```

---

## 6. Ready-to-paste recipes

**Recipe A — one full all-layers trial, everything captured (most common):**

```bash
# Terminal 1 — one command does it all:
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=all_layer_rm03_rm04 record_rosbag:=true
# Drive to the goal, then Ctrl-C. Bag + CSV land in dk_ros2_bags/<host>/all_layer_rm03_rm04_<ts>_*
```

**Recipe B — bring the stack up once, then capture several L3 sub-tests:**

```bash
# Terminal 1 — stack only (no bag):
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py \
    test_bag_name:=all_layer_rm03

# Terminal 2 — sub-test 1:
cd /workspaces/isaac_ros-dev
python3 dk_ros2_bags/relay_tracked_polygons.py S1        # Ctrl-C to save

# Terminal 2 again — sub-test 2:
python3 dk_ros2_bags/relay_tracked_polygons.py S2_again # Ctrl-C to save
# ...repeat per S-test. The stack stays up the whole time.
```

**Recipe C — catch-all triage when you don't know what you're hunting:**

```bash
# Terminal 1 — stack:
ros2 launch rona_navigation safety_route_cost_puc_parp_bringup_launch.py test_bag_name:=triage

# Terminal 2 — relay (catch-all does NOT bundle it):
python3 /workspaces/isaac_ros-dev/dk_ros2_bags/tracked_polygons_relay_node.py

# Terminal 3 — record everything:
cd /workspaces/isaac_ros-dev/dk_ros2_bags
python3 bag_record_all.py triage_session
```

---

## 7. Did it work? — verify after Ctrl-C

```bash
# All-layers trial (nested under host):
ros2 bag info /workspaces/isaac_ros-dev/dk_ros2_bags/<host>/<test_bag_name>_<ts>_bag

# Catch-all (flat folder):
ros2 bag info /workspaces/isaac_ros-dev/dk_ros2_bags/<test_bag_name>_<ts>_bag
```

A healthy recording shows:

- **`Storage id: mcap`** (the expected format).
- Roughly **18–25 topics** for an all-layers trial or L3 sub-test, or **50–80** for the catch-all.
- **`/tracked_polygons_logged`** listed as `gnn_interfaces/msg/TrackedPolygon` with a
  message count **greater than 0** → confirms the relay was running and obstacle data
  was captured.

---

## 8. Common problems & fixes

| Symptom | Likely cause | Fix |
|---|---|---|
| `python3 bag_record_all.py` → "No such file or directory" | You're not in `dk_ros2_bags/`, or used the wrong name. | `cd` into `dk_ros2_bags` first, or give the full path `python3 /workspaces/isaac_ros-dev/dk_ros2_bags/bag_record_all.py`. |
| Bag has `/tracked_polygons` with **0 messages** | The dual-type problem — recorded the raw topic without the relay. | Use the all-layers launch or the L3 sub-test recorder (both relay automatically), or start §3.4 manually. |
| `ros2 bag info` says "path does not exist" | You used the host path `/home/robot_3/...` while inside the container. | Use the container path `/workspaces/isaac_ros-dev/...`. |
| No `.csv` next to the bag | CSV logger only runs with the all-layers launch, not the L3 sub-test or catch-all. | Expected — only the all-layers launch writes CSVs. |
| Commands "not found" | Forgot to source the environment. | Run `enter_cuvslam` then `source install/setup.bash`. |

---

## 9. One-paragraph summary for a non-developer reader

To record a real experiment trial, run **one** launch command —
`safety_route_cost_puc_parp_bringup_launch.py` (all layers on: **L1 Communication +
L2 Observability + L3 Safety**) — adding `test_bag_name:=<your_label> record_rosbag:=true`,
then drive to the goal and press Ctrl-C. The result is an MCAP bag plus a CSV under
`dk_ros2_bags/<robot>/<your_label>_<timestamp>_*`. For short follow-up captures on a
running stack use `relay_tracked_polygons.py`; for blind "record everything" debugging
use `bag_record_all.py` (plus the `tracked_polygons_relay_node.py` helper if you
need obstacle polygons). The relay exists purely to turn one confusing dual-type topic
into a clean single-type one that recorders can capture.
