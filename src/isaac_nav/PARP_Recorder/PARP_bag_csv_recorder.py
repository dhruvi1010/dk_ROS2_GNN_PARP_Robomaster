#!/usr/bin/env python3
"""
PARP self-contained recorder — ros2 bag (MCAP) + matching metrics CSV, ONE process.

A single Ctrl+C cleanly stops all three jobs below:

  1. RELAY : /tracked_polygons (gnn_interfaces/TrackedPolygon) -> /tracked_polygons_logged.
             FALLBACK for dirty networks: if any stray rviz still advertises
             geometry_msgs/PolygonStamped on /tracked_polygons (making it dual-type),
             recording the relayed single-type copy stays safe. The safety launch records
             RAW /tracked_polygons directly (no relay) now that rviz is fixed.
  2. BAG   : `ros2 bag record -s mcap` of the curated PARP topic set.
  3. CSV   : one row per planner cycle with EVERY route_cost / PUC term + component,
             schema byte-identical to route_cost_csv_logger.py (the safety-launch CSV).

PORTABLE to any robot / any machine (rm03, rm04, cr01, flw, ...):
  - the per-robot namespace and the output paths auto-derive from
        $ROBOT  >  hostname  >  "rm03"   (hyphens sanitised to underscores)
  - host-independent topics (/tf, /navigate_to_pose/*, the relayed polygons, ...)
    are recorded WITHOUT a prefix; per-robot topics get the /<HOST>/ prefix.

OUTPUTS (paired by the same <label>_<ts> stem so they obviously belong together):
    <BASE_DIR>/<HOST>/<label>_<ts>_bag/    <- rosbag (MCAP)
    <BASE_DIR>/<HOST>/<label>_<ts>.csv     <- metrics CSV

USAGE (sourced container shell, Zenoh/DDS env set):
    python3 PARP_bag_csv_recorder.py                              # label "default_run"
    python3 PARP_bag_csv_recorder.py test_bag_name:=crossing_B4   # SAME arg style as the launch
    python3 PARP_bag_csv_recorder.py crossing_B4                  # bare positional also works
    ROBOT=rm04 python3 PARP_bag_csv_recorder.py test_bag_name:=rush_hour_B4
    BASE_DIR=/some/other/dir python3 PARP_bag_csv_recorder.py smoke

Run it on TOP of an already-running stack (any launch that brings up nav2 + route_cost
for this robot). Do NOT also pass record_rosbag:=true to the launch, or you'd get two
relays and two bags.
"""

import os
import sys
import csv
import time
import socket
import signal
import subprocess
import threading
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from gnn_interfaces.msg import TrackedPolygon
from perception_aware_nav2_msgs.msg import RouteCost, RoutePUC, RoutePUCComponents

# --------------------------------------------------------------------------- #
# Portable identity. Override either by exporting ROBOT=... or BASE_DIR=...    #
# --------------------------------------------------------------------------- #
BASE_DIR = os.environ.get("BASE_DIR", "/workspaces/isaac_ros-dev/dk_ros2_bags")
HOST = (os.environ.get("ROBOT") or socket.gethostname() or "rm03").replace("-", "_")
NS = f"/{HOST}"

SRC_TOPIC = "/tracked_polygons"          # dual-type source (GNN + foreign rviz subs)
DST_TOPIC = "/tracked_polygons_logged"   # single-type, recordable copy


def record_topics():
    """The curated PARP topic set, host-portable.

    Per-robot topics are prefixed with /<HOST>/; host-independent topics
    (tf, navigation action, diagnostics, the relayed polygons) stay global.
    """
    per_robot = [
        # 1. plans + path metrics
        "plan", "local_plan", "transformed_global_plan", "received_global_plan",
        # 2. costmaps (raw + compressed + updates)
        "global_costmap/costmap", "global_costmap/costmap_raw",
        "global_costmap/costmap_updates",
        "local_costmap/costmap", "local_costmap/costmap_raw",
        # 3. comms (L1) + battery (energy)
        "comms/link_stats", "fake_rsrp", "battery_state", #"modem_link",
        # 4. route_cost / PUC outputs (dissertation core)
        "route_cost", "route_puc", "route_puc_components",
        # 5. pose / ground truth / actuation
        "odom", "odom_vicon", "vicon_pose",
        "cmd_vel", "cmd_wheel_speed", "scan_filtered",
        # 6. behaviour tree
        "behavior_tree_log",
    ]
    host_independent = [
        DST_TOPIC,                         # relayed single-type perception input
        "/tf", "/tf_static",
        "/navigate_to_pose/feedback", "/navigate_to_pose/result",
        "/diagnostics",
    ]
    return [f"{NS}/{t}" for t in per_robot] + host_independent


class Relay(Node):
    """Republish /tracked_polygons -> /tracked_polygons_logged (single-type)."""

    def __init__(self):
        super().__init__("tracked_polygons_relay")
        self.pub = self.create_publisher(TrackedPolygon, DST_TOPIC, 10)
        self.create_subscription(TrackedPolygon, SRC_TOPIC,
                                 lambda m: self.pub.publish(m), 10)
        self.get_logger().info(f"relaying {SRC_TOPIC} -> {DST_TOPIC}")


class RouteCostCsvLogger(Node):
    """One CSV row per route_cost cycle, every J(pi) term + PUC component.

    Schema is mirrored from route_cost_puc_pynode/route_cost_csv_logger.py — keep
    the two HEADERs in sync if the RouteCost/RoutePUC messages ever change.
    """

    HEADER = [
        'wall_iso', 'sec', 'nanosec', 'robot_id', 'trial_id',
        'j_total', 'time_term', 'obs_term', 'comms_term', 'safety_term', 'energy_term',
        'path_length_m', 'nominal_speed_mps', 'soc_fraction',
        'lambda_obs', 'lambda_comms', 'lambda_safety', 'lambda_energy',
        'puc', 'p_t', 'p_o', 'p_c', 'p_s', 'p_e',
        'w_t', 'w_o', 'w_c', 'w_s', 'w_e',
        'n_obstacles', 'nearest_obstacle_m', 'rsrp_dbm', 'jitter_ms', 'min_ttc_s',
    ]

    def __init__(self, csv_path):
        super().__init__("parp_route_cost_csv_logger")
        self._last_puc = None
        self._last_comp = None

        self._f = open(csv_path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)
        self._f.flush()

        # Absolute, host-portable topic names — match route_cost_puc_node's
        # /<HOST>/route_cost publishers (relative topic + PushRosNamespace(prefix)).
        self.create_subscription(RouteCost, f"{NS}/route_cost", self.on_cost, 20)
        self.create_subscription(RoutePUC, f"{NS}/route_puc", self.on_puc, 20)
        self.create_subscription(RoutePUCComponents,
                                 f"{NS}/route_puc_components", self.on_comp, 20)
        self.get_logger().info(f"CSV logging to {csv_path}")

    def on_puc(self, msg):
        self._last_puc = msg

    def on_comp(self, msg):
        self._last_comp = msg

    def on_cost(self, msg):
        puc = self._last_puc
        comp = self._last_comp
        row = [
            datetime.now().isoformat(), msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.robot_id, msg.trial_id,
            msg.j_total, msg.time_term, msg.obs_term, msg.comms_term,
            msg.safety_term, msg.energy_term,
            msg.path_length_m, msg.nominal_speed_mps, msg.soc_fraction,
            msg.lambda_obs, msg.lambda_comms, msg.lambda_safety, msg.lambda_energy,
            puc.puc if puc else '', comp.p_t if comp else '',
            comp.p_o if comp else '', comp.p_c if comp else '',
            comp.p_s if comp else '', comp.p_e if comp else '',
            puc.w_t if puc else '', puc.w_o if puc else '',
            puc.w_c if puc else '', puc.w_s if puc else '',
            puc.w_e if puc else '',
            msg.n_obstacles, msg.nearest_obstacle_m, msg.rsrp_dbm,
            msg.jitter_ms, msg.min_ttc_s,
        ]
        self._w.writerow(row)
        self._f.flush()

    def destroy_node(self):
        try:
            self._f.close()
        except Exception:
            pass
        super().destroy_node()


def parse_label(argv):
    """Trial label, mirroring the launch's `test_bag_name:=<name>` argument.

    Accepts (in priority order):
        test_bag_name:=crossing_B4   # launch-style key:=value (preferred)
        crossing_B4                  # bare positional (backward compatible)
    Falls back to "default_run".
    """
    for a in argv[1:]:
        if a.startswith("test_bag_name:="):
            return a.split(":=", 1)[1] or "default_run"
    for a in argv[1:]:
        if ":=" not in a:                 # first bare token
            return a
    return "default_run"


def main():
    label = parse_label(sys.argv)
    ts = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    stem = f"{label}_{ts}"

    out_dir = os.path.join(BASE_DIR, HOST)
    os.makedirs(out_dir, exist_ok=True)
    bag_uri = os.path.join(out_dir, f"{stem}_bag")
    csv_path = os.path.join(out_dir, f"{stem}.csv")

    topics = record_topics()

    rclpy.init()
    relay = Relay()
    logger = RouteCostCsvLogger(csv_path)

    executor = SingleThreadedExecutor()
    executor.add_node(relay)
    executor.add_node(logger)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(2.0)  # let discovery advertise /tracked_polygons_logged before record

    cmd = ["ros2", "bag", "record", "-o", bag_uri, "-s", "mcap"] + topics
    print(f"[parp_rec] host={HOST}  ns={NS}  label={label}")
    print(f"[parp_rec] bag : {bag_uri}  ({len(topics)} topics, MCAP)")
    print(f"[parp_rec] csv : {csv_path}")
    print(f"[parp_rec] recording... Ctrl+C once to finalise both bag and CSV.")
    proc = subprocess.Popen(cmd, start_new_session=True)

    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\n[parp_rec] Ctrl+C -> finalising bag...")
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.terminate()
    finally:
        executor.shutdown()
        logger.destroy_node()
        relay.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[parp_rec] bag saved: {bag_uri}")
        print(f"[parp_rec] csv saved: {csv_path}")


if __name__ == "__main__":
    main()
