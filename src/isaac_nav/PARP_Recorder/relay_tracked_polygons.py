#!/usr/bin/env python3
"""
Self-contained PARP bag recorder (Path B — L3 sub-tests).

Runs the /tracked_polygons -> /tracked_polygons_logged relay AND
'ros2 bag record' in ONE process. Ctrl+C once finalizes the bag.

Portable: namespace and bag dir auto-derive from $ROBOT > hostname.
On rm03 → /workspaces/isaac_ros-dev/dk_ros2_bags/rm03/L3_real_<sub>_<ts>_bag/
On rm04 → /workspaces/isaac_ros-dev/dk_ros2_bags/rm04/L3_real_<sub>_<ts>_bag/

Usage (in a sourced container shell, Zenoh env set):
    python3 relay_tracked_polygons.py          # SUB defaults to R1
    python3 relay_tracked_polygons.py R3_smear # SUB = R3_smear
    ROBOT=rm04_sim python3 relay_tracked_polygons.py R5_static_label
"""

import os
import sys
import time
import socket
import signal
import subprocess
import threading
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from gnn_interfaces.msg import TrackedPolygon

BASE_DIR  = "/workspaces/isaac_ros-dev/dk_ros2_bags"
SRC_TOPIC = "/tracked_polygons"
DST_TOPIC = "/tracked_polygons_logged"

# Portable namespace: env ROBOT > hostname > "rm03" sentinel.
HOST = os.environ.get("ROBOT") or socket.gethostname() or "rm03"
NS   = f"/{HOST}"


def ns_topics():
    """Canonical 25-topic PARP set (see §9 of change_All_record_ros2bag.md)."""
    return [
        # 1. plans + path metrics
        f"{NS}/plan", f"{NS}/local_plan",
        f"{NS}/transformed_global_plan", f"{NS}/received_global_plan",
        # 2. costmaps (raw + compressed + updates)
        f"{NS}/global_costmap/costmap", f"{NS}/global_costmap/costmap_raw",
        f"{NS}/global_costmap/costmap_updates",
        f"{NS}/local_costmap/costmap",  f"{NS}/local_costmap/costmap_raw",
        # 3. perception input — RELAYED single-type
        DST_TOPIC,
        # 4. comms (L1) + battery (energy)
        f"{NS}/comms/link_stats", f"{NS}/fake_rsrp", f"{NS}/battery_state",
        # 5. route_cost outputs
        f"{NS}/route_cost", f"{NS}/route_puc", f"{NS}/route_puc_components",
        # 6. pose / ground truth
        f"{NS}/odom", f"{NS}/odom_vicon", f"{NS}/vicon_pose",
        f"{NS}/cmd_vel", f"{NS}/cmd_wheel_speed", f"{NS}/scan_filtered",
        # 7. tf + navigation action
        "/tf", "/tf_static",
        f"{NS}/behavior_tree_log",
        "/navigate_to_pose/feedback", "/navigate_to_pose/result",
        "/diagnostics",
    ]


TOPICS = ns_topics()


class Relay(Node):
    def __init__(self):
        super().__init__("tracked_polygons_relay")
        self.pub = self.create_publisher(TrackedPolygon, DST_TOPIC, 10)
        self.create_subscription(TrackedPolygon, SRC_TOPIC,
                                 lambda m: self.pub.publish(m), 10)
        self.get_logger().info(f"relaying {SRC_TOPIC} -> {DST_TOPIC}")


def main():
    sub = sys.argv[1] if len(sys.argv) > 1 else "R1"
    run = f"_{sub}_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}"

    # Per-host subdir + _bag suffix so the layout matches the launch (Path A).
    out_dir = os.path.join(BASE_DIR, HOST)
    os.makedirs(out_dir, exist_ok=True)
    out_uri = os.path.join(out_dir, f"{run}_bag")

    rclpy.init()
    node = Relay()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(2.0)  # let discovery advertise /tracked_polygons_logged before record starts

    cmd = ["ros2", "bag", "record", "-o", out_uri, "-s", "mcap"] + TOPICS
    print(f"[record_l3] host={HOST}  ns={NS}  run={run}")
    print(f"[record_l3] starting recorder ({len(TOPICS)} topics, MCAP) -> {out_uri}")
    proc = subprocess.Popen(cmd, start_new_session=True)

    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\n[record_l3] Ctrl+C -> finalizing bag...")
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.terminate()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[record_l3] bag saved: {out_uri}")


if __name__ == "__main__":
    main()