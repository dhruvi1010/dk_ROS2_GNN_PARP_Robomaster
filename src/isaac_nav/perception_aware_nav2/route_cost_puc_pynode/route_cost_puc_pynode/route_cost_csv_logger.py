#!/usr/bin/env python3
"""
Subscribes to /rm03/route_cost, /rm03/route_puc, /rm03/route_puc_components
and writes one CSV row per planner cycle to ~/runs/<run_id>.csv.

Joins rows by approximate header.stamp; latest seen component values are used
for each route_cost row (PUC publishes alongside route_cost from the same node
so they almost always arrive together).
"""
import csv
import os
import socket
from datetime import datetime
from threading import Lock

import rclpy
from rclpy.node import Node
from perception_aware_nav2_msgs.msg import RouteCost, RoutePUC, RoutePUCComponents


class RouteCostCsvLogger(Node):
    HEADER = [
        'wall_iso', 'sec', 'nanosec', 'robot_id', 'trial_id',
        'j_total', 'time_term', 'obs_term', 'comms_term', 'safety_term', 'energy_term',
        'path_length_m', 'nominal_speed_mps', 'soc_fraction',
        'lambda_obs', 'lambda_comms', 'lambda_safety', 'lambda_energy',
        'puc', 'p_t', 'p_o', 'p_c', 'p_s', 'p_e',
        'w_t', 'w_o', 'w_c', 'w_s', 'w_e',
        'n_obstacles', 'nearest_obstacle_m', 'rsrp_dbm', 'jitter_ms', 'min_ttc_s',
    ]

    def __init__(self):
        super().__init__('route_cost_csv_logger')
        hostname = socket.gethostname().replace('-', '_')
        namespace = f'/{hostname}'
        super().__init__('fake_rsrp_node', namespace=namespace)
        self.declare_parameter('run_id', 'default_run')
        #self.declare_parameter('output_dir', os.path.expanduser('~/runs'))
        self.declare_parameter('output_dir', '/workspaces/isaac_ros-dev/dk_ros2_bags')
        self.run_id = self.get_parameter('run_id').value
        outdir = self.get_parameter('output_dir').value
        os.makedirs(outdir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(outdir, f'{self.run_id}_{ts}.csv')

        self._lock = Lock()
        self._last_puc = None
        self._last_comp = None

        self._f = open(self.path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)
        self._f.flush()

        self.create_subscription(RouteCost, '{namespace}/route_cost', self.on_cost, 20)
        self.create_subscription(RoutePUC, '{namespace}/route_puc', self.on_puc, 20)
        self.create_subscription(RoutePUCComponents,
                                 '{namespace}/route_puc_components', self.on_comp, 20)
        self.get_logger().info(f'\033[92mCSV logging to {self.path}\033[0m')


    def on_puc(self, msg):
        with self._lock:
            self._last_puc = msg

    def on_comp(self, msg):
        with self._lock:
            self._last_comp = msg

    def on_cost(self, msg):
        with self._lock:
            puc = self._last_puc
            comp = self._last_comp
        wall_iso = datetime.now().isoformat()
        row = [
            wall_iso, msg.header.stamp.sec, msg.header.stamp.nanosec,
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


def main(args=None):
    rclpy.init(args=args)
    node = RouteCostCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
