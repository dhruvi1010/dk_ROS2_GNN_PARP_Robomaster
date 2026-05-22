#!/usr/bin/env python3
"""Periodic comms-link monitor.

- Sends a small UDP packet to an edge echo server, 
- measures RTT, maintains a sliding-window RTT history to derive jitter, 
- and reads RSSI/RSRP from the OS
- (Wi-Fi via `iw`, cellular via `mmcli` if available).  

- Publishes a perception_aware_msgs/LinkStats message on the relative topic
``comms/link_stats`` (so multi-robot namespaces work automatically).
"""

import math
import os
import socket
import statistics
import subprocess
import time

import rclpy
from rclpy.node import Node
from perception_aware_nav2_msgs.msg import LinkStats


class CommsMonitor(Node):
    def __init__(self):
        super().__init__('comms_monitor')

        # --- parameters -------------------------------------------------------
        self.declare_parameter('edge_ip',       '172.16.3.62')  # 5G server default
        self.declare_parameter('edge_port',     5005)
        self.declare_parameter('iface',         'ens34')
        self.declare_parameter('rate_hz',       5.0)
        self.declare_parameter('window_size',   20)
        self.declare_parameter('socket_timeout_s', 0.2)
        self.declare_parameter('robot_id',      os.environ.get('ROBOT', 'unknown'))
        self.declare_parameter('link_source',   'auto')   # 'wifi' | 'modem' | 'auto'

        self.edge_ip        = self.get_parameter('edge_ip').value
        self.edge_port      = int(self.get_parameter('edge_port').value)
        self.iface          = self.get_parameter('iface').value
        self.window_size    = int(self.get_parameter('window_size').value)
        self.socket_timeout = float(self.get_parameter('socket_timeout_s').value)
        self.robot_id       = self.get_parameter('robot_id').value
        self.link_source    = self.get_parameter('link_source').value
        rate_hz             = float(self.get_parameter('rate_hz').value)

        # --- state ------------------------------------------------------------
        self.rtt_window = []
        self.tx_count   = 0
        self.rx_count   = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.socket_timeout)

        self.pub = self.create_publisher(LinkStats, 'comms/link_stats', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"[comms_monitor] robot_id={self.robot_id} "
            f"edge={self.edge_ip}:{self.edge_port} iface={self.iface} "
            f"rate={rate_hz:.1f}Hz window={self.window_size} source={self.link_source}"
        )

    # ---------------- link-layer signal readout -------------------------------
    def _read_rsrp_wifi(self):
        try:
            out = subprocess.check_output(
                ['iw', 'dev', self.iface, 'link'],
                text=True, stderr=subprocess.DEVNULL, timeout=0.3,
            )
            for line in out.splitlines():
                s = line.strip()
                if s.startswith('signal:'):
                    return float(s.split('signal:')[1].split('dBm')[0].strip())
        except Exception:
            pass
        return float('nan')

    def _read_rsrp_modem(self):
        # ModemManager: first modem, --signal-get prints e.g.  "rsrp: -82.00 dBm"
        try:
            out = subprocess.check_output(
                ['mmcli', '-m', '0', '--signal-get'],
                text=True, stderr=subprocess.DEVNULL, timeout=0.3,
            )
            for line in out.splitlines():
                s = line.strip().lower()
                if 'rsrp' in s:
                    # example: "rsrp: -82.00 dBm"
                    parts = s.split(':', 1)[1].strip().split()
                    return float(parts[0])
        except Exception:
            pass
        return float('nan')

    def _read_rsrp_dbm(self):
        if self.link_source == 'wifi':
            return self._read_rsrp_wifi()
        if self.link_source == 'modem':
            return self._read_rsrp_modem()
        # auto: try modem first (real 5G), fall back to wifi
        v = self._read_rsrp_modem()
        if not math.isnan(v):
            return v
        return self._read_rsrp_wifi()

    # ---------------- UDP echo probe ------------------------------------------
    def _probe_rtt_ms(self):
        self.tx_count += 1
        try:
            t0 = time.monotonic()
            self.sock.sendto(b'p', (self.edge_ip, self.edge_port))
            self.sock.recvfrom(128)
            self.rx_count += 1
            return (time.monotonic() - t0) * 1000.0
        except socket.timeout:
            return None
        except Exception as e:
            self.get_logger().warn(f"UDP probe failed: {e}", throttle_duration_sec=5.0)
            return None

    # ---------------- periodic publish ----------------------------------------
    def _tick(self):
        rtt = self._probe_rtt_ms()
        if rtt is not None:
            self.rtt_window.append(rtt)
            self.rtt_window = self.rtt_window[-self.window_size:]

        msg = LinkStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_id
        msg.robot_id = self.robot_id
        msg.rsrp_dbm = float(self._read_rsrp_dbm())
        msg.rtt_ms = float(statistics.mean(self.rtt_window)) if self.rtt_window else float('nan')
        msg.jitter_ms = (
            float(statistics.pstdev(self.rtt_window)) if len(self.rtt_window) >= 3
            else float('nan')
        )
        msg.loss_rate = float(1.0 - (self.rx_count / max(self.tx_count, 1)))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()