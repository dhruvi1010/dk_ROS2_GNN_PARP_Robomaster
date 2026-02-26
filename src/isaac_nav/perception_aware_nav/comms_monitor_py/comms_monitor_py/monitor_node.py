#!/usr/bin/env python3
"""
monitor_node.py

Features:
1) Measures RTT via UDP to an echo server (works with your current udp_echo_server.py).
2) Estimates uplink/downlink latency:
   - If the server supports timestamp replies (see notes below) AND clocks are NTP-synced,
     it will compute one-way uplink/downlink.
   - Otherwise it falls back to RTT/2.
3) Subscribes to RSRP from /(namespace)/fake_rsrp by subscribing to the *relative* topic name
   "fake_rsrp" (namespacing is handled by your launch PushRosNamespace).

Notes on message fields:
- Your current LinkStats.msg has: rsrp_dbm, jitter_ms, rtt_ms, loss_rate.
- If you extend LinkStats.msg with:
    float32 uplink_ms
    float32 downlink_ms
  then this node will automatically fill those fields too.
  (It uses hasattr() so it still runs even if you keep the old message.)
"""

import time
import socket
import struct
import statistics

import rclpy
from rclpy.node import Node

from perception_aware_msgs.msg import LinkStats
from std_msgs.msg import Float32


class CommsMonitor(Node):
    # Client request:  (seq:uint32, t1_epoch_ns:uint64)
    REQ_FMT = "!IQ"
    # Server reply (optional): (seq:uint32, t1_epoch_ns:uint64, t2_srv_rx_ns:uint64, t3_srv_tx_ns:uint64)
    REP_FMT = "!IQQQ"

    def __init__(self):
        super().__init__("comms_monitor")

        # --- PARAMETERS ---
        self.declare_parameter("edge_ip", "172.16.3.62")
        self.declare_parameter("edge_port", 5005)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("window_size", 20)
        self.declare_parameter("udp_timeout_s", 0.2)

        # RSRP topic (relative -> becomes /<namespace>/fake_rsrp due to PushRosNamespace)
        self.declare_parameter("rsrp_topic", "fake_rsrp")

        # If timestamp-based one-way is not possible / not sane, approximate uplink=downlink=RTT/2
        self.declare_parameter("use_symmetric_fallback", True)

        self.edge_ip = str(self.get_parameter("edge_ip").value)
        self.edge_port = int(self.get_parameter("edge_port").value)
        self.window_size = int(self.get_parameter("window_size").value)
        self.udp_timeout_s = float(self.get_parameter("udp_timeout_s").value)
        self.use_symmetric_fallback = bool(self.get_parameter("use_symmetric_fallback").value)

        # --- PUB/SUB ---
        self.pub = self.create_publisher(LinkStats, "comms/link_stats", 10)

        self.rsrp_dbm_latest = None
        rsrp_topic = str(self.get_parameter("rsrp_topic").value)
        self.create_subscription(Float32, rsrp_topic, self._rsrp_cb, 10)

        # --- UDP SOCKET ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.udp_timeout_s)

        # --- STATS WINDOWS ---
        self.rtt_history_ms = []
        self.success_history = []  # 1 if success, 0 if timeout/error
        self.seq = 0

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"Monitor running. Target: {self.edge_ip}:{self.edge_port}, rsrp_topic='{rsrp_topic}'"
        )

    def _rsrp_cb(self, msg: Float32):
        self.rsrp_dbm_latest = float(msg.data)

    def get_rsrp(self) -> float:
        return float(self.rsrp_dbm_latest) if self.rsrp_dbm_latest is not None else -1.0

    def measure_latencies(self):
        """
        Returns (ok, rtt_ms, uplink_ms, downlink_ms)

        Works with plain echo server:
          - RTT is measured.
          - uplink/downlink fallback to RTT/2.

        If server returns timestamp reply (REP_FMT) AND clocks are NTP-synced:
          uplink_ms  = t2_server_rx - t1_client_tx
          downlink_ms = t4_client_rx - t3_server_tx

        If timestamps look insane (clock not synced), fallback to RTT/2 (if enabled).
        """
        self.seq = (self.seq + 1) & 0xFFFFFFFF

        # epoch time for one-way calc, monotonic for RTT (more stable)
        t1_epoch_ns = time.time_ns()
        t1_mono_ns = time.perf_counter_ns()

        req = struct.pack(self.REQ_FMT, self.seq, t1_epoch_ns)

        try:
            self.sock.sendto(req, (self.edge_ip, self.edge_port))
            data, _ = self.sock.recvfrom(2048)
        except socket.timeout:
            return False, None, None, None
        except Exception:
            return False, None, None, None

        t4_mono_ns = time.perf_counter_ns()
        t4_epoch_ns = time.time_ns()

        rtt_ms = (t4_mono_ns - t1_mono_ns) / 1e6

        uplink_ms = None
        downlink_ms = None

        # Try to parse timestamp reply
        rep_size = struct.calcsize(self.REP_FMT)
        if len(data) >= rep_size:
            try:
                seq_rx, t1_rx, t2_srv, t3_srv = struct.unpack(self.REP_FMT, data[:rep_size])
                if seq_rx == self.seq and t1_rx == t1_epoch_ns:
                    ul_ns = int(t2_srv - t1_epoch_ns)
                    dl_ns = int(t4_epoch_ns - t3_srv)

                    # Sanity bounds: if clocks aren't synced, ul/dl can be negative/huge
                    sane = (0 <= ul_ns < 2_000_000_000) and (0 <= dl_ns < 2_000_000_000)
                    if sane:
                        uplink_ms = ul_ns / 1e6
                        downlink_ms = dl_ns / 1e6
            except Exception:
                pass

        # Fallback: approximate one-way delays with RTT/2
        if (uplink_ms is None or downlink_ms is None) and self.use_symmetric_fallback and rtt_ms is not None:
            uplink_ms = rtt_ms / 2.0
            downlink_ms = rtt_ms / 2.0

        return True, rtt_ms, uplink_ms, downlink_ms

    def tick(self):
        ok, rtt_ms, uplink_ms, downlink_ms = self.measure_latencies()

        # Loss tracking
        self.success_history.append(1 if ok else 0)
        if len(self.success_history) > self.window_size:
            self.success_history.pop(0)

        if self.success_history:
            loss_rate = 1.0 - (sum(self.success_history) / float(len(self.success_history)))
        else:
            loss_rate = 1.0

        # RTT window for jitter
        if ok and rtt_ms is not None:
            self.rtt_history_ms.append(float(rtt_ms))
            if len(self.rtt_history_ms) > self.window_size:
                self.rtt_history_ms.pop(0)

        if len(self.rtt_history_ms) > 2:
            jitter_ms = float(statistics.stdev(self.rtt_history_ms))
            avg_rtt_ms = float(statistics.mean(self.rtt_history_ms))
        elif len(self.rtt_history_ms) == 1:
            jitter_ms = 0.0
            avg_rtt_ms = float(self.rtt_history_ms[0])
        else:
            jitter_ms = 0.0
            avg_rtt_ms = -1.0

        rsrp = self.get_rsrp()

        msg = LinkStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.rsrp_dbm = float(rsrp)
        msg.jitter_ms = float(jitter_ms)
        msg.rtt_ms = float(avg_rtt_ms)
        msg.loss_rate = float(loss_rate)

        # Only set if your LinkStats.msg includes these fields
        if hasattr(msg, "uplink_ms"):
            msg.uplink_ms = float(uplink_ms) if uplink_ms is not None else -1.0
        if hasattr(msg, "downlink_ms"):
            msg.downlink_ms = float(downlink_ms) if downlink_ms is not None else -1.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommsMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# import rclpy
# from rclpy.node import Node
# from perception_aware_msgs.msg import LinkStats
# import time
# import socket
# import subprocess
# import statistics
# import re

# class CommsMonitor(Node):
#     def __init__(self):
#         super().__init__('comms_monitor')
        
#         # --- CONFIGURATION ---
#         self.declare_parameter('edge_ip', '172.16.3.62') # Your Edge IP
#         self.declare_parameter('edge_port', 5005)
#         self.declare_parameter('interface', 'wwan0')
#         self.declare_parameter('rate_hz', 5.0)
#         self.declare_parameter('window_size', 20)

#         self.edge_ip = self.get_parameter('edge_ip').value
#         self.edge_port = int(self.get_parameter('edge_port').value)
#         self.iface = self.get_parameter('interface').value
#         self.window_size = int(self.get_parameter('window_size').value)

#         self.pub = self.create_publisher(LinkStats, 'comms/link_stats', 10)
#         self.rtt_history = []
        
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.settimeout(0.2)

#         period = 1.0 / self.get_parameter('rate_hz').value
#         self.timer = self.create_timer(period, self.tick)
        
#         self.get_logger().info(f"Monitor running. Target: {self.edge_ip}")

#     def get_rsrp(self):
#         """
#         Attempts to read 5G RSRP. If modem hides it, returns -1.0.
#         """
#         try:
#             output = subprocess.check_output(['mmcli', '-m', 'any', '--signal-get'], text=True)
#             # Look for rsrp: -85.0
#             match = re.search(r'rsrp:\s*(-?\d+(\.\d+)?)', output)
#             if match:
#                 return float(match.group(1))
#         except Exception:
#             pass
        
#         # Fallback: Check general status for a % value
#         try:
#             output = subprocess.check_output(['mmcli', '-m', 'any'], text=True)
#             # Look for "signal quality: 75%"
#             match = re.search(r'signal quality:\s*(\d+)%', output)
#             if match:
#                 qual = float(match.group(1))
#                 # Approx conversion: 100% = -50dBm, 0% = -110dBm
#                 return -110.0 + (qual * 0.6)
#         except Exception:
#             pass

#         return -1.0 # Indicator for "Unknown/Hidden"

#     def measure_rtt(self):
#         try:
#             msg = b'ping'
#             start_time = time.time()
#             self.sock.sendto(msg, (self.edge_ip, self.edge_port))
#             data, addr = self.sock.recvfrom(1024)
#             end_time = time.time()
#             return (end_time - start_time) * 1000.0
#         except socket.timeout:
#             return None
#         except Exception as e:
#             # self.get_logger().warn(f"Ping error: {e}")
#             return None

#     def tick(self):
#         # 1. Measure RTT (Jitter)
#         rtt = self.measure_rtt()
        
#         if rtt is not None:
#             self.rtt_history.append(rtt)
        
#         if len(self.rtt_history) > self.window_size:
#             self.rtt_history.pop(0)

#         if len(self.rtt_history) > 2:
#             jitter = statistics.stdev(self.rtt_history)
#             avg_rtt = statistics.mean(self.rtt_history)
#         else:
#             jitter = 0.0
#             avg_rtt = 0.0 if not rtt else rtt

#         # 2. Get Signal (Best Effort)
#         #rsrp = self.get_rsrp()
#         rsrp = -1.0 # Placeholder since modem hides it. Implementing a fake value for testing.

#         # 3. Publish
#         msg = LinkStats()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "base_link"
#         msg.rsrp_dbm = float(rsrp)
#         msg.jitter_ms = float(jitter)
#         msg.rtt_ms = float(avg_rtt if rtt else -1.0)
        
#         self.pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CommsMonitor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

