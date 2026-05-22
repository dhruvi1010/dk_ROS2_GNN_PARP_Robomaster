#!/usr/bin/env python3

"""
comms_monitor_pynode.py   by dk  with tobot id: 

What it does
- Publishes LinkStats on:  <ns>/comms/link_stats
- Subscribes RSRP from:    <ns>/fake_rsrp   (relative name "fake_rsrp" so namespace applies)
- Measures UDP RTT to edge server, and estimates uplink/downlink:
    * If UDP server supports timestamp replies AND clocks are NTP-synced -> one-way estimates
    * Otherwise -> fallback uplink = downlink = RTT/2
- Averages RTT + uplink + downlink over a sliding window (window_size)
- Computes RTT jitter as stddev over the same window
- Computes loss_rate over the same window (timeouts/errors)

Compatibility
- Your current LinkStats.msg contains only: rsrp_dbm, jitter_ms, rtt_ms, loss_rate.
- If you extend LinkStats.msg later with:
      float32 uplink_ms
      float32 downlink_ms
  this node will automatically fill them (it uses hasattr()).
- Optionally, it can also publish averaged uplink/downlink as Float32 topics.
"""
import rclpy
import os
from rclpy.node import Node
from perception_aware_nav2_msgs.msg import LinkStats
import time, subprocess, statistics, socket, statistics

class CommsMonitor(Node):
    def __init__(self):
        super().__init__('comms_monitor')

        # --- Parameters ---
        self.declare_parameter('edge_ip', '172.16.3.62')   # 5G server default
        self.declare_parameter('edge_port', 5005)
        self.declare_parameter('iface', 'wlan0')   # default Wi-Fi interface
        self.declare_parameter('rate_hz', 5.0)      #  how often node measures and publishes communication link statistics (RTT and signal strength)
        ##can be tune using: ros2 run comms_monitor_pynode comms_monitor_pynode --ros-args -p rate_hz:=10.0
        self.declare_parameter('window_size', 20)
                ## did not include time out as of now, but can be added as needed
        self.declare_parameter('socket_timeout_s', 0.2)

        #  --- Robot ID and link source (for multi-robot setups and potential future use of cellular stats)
        self.declare_parameter('robot_id',      os.environ.get('ROBOT', 'unknown'))
        self.declare_parameter('link_source',   'auto')   # 'wifi' | 'modem' | 'auto'

        
        # Read parameters ----
        self.edge_ip        = self.get_parameter('edge_ip').value
        self.edge_port      = int(self.get_parameter('edge_port').value)
        self.iface          = self.get_parameter('iface').value
        self.window_size    = int(self.get_parameter('window_size').value)
        self.socket_timeout = float(self.get_parameter('socket_timeout_s').value)
        self.robot_id       = self.get_parameter('robot_id').value
        self.link_source    = self.get_parameter('link_source').value
        rate_hz             = float(self.get_parameter('rate_hz').value)

        
        #--- State ---
        self.rtts = []

        # --UDP Socket Setup---

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.socket_timeout)

        self.pub = self.create_publisher(LinkStats, '/comms/link_stats', 10)
        
        # remove for clean op
        self.get_logger().info(
            f"[comms_monitor] robot_id={self.robot_id} "
            f"edge={self.edge_ip}:{self.edge_port} iface={self.iface} "
            f"rate={rate_hz:.1f}Hz window={self.window_size} source={self.link_source}"
        )

        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(period, self.tick)

    def read_rsrp_dbm(self):
        # Wi-Fi RSSI as proxy
        try:
            out = subprocess.check_output(['iw', 'dev', self.iface, 'link'], text=True)
            for line in out.splitlines():
                if 'signal:' in line:
                    # e.g. "signal: -52 dBm"
                    return float(line.split('signal:')[1].split('dBm')[0].strip())
        except Exception:
            pass
        return float('nan')

    def probe_rtt(self):
        try:
            payload = b'ping'
            t0 = time.time()
            self.sock.sendto(payload, (self.edge_ip, self.edge_port))
            _data, _addr = self.sock.recvfrom(1024)
            return (time.time() - t0) * 1000.0
        except Exception:
            return None

    def tick(self):
        rtt = self.probe_rtt()
        if rtt is not None:
            self.rtts.append(rtt)
            self.rtts = self.rtts[-self.window_size:]
        msg = LinkStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_id
        msg.rsrp_dbm = self.read_rsrp_dbm()
        msg.rtt_ms = float(statistics.mean(self.rtts)) if self.rtts else float('nan')
        msg.jitter_ms = float(statistics.pstdev(self.rtts)) if len(self.rtts) >= 3 else float('nan')
        msg.loss_rate = 0.0  # you can compute if you track sent vs received
        msg.robot_id = self.robot_id
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CommsMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()