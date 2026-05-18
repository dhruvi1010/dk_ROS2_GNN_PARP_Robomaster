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
from std_msgs.msg import Float32
import time, subprocess, statistics, socket, statistics, struct

class CommsMonitor(Node):
    # Client request:  (seq:uint32, t1_epoch_ns:uint64)
    REQ_FMT = "!IQ"
    # Server reply (optional): (seq:uint32, t1_epoch_ns:uint64, t2_srv_rx_ns:uint64, t3_srv_tx_ns:uint64)
    REP_FMT = "!IQQQ"

    def __init__(self):
        super().__init__('comms_monitor')

        # --- Parameters ---
        self.declare_parameter('edge_ip', '172.16.3.62')   # 5G server default
        self.declare_parameter('edge_port', 5005)
        #self.declare_parameter('iface', 'wlan0')   # default Wi-Fi interface
        self.declare_parameter('rate_hz', 5.0)      #  how often node measures and publishes communication link statistics (RTT and signal strength)
        ##can be tune using: ros2 run comms_monitor_pynode comms_monitor_pynode --ros-args -p rate_hz:=10.0
        self.declare_parameter("window_size", 20)
        self.declare_parameter("udp_timeout_s", 0.2)

        # Namespaced automatically by PushRosNamespace in your launch
        self.declare_parameter("rsrp_topic", "fake_rsrp")


        # If timestamp-based one-way is not possible / not sane -> RTT/2 fallback
        self.declare_parameter("use_symmetric_fallback", True)

        # Optional: publish averaged uplink/downlink as topics (even if LinkStats.msg has no fields)
        # self.declare_parameter("publish_oneway_topics", False)
        # self.declare_parameter("uplink_topic", "comms/uplink_ms")
        # self.declare_parameter("downlink_topic", "comms/downlink_ms")

    
        ## did not include time out as of now, but can be added as needed
        #self.declare_parameter('socket_timeout_s', 0.2)

        #  --- Robot ID and link source (for multi-robot setups and potential future use of cellular stats)
        self.declare_parameter('robot_id',      os.environ.get('ROBOT', 'unknown'))
        #self.declare_parameter('link_source',   'auto')   # 'wifi' | 'modem' | 'auto'

        
        # Read parameters ----
        self.edge_ip        = self.get_parameter('edge_ip').value
        self.edge_port      = int(self.get_parameter('edge_port').value)
        #self.iface          = self.get_parameter('iface').value
        self.window_size    = int(self.get_parameter('window_size').value)
        self.udp_timeout_s = float(self.get_parameter("udp_timeout_s").value)
        self.use_symmetric_fallback = bool(self.get_parameter("use_symmetric_fallback").value)

        # self.publish_oneway_topics = bool(self.get_parameter("publish_oneway_topics").value)
        # self.uplink_topic = str(self.get_parameter("uplink_topic").value)
        # self.downlink_topic = str(self.get_parameter("downlink_topic").value)

        #self.socket_timeout = float(self.get_parameter('socket_timeout_s').value)
        self.robot_id       = self.get_parameter('robot_id').value
        #self.link_source    = self.get_parameter('link_source').value
        rate_hz             = float(self.get_parameter('rate_hz').value)

        
        # --- Pub/Sub ---
        self.pub = self.create_publisher(LinkStats, "comms/link_stats", 10)

        # self.pub_ul = None
        # self.pub_dl = None
        # if self.publish_oneway_topics:
        #     self.pub_ul = self.create_publisher(Float32, self.uplink_topic, 10)
        #     self.pub_dl = self.create_publisher(Float32, self.downlink_topic, 10)

        self.rsrp_dbm_latest = None
        rsrp_topic = str(self.get_parameter("rsrp_topic").value)
        self.create_subscription(Float32, rsrp_topic, self._rsrp_cb, 10)


        # --UDP Socket Setup---

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.udp_timeout_s)


        # --- Sliding windows ---
        self.rtt_hist_ms = []
        self.ul_hist_ms = []
        self.dl_hist_ms = []
        self.success_hist = []  # 1 on success, 0 on timeout/error

        self.seq = 0

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"Monitor running. Target={self.edge_ip}:{self.edge_port}, rsrp_topic='{rsrp_topic}', window={self.window_size}"
        )
        
        # # remove for clean op
        # self.get_logger().info(
        #     f"[comms_monitor] robot_id={self.robot_id} "
        #     f"edge={self.edge_ip}:{self.edge_port} iface={self.iface} "
        #     f"rate={rate_hz:.1f}Hz window={self.window_size} source={self.link_source}"
        # )

        # period = 1.0 / float(self.get_parameter('rate_hz').value)
        # self.timer = self.create_timer(period, self.tick)

    def _rsrp_cb(self, msg: Float32):
        self.rsrp_dbm_latest = float(msg.data)

    def get_rsrp(self) -> float:
        return float(self.rsrp_dbm_latest) if self.rsrp_dbm_latest is not None else -1.0

    def _push_window(self, window_list, value: float):
        window_list.append(float(value))
        if len(window_list) > self.window_size:
            window_list.pop(0)

    def measure_latencies(self):
        """
        Returns (ok, rtt_ms, uplink_ms, downlink_ms).

        RTT: always measured (monotonic clock).
        One-way:
          - If server supports REP_FMT timestamps and clocks are synced -> true one-way estimate.
          - Else fallback to RTT/2 (if enabled).
        """
        self.seq = (self.seq + 1) & 0xFFFFFFFF

        # Epoch time for one-way (needs NTP), monotonic for RTT stability
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

        # Try parse timestamp reply
        rep_size = struct.calcsize(self.REP_FMT)
        if len(data) >= rep_size:
            try:
                seq_rx, t1_rx, t2_srv, t3_srv = struct.unpack(self.REP_FMT, data[:rep_size])
                if seq_rx == self.seq and t1_rx == t1_epoch_ns:
                    ul_ns = int(t2_srv - t1_epoch_ns)
                    dl_ns = int(t4_epoch_ns - t3_srv)

                    # sanity bounds (if clocks not synced, ul/dl can be negative/huge)
                    sane = (0 <= ul_ns < 2_000_000_000) and (0 <= dl_ns < 2_000_000_000)
                    if sane:
                        uplink_ms = ul_ns / 1e6
                        downlink_ms = dl_ns / 1e6
            except Exception:
                pass

        # Fallback: approximate one-way by RTT/2
        if (uplink_ms is None or downlink_ms is None) and self.use_symmetric_fallback and rtt_ms is not None:
            uplink_ms = rtt_ms / 2.0
            downlink_ms = rtt_ms / 2.0

        return True, rtt_ms, uplink_ms, downlink_ms

    def tick(self):
        ok, rtt_ms, uplink_ms, downlink_ms = self.measure_latencies()

        # success/loss window
        self._push_window(self.success_hist, 1.0 if ok else 0.0)
        loss_rate = 1.0 - (sum(self.success_hist) / float(len(self.success_hist))) if self.success_hist else 1.0

        # Only push latency windows on successful measurement
        if ok and rtt_ms is not None:
            self._push_window(self.rtt_hist_ms, rtt_ms)

        if ok and uplink_ms is not None and downlink_ms is not None:
            self._push_window(self.ul_hist_ms, uplink_ms)
            self._push_window(self.dl_hist_ms, downlink_ms)

        # RTT avg + jitter (stddev)
        if len(self.rtt_hist_ms) >= 2:
            avg_rtt_ms = float(statistics.mean(self.rtt_hist_ms))
            jitter_ms = float(statistics.stdev(self.rtt_hist_ms))
        elif len(self.rtt_hist_ms) == 1:
            avg_rtt_ms = float(self.rtt_hist_ms[0])
            jitter_ms = 0.0
        else:
            avg_rtt_ms = -1.0
            jitter_ms = 0.0

        # One-way averages
        avg_ul_ms = float(statistics.mean(self.ul_hist_ms)) if self.ul_hist_ms else -1.0
        avg_dl_ms = float(statistics.mean(self.dl_hist_ms)) if self.dl_hist_ms else -1.0

        # RSRP
        rsrp = self.get_rsrp()

        # Publish LinkStats
        msg = LinkStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.rsrp_dbm = float(rsrp)
        msg.jitter_ms = float(jitter_ms)
        msg.rtt_ms = float(avg_rtt_ms)
        msg.loss_rate = float(loss_rate)
        msg.robot_id = self.robot_id

        # If you later extend LinkStats.msg to include these, they will be filled automatically
        if hasattr(msg, "uplink_ms"):
            msg.uplink_ms = float(avg_ul_ms)
        if hasattr(msg, "downlink_ms"):
            msg.downlink_ms = float(avg_dl_ms)

        self.pub.publish(msg)

        # Optional separate topics (useful even with old LinkStats.msg)
        # if self.publish_oneway_topics:
        #     ul_msg = Float32()
        #     dl_msg = Float32()
        #     ul_msg.data = float(avg_ul_ms)
        #     dl_msg.data = float(avg_dl_ms)
        #     self.pub_ul.publish(ul_msg)
        #     self.pub_dl.publish(dl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommsMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
