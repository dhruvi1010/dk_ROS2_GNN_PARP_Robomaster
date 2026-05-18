#!/usr/bin/env python3

#from /robomaster/isaac_nav/orchestrator_kub/scripts/fake_rsrp.py

#from https://gitlab.cc-asp.fraunhofer.de/robomaster/isaac_nav/-/blob/humble/orchestrator_kub/scripts/fake_rsrp.py?ref_type=heads

import numpy as np
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

def get_rsrp_fake(pos):
    x = pos[0]
   # x_min, x_max = 9.0, 19.0 #zft hall
    x_min, x_max = -8.0, 9.0 #flw hall
    rsrp_min, rsrp_max = -110, -70

    x_clamped = np.clip(x, x_min, x_max)
    rsrp = rsrp_min + ((x_clamped - x_min) / (x_max - x_min)) * (rsrp_max - rsrp_min)

    noise = np.random.uniform(-2, 2)
    rsrp_with_noise = rsrp + noise

    return rsrp_with_noise

class FakeRSRPNode(Node):
    def __init__(self):
        hostname = socket.gethostname().replace('-', '_')
        namespace = f'/{hostname}'
        super().__init__('fake_rsrp_node', namespace=namespace)

        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('fake_x', 0.0)

        self._fake_x = float(self.get_parameter('fake_x').value)

        self.publisher = self.create_publisher(
            Float32,
            f'{namespace}/fake_rsrp',
            10)

        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Publishing fake RSRP at {1.0/period:.1f} Hz with fake_x={self._fake_x}")

    def _tick(self):
        rsrp = get_rsrp_fake((self._fake_x, 0.0))
        self.get_logger().info(f'Publishing RSRP: {rsrp:.2f}')
        msg = Float32()
        msg.data = float(rsrp)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeRSRPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
