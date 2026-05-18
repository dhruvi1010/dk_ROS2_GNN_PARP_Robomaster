#!/usr/bin/env python3

#from /robomaster/isaac_nav/orchestrator_kub/scripts/fake_rsrp.py

#from https://gitlab.cc-asp.fraunhofer.de/robomaster/isaac_nav/-/blob/humble/orchestrator_kub/scripts/fake_rsrp.py?ref_type=heads

import numpy as np
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
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

        topic_to_subscribe = f'{namespace}/vicon_pose'
        self.get_logger().info(f"Subscribing to topic: {topic_to_subscribe}")

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            topic_to_subscribe,
            self.pose_callback,
            10)

        self.publisher = self.create_publisher(
            Float32,
            f'{namespace}/fake_rsrp',
            10)

        #self.get_logger().info("Node started with namespace topics.")
        self.get_logger().info(f"Node started. Namespace: {namespace}, Subscribing: {topic_to_subscribe}, Publishing: {namespace}/fake_rsrp")

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.get_logger().info(f'Received pose: x={x}, y={y}, z={z}')
        rsrp = get_rsrp_fake((x, y))
        self.get_logger().info(f'Publishing RSRP: {rsrp:.2f}')
        msg_out = Float32()
        msg_out.data = rsrp
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = FakeRSRPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
