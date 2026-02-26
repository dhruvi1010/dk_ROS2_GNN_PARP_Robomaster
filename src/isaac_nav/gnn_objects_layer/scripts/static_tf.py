#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.broadcaster = StaticTransformBroadcaster(self)

        # Create and send static transform
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'rm04/odom'
        static_transform_stamped.child_frame_id = 'rm04/base_footprint'
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info('✅ Static transform published: rm04/odom → rm04/base_footprint')

rclpy.init()
node = StaticTFBroadcaster()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
rclpy.shutdown()
