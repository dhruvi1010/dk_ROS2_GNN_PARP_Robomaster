import rclpy
from rclpy.node import Node
from gnn_interfaces.msg import TrackedPolygon
from geometry_msgs.msg import Point32
import math
import random
import numpy as np

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')
        self.publisher_ = self.create_publisher(TrackedPolygon, '/tracked_polygons', 10)
        self.timer = self.create_timer(10.0, self.publish_polygon)  # every 2 seconds
        self.step = 0
        self.rect_length = 6.0
        self.rect_width = 3.0
        self.speed = 1.0  # meters per step
        self.path_points = self._generate_rectangle_path()

    def _generate_rectangle_path(self):
        points = []

        # 1. Right along X
        for x in range(-int(self.rect_length), int(self.rect_length), int(self.speed)):
            points.append((x, 0))

        # 2. Up along Y
        for y in range(-int(self.rect_width), int(self.rect_width), int(self.speed)):
            points.append((self.rect_length, y))

        # 3. Left along X
        for x in reversed(range(-int(self.rect_length), int(self.rect_length), int(self.speed))):
            points.append((x, self.rect_width))

        # 4. Down along Y
        for y in reversed(range(-int(self.rect_width), int(self.rect_width), int(self.speed))):
            points.append((0, y))

        return points

    def _generate_random_polygon(self, cx, cy):
        num_vertices = random.randint(3, 8)
        radius = random.uniform(0.3, 1.2)
        angle_offset = random.uniform(0, 2 * math.pi)

        points = []
        for i in range(num_vertices):
            angle = 2 * math.pi * i / num_vertices + angle_offset
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append(Point32(x=x, y=y))

        return points

    def publish_polygon(self):
        if not self.path_points:
            return

        cx, cy = self.path_points[self.step % len(self.path_points)]

        msg = TrackedPolygon()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.polygon.points = self._generate_random_polygon(cx, cy)
        msg.label = random.randint(1, 4)  # labels 1–5
        msg.confidence = round(random.uniform(0.7, 0.95), 3)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published label={msg.label} conf={msg.confidence:.2f} at ({cx:.2f}, {cy:.2f}) with {len(msg.polygon.points)} pts"
        )

        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = PolygonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
