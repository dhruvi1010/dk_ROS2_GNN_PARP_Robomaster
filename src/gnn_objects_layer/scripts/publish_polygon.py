import rclpy
from rclpy.node import Node
from gnn_interfaces.msg import TrackedPolygon
from geometry_msgs.msg import Point32
import math
import random


class PolygonPublisher(Node):
    def __init__(self, num_vertices=6):
        super().__init__('polygon_publisher')
        self.publisher_ = self.create_publisher(TrackedPolygon, '/tracked_polygons', 10)
        self.timer = self.create_timer(2.0, self.publish_polygons)  # Every 2 seconds
        self.step = 0
        self.num_vertices = max(num_vertices, 3)  # prevent zero or invalid value
        self.fixed_radius = 0.5  # consistent polygon size

        spacing = 2.5  # new spacing
        self.quadrant_centers = {
            1: (spacing, spacing),     # Top-right
            2: (-spacing, spacing),    # Top-left
            3: (-spacing, -spacing),   # Bottom-left
            4: (spacing, -spacing)     # Bottom-right
        }

    def _generate_polygon(self, cx, cy):
        angle_offset = random.uniform(0, 2 * math.pi)
        points = []
        for i in range(self.num_vertices):
            angle = 2 * math.pi * i / self.num_vertices + angle_offset
            x = cx + self.fixed_radius * math.cos(angle)
            y = cy + self.fixed_radius * math.sin(angle)
            points.append(Point32(x=x, y=y))
        return points

    def publish_polygons(self):
        for label, (cx, cy) in self.quadrant_centers.items():
            msg = TrackedPolygon()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.polygon.points = self._generate_polygon(cx, cy)
            msg.label = label
            msg.confidence = round(random.uniform(0.7, 0.95), 3)

            self.publisher_.publish(msg)
            self.get_logger().info(
                f"Published label={msg.label} conf={msg.confidence:.2f} at ({cx:.2f}, {cy:.2f}) with {len(msg.polygon.points)} pts"
            )

        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    # You can pass num_vertices=30 to simulate a circle
    node = PolygonPublisher(num_vertices=3)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
