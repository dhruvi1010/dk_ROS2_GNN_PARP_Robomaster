#!/usr/bin/env python3

import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

WAYPOINTS = [
    {
        "name": "workstation",
        "x": 16.237928378863,
        "y": 1.640261300240867,
        "orientation": {
            "x": -0.005759263101505982,
            "y": 0.0027609629501285237,
            "z": 0.3488328495604457,
            "w": 0.9371631933871791
        }
    },
    {
        "name": "load_station",
        "x": 13.546630082609385,
        "y": 5.534828038492775,
        "orientation": {
            "x": -0.0026212716881792017,
            "y": -0.0022029291640848665,
            "z": -0.48825732572030434,
            "w": 0.872692992935254
        }
    },
    {
        "name": "pickup_station",
        "x": 11.883054016071227,
        "y": -2.281561224720647,
        "orientation": {
            "x": -0.0042111173584740876,
            "y": -0.0038075688791627004,
            "z": 0.22090023789022906,
            "w": 0.9752798848586306
        }
    },
]


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        robot_namespace = os.getenv("ROBOT", "default")
        self.action_name = f"/{robot_namespace}/navigate_to_pose"
        self._action_client = ActionClient(self, NavigateToPose, self.action_name)

    def send_waypoints(self):
        self._action_client.wait_for_server()

        while rclpy.ok():
            for waypoint in WAYPOINTS:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = PoseStamped()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = waypoint['x']
                goal_msg.pose.pose.position.y = waypoint['y']
                goal_msg.pose.pose.orientation.x = waypoint['orientation']['x']
                goal_msg.pose.pose.orientation.y = waypoint['orientation']['y']
                goal_msg.pose.pose.orientation.z = waypoint['orientation']['z']
                goal_msg.pose.pose.orientation.w = waypoint['orientation']['w']

                self.get_logger().info(f"Navigating to {waypoint['name']}...")
                future = self._action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, future)
                goal_handle = future.result()

                if not goal_handle.accepted:
                    self.get_logger().error(f"Goal {waypoint['name']} rejected")
                    continue

                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result().result

                self.get_logger().info(f"Arrived at {waypoint['name']} with result: {result}")
                
                self.get_logger().info("Waiting for 2 minutes before next waypoint...")
                time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    navigator.send_waypoints()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
