import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import socket

class CircularTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circular_trajectory_publisher')
        self.namespace = socket.gethostname()
        # Declare the radius parameter
        self.declare_parameter('radius', 1.2)  # Default value of 1.2 meters
        
        # Get the radius value from parameter
        self.radius = self.get_parameter('radius').value
        
        # Create a publisher for the /goal topic (example)
        self.publisher_ = self.create_publisher(PoseStamped, self.namespace +'/goal_pose', 10)
        
        # Parameters for the circular trajectory
        self.num_points = 35  # Number of waypoints around the circle
        self.origin_x = 0.0
        self.origin_y = 0.0
        
        # Generate waypoints
        self.waypoints = self.generate_waypoints()
        self.current_waypoint_index = 0
        self.loop_count = 0
        self.max_loops = 4  # Number of loops around the circle
        
        # Timer to publish waypoints
        if self.radius == 1.0:
            self.timer_period = 0.2  # seconds
        elif self.radius == 2.0:
            self.timer_period = 0.4  # seconds
        elif self.radius == 3.0:
            self.timer_period = 0.5  
        else:
            self.timer_period = 0.3  
        
        self.get_logger().info(f"The timer for the radius {self.radius} is: {self.timer_period}")
        self.timer = self.create_timer(self.timer_period, self.publish_waypoint)

    def generate_waypoints(self):
        waypoints = []
        for i in range(self.num_points):
            angle = 2 * math.pi * i / self.num_points
            x = self.origin_x + self.radius * math.cos(angle)
            y = self.origin_y + self.radius * math.sin(angle)
            
            # Calculate orientation (yaw) tangent to the circle
            yaw = angle + math.pi / 2  # Add 90 degrees to face the direction of movement
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            waypoint.pose.orientation.x = 0.0
            waypoint.pose.orientation.y = 0.0
            waypoint.pose.orientation.z = math.sin(yaw / 2)
            waypoint.pose.orientation.w = math.cos(yaw / 2)
            
            waypoints.append(waypoint)
        return waypoints

    def publish_waypoint(self):
        if self.loop_count < self.max_loops:
            if self.current_waypoint_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_waypoint_index]
                waypoint.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(waypoint)
                self.current_waypoint_index += 1
            else:
                # Reset waypoint index to start the next loop
                self.current_waypoint_index = 0
                self.loop_count += 1
                self.get_logger().info(f'Completed loop {self.loop_count}/{self.max_loops}')
        else:
            self.get_logger().info('Completed all loops! Shutting down the publisher.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CircularTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
### for navigation_T_poses ###
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateThroughPoses
# from geometry_msgs.msg import PoseStamped
# import math

# class CircularTrajectoryPublisher(Node):
#     def __init__(self):
#         super().__init__('circular_trajectory_publisher')
        
#         # Declare the radius parameter
#         self.declare_parameter('radius', 1.2)  # Default value of 1.2 meters
        
#         # Get the radius value from parameter
#         self.radius = self.get_parameter('radius').value
        
#         # Create an action client instead of a publisher
#         self.action_client = ActionClient(self, NavigateThroughPoses, 'ep02/navigate_through_poses')
        
#         # Parameters for the circular trajectory
#         # self.radius = 1.2  # Remove this line since we now get it from parameter
#         self.num_points = 20  # Number of waypoints around the circle
#         self.origin_x = 0.0
#         self.origin_y = 0.0
        
#         # Generate waypoints
#         self.waypoints = self.generate_waypoints()
#         self.current_waypoint_index = 0
        
#         # Timer to publish waypoints
#         self.timer_period = 1.0  # seconds
#         self.timer = self.create_timer(self.timer_period, self.publish_waypoint)

#     def generate_waypoints(self):
#         waypoints = []
#         for i in range(self.num_points):
#             angle = 2 * math.pi * i / self.num_points
#             x = self.origin_x + self.radius * math.cos(angle)
#             y = self.origin_y + self.radius * math.sin(angle)
            
#             # Calculate orientation (yaw) tangent to the circle
#             yaw = angle + math.pi/2  # Add 90 degrees to face the direction of movement
            
#             waypoint = PoseStamped()
#             waypoint.header.frame_id = 'map'
#             waypoint.pose.position.x = x
#             waypoint.pose.position.y = y
#             waypoint.pose.position.z = 0.0
            
#             # Convert yaw to quaternion
#             waypoint.pose.orientation.x = 0.0
#             waypoint.pose.orientation.y = 0.0
#             waypoint.pose.orientation.z = math.sin(yaw/2)
#             waypoint.pose.orientation.w = math.cos(yaw/2)
            
#             waypoints.append(waypoint)
#         return waypoints

#     def publish_waypoint(self):
#         if self.current_waypoint_index < len(self.waypoints):
#             # Create the action goal
#             goal_msg = NavigateThroughPoses.Goal()
#             goal_msg.poses = self.waypoints[self.current_waypoint_index:]
            
#             # Wait for action server
#             self.action_client.wait_for_server()
            
#             # Send goal
#             self.get_logger().info('Sending goal to navigation action server')
#             self.action_client.send_goal_async(goal_msg)
            
#             # Cancel the timer as we're sending all remaining poses at once
#             self.timer.cancel()
#             self.get_logger().info('Navigation goal sent!')
#         else:
#             self.get_logger().info('All waypoints sent!')
#             self.timer.cancel()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CircularTrajectoryPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()