import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
import math
import socket
from argparse import ArgumentParser

class SingleCircularTrajectoryPublisher(Node):
    def __init__(self, linear_velocity):
        super().__init__('single_circular_trajectory_publisher')
        self.linear_velocity = linear_velocity
        self.namespace = socket.gethostname()

        # Define radii and initial positions for circular trajectory
        self.radii = [2.0, 2.5, 3.1]
        self.start_positions = [(-3.75, -2), (-3.75, -2.5), (-3.75, -3.1)]

        # Action client for NavigateToPose
        self.action_client = ActionClient(self, NavigateToPose, f'{self.namespace}/navigate_to_pose')

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)

        # State management
        self.initial_goal_sent = False
        self.start_circular_motion = False
        self.current_radius_index = 0  # Start with the first radius
        self.num_radii = len(self.radii)
        self.start_time = None

        # Timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def send_initial_goal(self, position):
        """Send the initial goal to the NavigateToPose action server."""
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(position[0])
        goal_msg.pose.pose.position.y = float(position[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation

        self.get_logger().info(f'Sending goal to position {position}...')
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for when the NavigateToPose action server responds to the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected!')
            return

        # self.get_logger().info('Goal accepted!')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback for when the NavigateToPose action is complete."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            # self.get_logger().info('Reached goal position!')
            self.initial_goal_sent = True
        else:
            self.get_logger().info(f'Goal failed with status: {result.status}')

    def generate_circular_motion(self, radius, linear_velocity):
        """Calculate angular velocity and return a Twist message."""
        angular_velocity = linear_velocity / radius
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity  # Counterclockwise motion
        return twist_msg


    def control_loop(self):
        """Control loop for managing the robot."""
        if self.current_radius_index < self.num_radii:
            radius = self.radii[self.current_radius_index]
            position = self.start_positions[self.current_radius_index]

            # Initialize elapsed_time
            elapsed_time = 0.0

            # Initial goal and circular motion
            if not self.initial_goal_sent:
                self.send_initial_goal(position)
            elif not self.start_circular_motion:
                self.start_circular_motion = True
                self.start_time = self.get_clock().now()
                self.get_logger().info(f'Starting circular motion at radius {radius}')
            elif self.start_circular_motion:
                elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
                if elapsed_time < 60.0:  # Run each radius for 20 seconds
                    twist_msg = self.generate_circular_motion(radius, self.linear_velocity)
                    self.cmd_vel_publisher.publish(twist_msg)
                else:
                    self.cmd_vel_publisher.publish(Twist())  # Stop the robot

            # Advance to the next radius
            if self.start_circular_motion and elapsed_time >= 60.0:
                # self.get_logger().info(f'Completed radius {radius}. Advancing to the next.')
                self.current_radius_index += 1
                self.initial_goal_sent = False
                self.start_circular_motion = False

        else:
            self.get_logger().info('Completed all circles Shutting down.')
            self.send_initial_goal((-1,2.5))
            self.timer.cancel()


def main():
    parser = ArgumentParser(description='Single circular trajectory controller.')
    parser.add_argument('vel', type=float, help='Set the constant linear velocity (e.g., 0.8, 1.0, 1.5 m/s)')
    args = parser.parse_args()

    rclpy.init()  # Initialize without passing argparse arguments

    node = SingleCircularTrajectoryPublisher(linear_velocity=args.vel)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, Twist
# import math
# import socket
# from time import sleep

# class CircularTrajectoryPublisher(Node):
#     def __init__(self):
#         super().__init__('circular_trajectory_with_nav_goal')
#         self.namespace = socket.gethostname()

#         # Declare and get parameters
#         self.declare_parameter('radius', 2.0)  
#         self.declare_parameter('angular_velocity', 0.5)  
#         self.declare_parameter('initial_x', 0.0)  
#         self.declare_parameter('initial_y', -2.0)  
#         self.declare_parameter('duration', 60.0)  
        
#         self.radius = self.get_parameter('radius').value
#         self.angular_velocity = self.get_parameter('angular_velocity').value
#         self.initial_x = self.get_parameter('initial_x').value
#         self.initial_y = self.get_parameter('initial_y').value
#         self.duration = self.get_parameter('duration').value

#         ## params for medium size####
#         self.mradius = 3.5
#         self.mangular_velocity = 0.5
#         self.minitial_x = 0.0
#         self.minitial_y = -2.0
#         self.mduration = 60

#         ## params for big size ####
#         self.bradius = 4.0
#         self.bangular_velocity = 0.5
#         self.binitial_x = 0.0
#         self.binitial_y = -2.0
#         self.bduration = 60
        
#         # Calculate linear velocity based on radius and angular velocity
#         self.linear_velocity = self.angular_velocity * self.radius

#         self.get_logger().info(f'Radius: {self.radius}, Angular Velocity: {self.angular_velocity}')
#         self.get_logger().info(f'Calculated Linear Velocity: {self.linear_velocity}')

#         # Publishers
#         self.goal_publisher = self.create_publisher(PoseStamped, self.namespace + '/goal_pose', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

#         # State management
#         self.initial_goal_sent = False
#         self.start_circular_motion = False
#         self.start_time = None

#         # Timer
#         self.timer_period = 0.1  # 10 Hz
#         self.timer = self.create_timer(self.timer_period, self.control_loop)

#     def send_initial_goal(self):
#         """Send the initial navigation goal."""
#         goal_msg = PoseStamped()
#         goal_msg.header.frame_id = 'map'
#         goal_msg.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.position.x = self.initial_x
#         goal_msg.pose.position.y = self.initial_y
#         goal_msg.pose.position.z = 0.0
#         goal_msg.pose.orientation.w = 1.0  # No rotation
        
#         self.get_logger().info(f'Sending initial goal to ({self.initial_x}, {self.initial_y})...')
#         self.goal_publisher.publish(goal_msg)
#         self.initial_goal_sent = True

#     def start_circular_trajectory(self):
#         """Start the circular trajectory motion."""
#         self.get_logger().info('Starting circular trajectory...')
#         self.start_circular_motion = True
#         self.start_time = self.get_clock().now()

#     def control_loop(self):
#         """Control loop to manage initial positioning and circular motion."""
#         if not self.initial_goal_sent:
#             # Send the initial navigation goal
#             self.send_initial_goal()
#             sleep(5)  # Allow some time for the navigation stack to process

#         elif self.initial_goal_sent and not self.start_circular_motion:
#             # Assume the robot reached the goal (or implement proper feedback logic)
#             self.start_circular_trajectory()

#         elif self.start_circular_motion:
#             # Publish cmd_vel messages for circular trajectory
#             elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
#             if elapsed_time < self.duration:
#                 twist_msg = Twist()
#                 twist_msg.linear.x = self.linear_velocity
#                 twist_msg.angular.z = self.angular_velocity
#                 self.cmd_vel_publisher.publish(twist_msg)
#             else:
#                 # Stop the robot after completing the circular motion
#                 twist_msg = Twist()
#                 self.cmd_vel_publisher.publish(twist_msg)  # Stop robot
#                 self.get_logger().info('Completed circular trajectory. Shutting down.')
#                 self.timer.cancel()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CircularTrajectoryPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
