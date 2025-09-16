import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import socket
from argparse import ArgumentParser

class DualCircularTrajectoryPublisher(Node):
    def __init__(self, linear_velocity):
        super().__init__('dual_circular_trajectory_publisher')

        # Set the linear velocity from user input
        self.linear_velocity = linear_velocity

        #with obstacle, default 1,2,3 radius
        # Declare parameters for the first robot (ep02)
        self.namespace = socket.gethostname()
        self.radii_1 = [2.0, 2.5, 3.0]
        self.start_positions_1 = [(0, -2), (0, -2.5), (0, -3.0)]

        # Declare parameters for the second robot (ep03)
        self.namespace2 = "rm04"
        self.radii_2 = [2.0, 2.5, 3.0]
        self.start_positions_2 = [(2, -2), (2, -2.5), (2, -3.0)]

        # Publishers for both robots
        self.goal_publisher_1 = self.create_publisher(PoseStamped, self.namespace + '/goal_pose', 10)
        self.cmd_vel_publisher_1 = self.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

        self.goal_publisher_2 = self.create_publisher(PoseStamped, self.namespace2 + '/goal_pose', 10)
        self.cmd_vel_publisher_2 = self.create_publisher(Twist, self.namespace2 + '/cmd_vel', 10)

        # State management
        self.initial_goal_sent_1 = False
        self.start_circular_motion_1 = False
        self.initial_goal_sent_2 = False
        self.start_circular_motion_2 = False

        self.start_time_1 = None
        self.start_time_2 = None

        # Timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Set robot states for different radii
        self.current_radius_index = 0  # Start with the first radius
        self.num_radii = len(self.radii_1)

    def send_initial_goal(self, namespace, position, goal_publisher):
        """Send an initial goal for a robot."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Ensure float type for position fields
        goal_msg.pose.position.x = float(position[0])
        goal_msg.pose.position.y = float(position[1])
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # No rotation
        
        self.get_logger().info(f'Sending initial goal for {namespace} to {position}...')
        goal_publisher.publish(goal_msg)


    def start_circular_motion(self, radius, linear_velocity, clockwise):
        """Calculate angular velocity and return a Twist message."""
        angular_velocity = linear_velocity / radius
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = -angular_velocity if clockwise else angular_velocity
        return twist_msg

    def control_loop(self):
        """Control loop for managing both robots."""
        if self.current_radius_index < self.num_radii:
            radius_1 = self.radii_1[self.current_radius_index]
            position_1 = self.start_positions_1[self.current_radius_index]
            radius_2 = self.radii_2[self.current_radius_index]
            position_2 = self.start_positions_2[self.current_radius_index]

            # Initialize elapsed times
            elapsed_time_1 = 0.0
            elapsed_time_2 = 0.0

            # Robot 1: Initial goal and circular motion
            if not self.initial_goal_sent_1:
                self.send_initial_goal(self.namespace, position_1, self.goal_publisher_1)
                self.initial_goal_sent_1 = True
            elif not self.start_circular_motion_1:
                self.start_circular_motion_1 = True
                self.start_time_1 = self.get_clock().now()
                self.get_logger().info(f'Starting circular motion for {self.namespace} at radius {radius_1}')
            elif self.start_circular_motion_1:
                elapsed_time_1 = (self.get_clock().now() - self.start_time_1).nanoseconds * 1e-9
                if elapsed_time_1 < 60.0:  # Run each radius for 20 seconds
                    twist_msg_1 = self.start_circular_motion(radius_1, self.linear_velocity, clockwise=True)
                    self.cmd_vel_publisher_1.publish(twist_msg_1)
                else:
                    self.cmd_vel_publisher_1.publish(Twist())  # Stop robot 1

            # Robot 2: Initial goal and circular motion
            if not self.initial_goal_sent_2:
                self.send_initial_goal(self.namespace2, position_2, self.goal_publisher_2)
                self.initial_goal_sent_2 = True
            elif not self.start_circular_motion_2:
                self.start_circular_motion_2 = True
                self.start_time_2 = self.get_clock().now()
                self.get_logger().info(f'Starting circular motion for {self.namespace2} at radius {radius_2}')
            elif self.start_circular_motion_2:
                elapsed_time_2 = (self.get_clock().now() - self.start_time_2).nanoseconds * 1e-9
                if elapsed_time_2 < 60.0:  # Run each radius for 20 seconds
                    twist_msg_2 = self.start_circular_motion(radius_2, self.linear_velocity, clockwise=False)
                    self.cmd_vel_publisher_2.publish(twist_msg_2)
                else:
                    self.cmd_vel_publisher_2.publish(Twist())  # Stop robot 2

            # Advance to the next radius
            if self.start_circular_motion_1 and self.start_circular_motion_2 and elapsed_time_1 >= 20.0 and elapsed_time_2 >= 20.0:
                self.get_logger().info(f'Completed radius {radius_1} for both robots.')
                self.current_radius_index += 1
                self.initial_goal_sent_1 = False
                self.start_circular_motion_1 = False
                self.initial_goal_sent_2 = False
                self.start_circular_motion_2 = False

        else:
            self.get_logger().info('Completed all radii for both robots. Shutting down.')
            self.timer.cancel()

def main():
    parser = ArgumentParser(description='Dual circular trajectory controller.')
    parser.add_argument('vel', type=float, help='Set the constant linear velocity (e.g., 0.8, 1.0, 1.5 m/s)')
    args = parser.parse_args()

    rclpy.init()  # Initialize without passing argparse arguments

    node = DualCircularTrajectoryPublisher(linear_velocity=args.vel)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
