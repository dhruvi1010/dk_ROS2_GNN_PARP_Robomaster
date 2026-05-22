#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import sys, os
from tf2_ros import TransformBroadcaster, TransformStamped
import sensor_msgs.msg as sensor_msgs
from math import sin,cos,pi
from sensor_msgs.msg import JointState


class joint_tester(Node):
    
    def __init__(self):
        rclpy.init()
        super().__init__('robomaster_state_publisher')

        qos_profile = QoSProfile(depth=10)
        
        # Broadcaster
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        #self.get_logger().info("{0} started".format(self.nodeName))
        self.get_logger().info("Starting Robomaster Controller Version 2")

        # Rospyrate
        self.rate = self.create_rate(50)

        ## Joint State Publisher 
        # self.joint_pub = self.create_publisher(JointState, '/robomaster/wheel_joint_states', qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'wheel_joint_states', qos_profile)

        self.talker()
  
    ## Update all  joints input is in degree 
    def update_joint(self,angle_degree):
        now = self.get_clock().now().to_msg()
        # self.get_logger().info("Update Joints")
        msg = JointState()
        msg.header.frame_id="robomaster_wheels"
        msg.velocity = []
        msg.effort = []

        msg.header.stamp = now
        for x in [['wheel_fr_joint',0],['wheel_fl_joint',0],['wheel_rr_joint',0],['wheel_rl_joint',0]]:
            msg.name = [x[0]]
            angle=angle_degree*(pi/180)
            msg.position = [angle] 
            # self.get_logger().info("{} set angle to {}".format(x, str(angle)))
            self.joint_pub.publish(msg)  
        

    ## This will Run with the defined ROS Rate ans send Updates to the Motors.
    def talker(self):
        angle_degree=0       
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_joint(angle_degree)
            angle_degree=angle_degree+7.2
            self.rate.sleep()

def main():
    try:
        node = joint_tester()
    except Exception as e:
        print(e)



if __name__ == '__main__':
    main()    
