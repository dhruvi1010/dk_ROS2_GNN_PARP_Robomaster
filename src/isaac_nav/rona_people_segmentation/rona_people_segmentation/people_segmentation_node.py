#!/usr/bin/env python3

from rona_people_segmentation.people_segmentation import PeopleSegmentation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError

class PeopleSegmentationNode(Node, PeopleSegmentation):
    def __init__(self):
        Node.__init__(self, "people_segmentation_node")
        PeopleSegmentation.__init__(self, "/workspaces/isaac_ros-dev/isaac_ros_assets/models/peoplesemsegformer/1/model.onnx")
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(Image, "image_raw", self.people_segmentation_cb, qos_profile_sensor_data)
        self.seg_pub = self.create_publisher(Image, "segment_result", qos_profile_sensor_data)
        
    def people_segmentation_cb(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV format (BGR by default)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
        
        result_cv_img = self.predict_segmentation2(cv_image)
        ros_msg = self.bridge.cv2_to_imgmsg(result_cv_img, encoding='bgr8')
        self.seg_pub.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PeopleSegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
