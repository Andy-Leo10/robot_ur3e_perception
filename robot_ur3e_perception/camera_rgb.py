#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ShowingImage(Node):

    def __init__(self):
        super().__init__('my_camera_node')
        self.image_pub = self.create_publisher(Image, "/my_taken_photos", 10)
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image, "/wrist_rgbd_depth_sensor/image_raw", self.camera_callback, 10)

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow('image from camera',cv_image)
            #modifying the image
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            #publishing a modify image
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(gray_image, encoding="mono8"))
        except CvBridgeError as e:
            self.get_logger().info('{}'.format(e))

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    showing_image_object = ShowingImage()
    rclpy.spin(showing_image_object)
    showing_image_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()