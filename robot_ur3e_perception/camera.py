#!/usr/bin/env python3

'''
plan A: using hough circle to detect the circle in the image
https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
plan B: using canny and then contour to detect the circle in the image
https://www.youtube.com/watch?v=YyRIvbLC99U&list=PLBg7GSvtrU2MOLWM0bGU1_FT3LsJPyY-5&index=14
https://www.youtube.com/watch?v=R82EcsCgnfg&list=PLBg7GSvtrU2MOLWM0bGU1_FT3LsJPyY-5&index=23
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# version 4.5.4
import cv2
import numpy as np
# import my libraries

from robot_ur3e_perception.marker import MarkerPublisher
class ShowingImage(Node):

    def __init__(self):
        self.image_pub = self.create_publisher(Image, "/my_image_output", 10)
        self.bridge_object = CvBridge()
        # the image shape is 480x640x3
        self.image_sub = self.create_subscription(
            Image, "/wrist_rgbd_depth_sensor/image_raw", self.camera_callback, 10)
        # declaring my parameters
        self.minDist = self.declare_parameter("min_distance_centers", 20)
        self.param1 = self.declare_parameter("high_canny_threshold", 255) # greater is better
        self.param2 = self.declare_parameter("accumulator_threshold", 16)
        self.minRadius = self.declare_parameter("min_radius", 15)
        self.maxRadius = self.declare_parameter("max_radius", 30)

    def camera_callback(self, msg):
        # Get the latest values of the parameters
        self.minDist = self.get_parameter("min_distance_centers").value
        self.param1 = self.get_parameter("high_canny_threshold").value
        self.param2 = self.get_parameter("accumulator_threshold").value
        self.minRadius = self.get_parameter("min_radius").value
        self.maxRadius = self.get_parameter("max_radius").value

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(
                msg, desired_encoding="bgr8")
            # threshold the white color
            lower_white = np.array([200, 200, 200], dtype=np.uint8)
            upper_white = np.array([255, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(cv_image, lower_white, upper_white)
            thresholded = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            
            # modifying the image for circle detection
            gray_image = cv2.cvtColor(thresholded, cv2.COLOR_BGR2GRAY)
            # using median blur
            gray = cv2.medianBlur(gray_image, 5)
            gray = cv2.dilate(gray, (5, 5), iterations=6)
            gray = cv2.erode(gray, (5, 5), iterations=5)
            # image segmentation by mask
            segmentated = cv2.bitwise_and(cv_image, cv_image, mask=gray)
            
            # canny edge detection
            canny = cv2.Canny(gray, self.param1/2, self.param1)
            cv2.imshow('canny', canny)
            
            '''
            parameters for HoughCircles:
            image: 8-bit, single-channel, grayscale input image.
            method: Detection method to use. Currently, the only implemented 
                    method is HOUGH_GRADIENT
            dp: Inverse ratio of the accumulator resolution to the image 
                    resolution. For example, if dp=1 , the accumulator has the 
                    same resolution as the input image. If dp=2 , the accumulator 
                    has half as big width and height.
            minDist: Minimum distance between the centers of the detected 
                    circles. If the parameter is too small, multiple neighbor 
                    circles may be falsely detected in addition to a true one. 
                    If it is too large, some circles may be missed.
            param1: This is the higher threshold for the Canny edge detector 
                    (the lower one is twice smaller). The Canny edge detector 
                    is used internally in the Hough Gradient Method to detect 
                    edges in the image.
            param2: This is the accumulator threshold for the circle centers at 
                    the detection stage. The smaller it is, the more false circles 
                    may be detected. Circles, corresponding to the larger 
                    accumulator values, will be returned first.
            minRadius: Minimum circle radius.
            maxRadius: Maximum circle radius.
            '''
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1,
                                       minDist=self.minDist,
                                       param1=self.param1, param2=self.param2,
                                       minRadius=self.minRadius, maxRadius=self.maxRadius)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = (i[0], i[1])
                    # circle center
                    cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
                    # circle outline
                    radius = i[2]
                    cv2.circle(cv_image, center, radius, (255, 0, 255), 3)

            # publishing a modify image
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(cv_image, encoding="bgr8"))
            #self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(segmentated, encoding="mono8"))
            cv2.imshow('circles', cv_image)
            cv2.imshow('segmentation', segmentated)
        except CvBridgeError as e:
            self.get_logger().info('{}'.format(e))

        cv2.waitKey(1)

class MyNode(MarkerPublisher, ShowingImage):
    def __init__(self):
        Node.__init__(self, 'my_node_special')
        MarkerPublisher.__init__(self)
        ShowingImage.__init__(self)

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
