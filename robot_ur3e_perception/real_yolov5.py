#!/home/user/ros2_ws/src/robot_ur3e_perception/venv/bin/python

import rclpy
from rclpy.node import Node
# image data and processing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# vision processing
import cv2
import torch
from pathlib import Path
# point position
from geometry_msgs.msg import Point
import numpy as np

class ShowingImage(Node):

    def __init__(self):
        super().__init__('my_camera_node')
        # publishers
        self.image_pub = self.create_publisher(Image, "/my_image_output", 10)
        self.point_pub = self.create_publisher(Point, "/cup_rel_position", 10)
        # bridge for image processing
        self.bridge_object = CvBridge()
        # subscribers
        self.image_sub = self.create_subscription(
            Image, "/D415/color/image_raw", self.camera_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, "/D415/aligned_depth_to_color/image_raw", self.depth_callback, 10)
        # /D415/depth/image_rect_raw
        # Check if CUDA (GPU support) is available
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f'Using device: {self.device}')
        # Model
        self.model = torch.hub.load(str(Path("/home/user/yolov5")), 
                           "custom", 
                           path=Path("/home/user/ros2_ws/src/robot_ur3e_perception/linux.pt"), 
                           source="local").to(self.device)
        
        # atributtes for the cup detection
        self.cup_space_found = False
        self.cup_spaces = []
        self.hole_depth = 0.040  # meters
        
    def camera_callback(self, msg):
        try:
            self.cup_space_found = False # clear the flag
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            original_img = cv_image.copy()
            
            # Inference
            results = self.model(cv_image)

            # Results
            print('-------------------------------------------------------Results')
            results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
            # Display the image with detections
            results.show()
            print('-------------------------------------------------------coordinates')
            # Access the bounding box coordinates and other information
            detections = results.xyxy[0]  # Detections for the first image
            print("Detections (x1, y1, x2, y2, confidence, class):")
            print(detections)
            print('-------------------------------------------------------separating')
            # Optionally, you can iterate over the detections and print or use them as needed
            for detection in detections:
                x1, y1, x2, y2, confidence, cls = detection
                print(f"Box coordinates: {x1}, {y1}, {x2}, {y2}. Confidence: {confidence}. Class: {cls}")
                self.get_logger().info(f"Box coordinates: {x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f}. Confidence: {confidence:.2f}. Class: {cls}")
            self.get_logger().info(f"------------------------------------------------------")
            
            self.cup_spaces = [] # clear the list
            # Publish the yolo v5 image detection
            confidence_threshold = 0.75  # Set your desired confidence threshold here
            # Filter detections based on the confidence threshold
            filtered_detections = [detection for detection in detections if detection[4] > confidence_threshold]
            if len(filtered_detections) > 0:
                self.cup_space_found = True
            # Correctly unpack the detection tuple
            for detection in filtered_detections:
                x1, y1, x2, y2, confidence, cls_id = detection[:6].cpu().numpy()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                confidence = float(confidence)
                cls = int(cls_id)  # Convert class id to int if it's not already
                cv2.rectangle(original_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(original_img, f'Class: {cls}, Conf: {confidence:.2f}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 0, 255), 2)
                center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                cv2.circle(original_img, center, 2, (0, 255, 0), 2)
                self.cup_spaces.append(center)
            cv2.putText(original_img, f'Image Size: {original_img.shape[1]}x{original_img.shape[0]}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Display the image with bounding boxes
            cv2.imshow('Detected Objects', original_img)
            #publishing a modify image
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(original_img, encoding="bgr8"))
            
        except CvBridgeError as e:
            self.get_logger().info('{}'.format(e))

        cv2.waitKey(1)

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(
                msg, desired_encoding="32FC1")
            if(self.cup_space_found):
                # take the 1st cup
                center = self.cup_spaces[0]
                # get the depth value
                depth = cv_image[center[1], center[0]]
                
                # get the intrinsic parameters
                fx = 520.7813804684724  # focal length in x
                fy = 520.7813804684724  # focal length in y
                cx = 320.5  # optical center x
                cy = 240.5  # optical center y
                
                # calculate 3D position
                X = (center[0] - cx) * depth / fx
                Y = (center[1] - cy) * depth / fy
                Z = depth

                # modify the relative position because point of interest is on top
                # of the hole for the cup and not in the deepest part of the hole
                # for that reason: is necessary to modify the magnitude of this
                # vector with a FIX direction
                # Adjust Z value
                desired_Z = Z - self.hole_depth  # adjustment is the amount you want to subtract from Z
                # Calculate the current magnitude of the vector # 0.834
                current_magnitude = np.sqrt(X**2 + Y**2 + Z**2) 
                # self.get_logger().info('Current magnitude: {:.3f}'.format(current_magnitude))
                # Calculate the new magnitude based on the desired Z
                new_magnitude = current_magnitude * (desired_Z / Z)
                # Normalize the vector
                X_normalized = X / current_magnitude
                Y_normalized = Y / current_magnitude
                Z_normalized = Z / current_magnitude
                # self.get_logger().info('Normalized 3D position: ({:.3f}, {:.3f}, {:.3f})'.format(X_normalized, Y_normalized, Z_normalized))
                # Scale the normalized vector by the new magnitude
                X_new = X_normalized * new_magnitude
                Y_new = Y_normalized * new_magnitude
                Z_new = Z_normalized * new_magnitude

                self.get_logger().info('3D position: ({:.3f}, {:.3f}, {:.3f})'.format(X_new, Y_new, Z_new))
                self.point_pub.publish(Point(x=float(X_new), y=float(Y_new), z=float(Z_new)))
            else:
                self.get_logger().info('No cup detected')
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