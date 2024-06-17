#!/home/user/ros2_ws/src/robot_ur3e_perception/venv/bin/python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from pathlib import Path

class ShowingImage(Node):

    def __init__(self):
        super().__init__('my_camera_node')
        self.image_pub = self.create_publisher(Image, "/my_image_output", 10)
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image, "/wrist_rgbd_depth_sensor/image_raw", self.camera_callback, 10)

        # Check if CUDA (GPU support) is available
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f'Using device: {self.device}')

        # Model
        self.model = torch.hub.load(str(Path("/home/user/yolov5")), 
                           "custom", 
                           path=Path("/home/user/linux.pt"), 
                           source="local").to(self.device)

    def camera_callback(self, msg):
        try:
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
            

            # Publish the yolo v5 image detection
            confidence_threshold = 0.75  # Set your desired confidence threshold here
            # Filter detections based on the confidence threshold
            filtered_detections = [detection for detection in detections if detection[4] > confidence_threshold]
            # Correctly unpack the detection tuple
            for detection in filtered_detections:
                x1, y1, x2, y2, confidence, cls_id = detection[:6].cpu().numpy()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                confidence = float(confidence)
                cls = int(cls_id)  # Convert class id to int if it's not already
                cv2.rectangle(original_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(original_img, f'Class: {cls}, Conf: {confidence:.2f}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 0, 255), 2)
            cv2.putText(original_img, f'Image Size: {original_img.shape[1]}x{original_img.shape[0]}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Display the image with bounding boxes
            cv2.imshow('Detected Objects', original_img)
            #publishing a modify image
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(original_img, encoding="bgr8"))
            
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