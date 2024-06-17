import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Trigger

class SavePhotoNode(Node):
    def __init__(self):
        super().__init__('save_photo_node')
        self.subscription = self.create_subscription(
            Image,
            '/wrist_rgbd_depth_sensor/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.srv = self.create_service(Trigger, 'save_photo', self.save_photo_service_callback)
        self.get_logger().info("SavePhotoNode started, use the '/save_photo' service to save a photo.")

    def save_photo_service_callback(self, request, response):
        self.save_photo()
        response.success = True
        response.message = "Photo saved successfully."
        return response

    def listener_callback(self, msg):
        self.latest_msg = msg

    def save_photo(self):
        if hasattr(self, 'latest_msg') and self.latest_msg is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_msg, "bgr8")
                path = '/home/user/ros2_ws/src/robot_ur3e_perception/snapshot'
                photo_name = f"photo_{self.get_clock().now().to_msg().sec}.jpg"
                photo_path = path + '/' + photo_name
                cv2.imwrite(photo_path, cv_image)
                self.get_logger().info(f"Saved photo: {photo_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save photo: {e}")
        else:
            self.get_logger().warn("No image data available to save.")

def main(args=None):
    rclpy.init(args=args)
    save_photo_node = SavePhotoNode()
    rclpy.spin(save_photo_node)
    save_photo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()