#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# TF listener
from tf2_ros import TransformListener, Buffer
# TF publisher
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
# point position
from geometry_msgs.msg import Point

class TF_Manager(Node):
    def __init__(self):
        super().__init__('tf_manager')
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # TF publisher
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        # timer for loging transform
        self.create_timer(1, self.timer_callback)
        # point subscriber
        self.subscription = self.create_subscription(
            Point, 'cup_position', self.point_callback, 10)
    
    def point_callback(self, msg):
        # set the transform
        self.set_transform('wrist_rgbd_camera_link', 'CUP', msg.z, -msg.x, -msg.y)
                
    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % e)
            return None

    def timer_callback(self):
        transform = self.get_transform('world', 'CUP')
        if transform:
            self.log_transform(transform)

    def log_transform(self, transform):
        self.get_logger().info('Transform: %s -> %s' % (transform.header.frame_id, transform.child_frame_id))
        self.get_logger().info('Translation: %s' % transform.transform.translation)
        self.get_logger().info('Rotation: %s' % transform.transform.rotation)

    def set_transform(self, target_frame, source_frame, x, y, z):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = target_frame
        transform.child_frame_id = source_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)
        
    def set_transform_static(self, target_frame, source_frame):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = target_frame
        transform.child_frame_id = source_frame
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 1.0
        transform.transform.translation.z = 1.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    tf_listener = TF_Manager()
    rclpy.spin(tf_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()