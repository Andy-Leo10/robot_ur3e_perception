#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# marker data
from visualization_msgs.msg import Marker
# point position
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('MARKER_BOSS')
        # subscriber
        self.subscription = self.create_subscription(
            Point, 'cup_abs_position', self.point_callback, 10)
        # publisher
        self.publisher_ = self.create_publisher(Marker, 'free_space_marker', 10)
        # timer for managing the marker
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # declaring my parameters
        self.cylinder_x = self.declare_parameter("cylinder_x", 0.10)
        self.cylinder_y = self.declare_parameter("cylinder_y", 0.10)
        self.cylinder_z = self.declare_parameter("cylinder_z", 0.10)
        # creating the marker
        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "my_namespace"
        self.marker.id = 0
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.06
        # variable to keep track of whether a new message has been received
        self.new_message_received = False

    def point_callback(self, msg):
        # draw the marker at the received point
        #self.draw_marker(msg.z, -msg.x, -msg.y)
        self.draw_marker(msg.x, msg.y, msg.z)
        self.new_message_received = True

    def timer_callback(self):
        # Get the latest values of the parameters
        self.cylinder_x = self.get_parameter("cylinder_x").value
        self.cylinder_y = self.get_parameter("cylinder_y").value
        self.cylinder_z = self.get_parameter("cylinder_z").value
        
        if self.new_message_received:
            # if a new message has been received, draw the marker
            self.publisher_.publish(self.marker)
        else:
            # if no new message has been received, hide the marker
            self.hide_marker()
        # set new_message_received to False
        self.new_message_received = False
                
    def draw_marker(self, x, y, z):
        self.marker.scale.x = self.cylinder_x
        self.marker.scale.y = self.cylinder_y
        self.marker.scale.z = self.cylinder_z
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.a = 0.5 # transparency
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.publisher_.publish(self.marker)
    
    def hide_marker(self):
        self.marker.color.a = 0.0
        self.publisher_.publish(self.marker)
        
def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()