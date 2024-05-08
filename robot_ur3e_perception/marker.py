import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MarkerPublisher(Node):
    def __init__(self):
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
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
        self.marker.scale.x = 0.10
        self.marker.scale.y = 0.10
        self.marker.scale.z = 0.10
                
    def draw_marker(self, x, y, z):
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.publisher_.publish(self.marker)
    
    def erase_marker(self):
        self.marker.color.a = 0.0
        self.publisher_.publish(self.marker)
        
