#!/home/user/ros2_ws/src/robot_ur3e_perception/venv/bin/python
'''
this program basically do the following:
GET
->ros2 topic echo /joint_states
values of interest are the ones which these names
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
->ros2 run tf2_ros tf2_echo base_link tool0
- Translation: [X, Y, Z]
SAVE
data.csv
- [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, X, Y, Z]
when service is called
'''

import rclpy
from rclpy.node import Node
# TF listener
from tf2_ros import TransformListener, Buffer
# service
from std_srvs.srv import Trigger
# joint states
from sensor_msgs.msg import JointState
# manage data
import csv


class Panda_listener(Node):
    def __init__(self):
        super().__init__('Panda_listener')
        # TF listener for getting the position of the end effector
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        # service
        self.srv = self.create_service(
            Trigger, 'save_data', self.save_data_service_callback)
        # joint states
        self.joint_positions = {}
        self.joint_names_of_interest = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names_of_interest:
                self.joint_positions[name] = position
        self.get_logger().info('Joint positions: %s' % self.joint_positions)

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % e)
            return None

    def save_data_service_callback(self, request, response):
        with open('data.csv', 'a', newline='') as csvfile:
            fieldnames = self.joint_names_of_interest + ['X', 'Y', 'Z']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            data = {name: self.joint_positions.get(
                name, 0) for name in self.joint_names_of_interest}
            data.update({'X': self.pos_x, 'Y': self.pos_y, 'Z': self.pos_z})
            writer.writerow(data)
        response.success = True
        response.message = "Data saved successfully"
        return response

    def log_transform(self, transform):
        self.get_logger().info('Transform: %s -> %s' %
                               (transform.header.frame_id, transform.child_frame_id))
        self.get_logger().info('Translation: (%.3f, %.3f, %.3f)' % (transform.transform.translation.x,
                                                                    transform.transform.translation.y, transform.transform.translation.z))
        self.pos_x = transform.transform.translation.x
        self.pos_y = transform.transform.translation.y
        self.pos_z = transform.transform.translation.z
        self.get_logger().info('Rotation: %.3f, %.3f, %.3f, %.3f' % (transform.transform.rotation.x,
                                                                     transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w))


def main(args=None):
    rclpy.init(args=args)
    tf_listener = Panda_listener()
    rclpy.spin(tf_listener)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
