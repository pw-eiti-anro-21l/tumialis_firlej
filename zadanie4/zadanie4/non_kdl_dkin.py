#! /usr/bin/env python
from math import sin, cos, atan2, sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy
import yaml


class NonKdlDkin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('non_kdl_dkin')
        self.declare_parameter('yaml_file')
        yaml_file = self.get_parameter('yaml_file').get_parameter_value().string_value

        # load DH parameters
        with open(yaml_file, 'r') as stream:
            try:
                macierzDH = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.pose_stamped = PoseStamped()
        self.poz1 = 0.0
        self.poz2 = 0.0
        self.poz3 = 0.0

        self.d1 = dict(macierzDH.get('ramie1')).get('d')
        self.a2 = dict(macierzDH.get('ramie2')).get('a')
        self.a3 = dict(macierzDH.get('ramie3')).get('a')
        self.d3 = dict(macierzDH.get('ramie3')).get('d')
        self.alfa3 = dict(macierzDH.get('ramie3')).get('alfa')

        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_nonkdl', qos_profile)
        self.joint_state = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # timer
        self.create_timer(0.1, self.publish_pose_stamp)

    def listener_callback(self, msg):
        # get joint state
        [self.poz1, self.poz2, self.poz3] = msg.position

    def publish_pose_stamp(self):
        # publish pose of chwytak
        now = self.get_clock().now()
        self.calculate_pose_stamp()
        self.pose_stamped.header.stamp = now.to_msg()
        self.pose_stamped.header.frame_id = 'odom'
        self.pose_pub.publish(self.pose_stamped)

    def calculate_pose_stamp(self):
        T01 = numpy.array([[cos(self.poz1), -sin(self.poz1), 0.0, 0.0],
                           [sin(self.poz1), cos(self.poz1), 0.0, 0.0],
                           [0.0, 0.0, 1.0, self.d1],
                           [0.0, 0.0, 0.0, 1.0]])
        T12 = numpy.array([[cos(self.poz2), -sin(self.poz2), 0.0, self.a2],
                           [sin(self.poz2), cos(self.poz2), 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]])

        T23 = numpy.array([[1.0, 0.0, 0.0, self.a3],
                           [0.0, cos(self.alfa3), -sin(self.alfa3), 0.0],
                           [0.0, sin(self.alfa3), cos(self.alfa3), cos(self.alfa3) * (self.d3 + self.poz3)],
                           [0.0, 0.0, 0.0, 1.0]])

        T03 = numpy.matmul(T01, numpy.matmul(T12, T23))
        P3 = numpy.array([[0.0], [0.0], [0.0], [1.0]])  # poczatek ukladu wspolrzednych
        P0 = numpy.matmul(T03, P3)

        self.pose_stamped.pose.position.x = float(P0[0])
        self.pose_stamped.pose.position.y = float(P0[1])
        self.pose_stamped.pose.position.z = float(P0[2])
        self.pose_stamped.pose.orientation = euler_to_quaternion(*rot_to_euler(T03))


def rot_to_euler(t):
    yaw = atan2(t[1][0], t[0][0])
    pitch = atan2(-1 * t[2][0], sqrt(t[2][1] ** 2 + t[2][2] ** 2))
    roll = atan2(t[2][1], t[2][2])
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))


def main():
    node = NonKdlDkin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
