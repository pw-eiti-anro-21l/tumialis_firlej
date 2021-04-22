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
from PyKDL import *


class KdlDkin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('kdl_dkin')
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
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_kdl', qos_profile)
        self.joint_state = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # rate
        self.rate = self.create_rate(10)

    def listener_callback(self, msg):
        # get joint state
        [self.poz1, self.poz2, self.poz3] = msg.position

        # publish pose of chwytak
        now = self.get_clock().now()
        self.calculate_pose_stamp()
        self.pose_stamped.header.stamp = now.to_msg()
        self.pose_stamped.header.frame_id = 'baza'
        self.pose_pub.publish(self.pose_stamped)
        self.rate.sleep()

    def calculate_pose_stamp(self):

        chain = Chain()
        # baza_ramie1 = Joint(Joint.RotZ) 
        # frame1 = Frame(Rotation.RPY(0,0,0), Vector(0,0,self.d1)) 
        # segment1 = Segment(baza_ramie1,frame1)
        # chain.addSegment(segment1) 


        # ramie1_ramie2 = Joint(Joint.RotZ) 
        # frame2 = Frame(Rotation.RPY(0,0,0), Vector(self.a2,0,0)) 
        # segment2=Segment(ramie1_ramie2,frame2)
        # chain.addSegment(segment2)

        baza_ramie1 = Joint(Joint.RotZ) 
        frame1 = Frame(Rotation.RPY(0,0,0), Vector(self.a2,0,self.d1))
        segment1 = Segment(baza_ramie1,frame1)
        chain.addSegment(segment1) 


        ramie1_ramie2 = Joint(Joint.RotZ) 
        frame2 = Frame(Rotation.RPY(0,0,0), Vector(0,0,0))
        segment2=Segment(ramie1_ramie2,frame2)
        chain.addSegment(segment2)


        ramie2_ramie3 = Joint(Joint.TransZ)
        frame3 = Frame(Rotation.RPY(self.alfa3,0,0), Vector(self.a3,(self.d3*sin(self.alfa3)*(-1)),(self.d3*cos(self.alfa3))))
        segment3=Segment(ramie2_ramie3,frame3)
        chain.addSegment(segment3)


        jointPositions=JntArray(3)
        jointPositions[0]= self.poz1
        jointPositions[1]= self.poz2
        jointPositions[2]= -self.poz3

        solver=ChainFkSolverPos_recursive(chain)
        finalFrame=Frame()
        solver.JntToCart(jointPositions,finalFrame)

        quaternion = finalFrame.M.GetQuaternion()
        xyz = finalFrame.p
        
        self.pose_stamped.pose.position.x = xyz[0]
        self.pose_stamped.pose.position.y = xyz[1]
        self.pose_stamped.pose.position.z = xyz[2]
        self.pose_stamped.pose.orientation = Quaternion(x=float(quaternion[0]), y=float(quaternion[1]), z=float(quaternion[2]), w=float(quaternion[3]))


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    node = KdlDkin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
