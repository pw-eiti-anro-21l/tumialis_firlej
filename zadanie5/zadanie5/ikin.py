#! /usr/bin/env python
from math import sin, cos, atan2, sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import yaml
from tf2_ros import TransformBroadcaster, TransformStamped


class Ikin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('ikin_node')
        self.declare_parameter('yaml_file')
        yaml_file = self.get_parameter('yaml_file').get_parameter_value().string_value

        # wczytanie parametrow z macierzy DH
        with open(yaml_file, 'r') as stream:
            try:
                macierzDH = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # zapisanie parametrow manipulatora
        self.d1 = dict(macierzDH.get('ramie1')).get('d')
        self.a2 = dict(macierzDH.get('ramie2')).get('a')
        self.a3 = dict(macierzDH.get('ramie3')).get('a')
        self.d3 = dict(macierzDH.get('ramie3')).get('d')
        self.alfa3 = dict(macierzDH.get('ramie3')).get('alfa')  # niepotrzebne

        # deklaracja polozenia jointow
        self.poz1 = 0.0
        self.poz2 = 0.0
        self.poz3 = 0.0

        # deklaracja zmiennych przechowujacych nadchodzace dane polozenia
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # deklaracja publisherow i subscriberow
        qos_profile = QoSProfile(depth=10)
        self.pose_sub = self.create_subscription(PoseStamped, 'pose_stamped_oint', self.listener_callback, qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        # setup broadcastera danych tf
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'baza'
        self.joint_state = JointState()

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

    def listener_callback(self, msg):
        # pobranie zadanej pozycji
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        # obliczenie odwrotnej kinematyki
        self.calculate_joint_state()
        self.publish_joint()

    def publish_joint(self):
        # publikowanie pozycji jointow uzyskanych z kinematyki odwrotnej
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ["baza_do_ramie1", "ramie1_do_ramie2", "ramie2_do_ramie3"]
        self.joint_state.position = [self.poz1, self.poz2, self.poz3]

        self.odom_trans.header.stamp = now.to_msg()

        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)

    def calculate_joint_state(self):
        # funkcja liczaca kinematyke odwrotna dla aktualnych danych x,y,z
        # liczenie poz2
        cosVal2 = (self.x ** 2 + self.y ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3)
        if cosVal2 > 1 or cosVal2 < -1:
            self.get_logger().error("IMPOSSIBLE POSITION")  # jesli ta wartosc jest poza wartosciami to niemozliwe jest ustalenie pozycji
        else:
            sinVal2 = sqrt(1 - cosVal2 ** 2)  # "+/-   ->   lokiec u gory/dolu"
            self.poz2 = atan2(sinVal2, cosVal2)

            # liczenie poz1
            k1 = self.a2 + self.a3 * cosVal2
            k2 = self.a3 * sinVal2
            r = sqrt(k1 ** 2 + k2 ** 2)
            phi = atan2(k2, k1)
            k1 = r * cos(phi)
            k2 = r * sin(phi)
            self.poz1 = atan2(self.y, self.x) - atan2(k2, k1)

            # liczenie poz3
            self.poz3 = self.d1 - self.d3 - self.z



def main():
    node = Ikin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
