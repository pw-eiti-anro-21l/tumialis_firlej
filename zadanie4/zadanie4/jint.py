#! /usr/bin/env python
import time
from math import sin, cos, pi
import threading
import rclpy
from time import sleep
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from zadanie4_srv.srv import Interpolation


def interpolate_linear(x0, x1, t0, t1, timePassed):
    return x0 + ((x1 - x0) / (t1 - t0)) * (timePassed - t0)


class Jint(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('jint')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.jint_control_srv = self.create_service(Interpolation, "interpolation_params",
                                                    self.interpolation_params_callback)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # robot state parameters
        self.declare_parameter('poz1', 0.0)
        self.declare_parameter('poz2', 0.0)
        self.declare_parameter('poz3', 0.0)
        self.poz1 = self.get_parameter('poz1').get_parameter_value().double_value
        self.poz2 = self.get_parameter('poz2').get_parameter_value().double_value
        self.poz3 = self.get_parameter('poz3').get_parameter_value().double_value

        # interpolation parameters
        self.time = 0.0
        self.oldpoz1 = self.poz1
        self.oldpoz2 = self.poz2
        self.oldpoz3 = self.poz3
        self.newpoz1 = 0.0
        self.newpoz2 = 0.0
        self.newpoz3 = 0.0
        self.interpolationMethod = "linear"
        self.timePeriod = 0.1
        self.timePassed = 0.0
        self.rate = self.create_rate(5)

        # threading
        self.result = None

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'baza'
        self.joint_state = JointState()

        publsh_thread = threading.Thread(target=self.publish_state)
        publsh_thread.start()

    def interpolation_params_callback(self, request, response):

        self.newpoz1 = request.newpoz1
        self.newpoz2 = request.newpoz2
        self.newpoz3 = request.newpoz3

        if request.time <= 0.0:
            response.operation = "Time must be grater than zero"
        else:
            if request.interpolation == "linear":
                self.time = request.time
                self.oldpoz1 = self.poz1
                self.oldpoz2 = self.poz2
                self.oldpoz3 = self.poz3
                self.interpolationMethod = request.interpolation
                self.timePassed = 0.0

                thread = threading.Thread(target=self.update_state)  # enable loop on a different thread
                thread.start()
                thread.join()  # wait for the thread to stop
                self.get_logger().info(str(self.result))  # print if successful
                response.operation = "very very nice ;)"
            else:
                response.operation = "very not nice :("
        return response

    def publish_state(self):
        while True:
            try:
                # update joint_state
                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = ["baza_do_ramie1", "ramie1_do_ramie2", "ramie2_do_ramie3"]
                self.joint_state.position = [self.poz1, self.poz2, self.poz3]
                self.odom_trans.header.stamp = now.to_msg()

                # send the joint state and transform
                self.joint_pub.publish(self.joint_state)
                self.broadcaster.sendTransform(self.odom_trans)
                time.sleep(self.timePeriod)
            except KeyboardInterrupt:
                pass

    def update_state(self):

        while True:

            # change params
            if self.poz1 != self.newpoz1:
                self.poz1 = interpolate_linear(self.oldpoz1, self.newpoz1, 0, self.time, self.timePassed)

            if self.poz2 != self.newpoz2:
                self.poz2 = interpolate_linear(self.oldpoz2, self.newpoz2, 0, self.time, self.timePassed)

            if self.poz3 != self.oldpoz3:
                self.poz3 = interpolate_linear(self.oldpoz3, self.newpoz3, 0, self.time, self.timePassed)

            # limits - if reached, automatically sets parameters to limit values, displays error
            if self.poz1 > 3.14:
                self.poz1 = 3.14
                self.get_logger().error(
                    "Reached limit of joint position! Max parameter 'poz1' value: " + str(self.poz1))

            if self.poz1 < -3.14:
                self.poz1 = -3.14
                self.get_logger().error(
                    "Reached limit of joint position! Min parameter 'poz1' value: " + str(self.poz1))

            if self.poz2 > 2.6:
                self.poz2 = 2.6
                self.get_logger().error(
                    "Reached limit of joint position! Max parameter 'poz2' value: " + str(self.poz2))

            if self.poz2 < -2.6:
                self.poz2 = -2.6
                self.get_logger().error(
                    "Reached limit of joint position! Min parameter 'poz2' value: " + str(self.poz2))

            if self.poz3 > 0.1:
                self.poz3 = 0.1
                self.get_logger().error(
                    "Reached limit of joint position! Max parameter 'poz3' value: " + str(self.poz3))

            if self.poz3 < -0.5:
                self.poz3 = -0.5
                self.get_logger().error(
                    "Reached limit of joint position! Min parameter 'poz3' value: " + str(self.poz3))

            # count time for interpolation
            time.sleep(self.timePeriod)
            # self.get_logger().info(str(self.timePassed))
            if self.timePassed < self.time:
                self.timePassed += self.timePeriod
            elif self.timePassed > self.time:
                self.timePassed = self.time  # zaokraglenie
                self.result = True
                break


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = Jint()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
