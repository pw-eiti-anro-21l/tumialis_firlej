#! /usr/bin/env python
import time
from math import sin, cos, pi, sqrt
import threading
import rclpy
from time import sleep
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from zadanie4_oint_srv.srv import OintControlSrv
from zadanie5_oint_figure_srv.srv import OintControlFigureSrv
from copy import deepcopy


class Oint(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('oint')

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'baza'
        self.pose_stamped = PoseStamped()
        self.oint_path = Path()

        # robot state parameters
        self.declare_parameter('x', 1.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.5)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.x = 1.0
        self.y = 0.0
        self.z = 0.5
        self.roll = self.get_parameter('roll').get_parameter_value().double_value
        self.pitch = self.get_parameter('pitch').get_parameter_value().double_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value

        # interpolation parameters
        self.target_time = 0.0
        self.oldx = self.x
        self.oldy = self.y
        self.oldz = self.z
        self.oldroll = self.roll
        self.oldpitch = self.pitch
        self.oldyaw = self.yaw
        self.newx = 0.0
        self.newy = 0.0
        self.newz = 0.0
        self.newroll = 0.0
        self.newpitch = 0.0
        self.newyaw = 0.0
        self.interpolation_method = ""
        self.time_period = 0.05
        self.time_passed = 0.0
        self.max_error = 0.05
        self.err_poz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # threading
        self.result = False

        # ellipse limit flag
        self.limitReached = False

        qos_profile = QoSProfile(depth=10)
        self.oint_pub = self.create_publisher(PoseStamped, 'pose_stamped_oint', qos_profile)
        self.path_pub = self.create_publisher(Path, 'path_poses', qos_profile)
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.oint_control_srv = self.create_service(OintControlSrv, "interpolation_params_oint",
                                                    self.interpolation_params_callback)
        self.oint_control_figure_srv = self.create_service(OintControlFigureSrv, "interpolation_figure_params_oint",
                                                           self.interpolation_figure_params_callback)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        publsh_thread = threading.Thread(target=self.publish_state)
        publsh_thread.start()

    def interpolation_params_callback(self, request, response):

        self.newx = request.newx
        self.newy = request.newy
        self.newz = request.newz
        self.newroll = request.newroll
        self.newpitch = request.newpitch
        self.newyaw = request.newyaw

        if request.time <= 0.0:
            response.operation = "Provided time of movement must be greater than zero"
        else:
            if request.interpolation == "linear" or request.interpolation == "spline":
                self.target_time = request.time
                self.oldx = self.x
                self.oldy = self.y
                self.oldz = self.z
                self.oldroll = self.roll
                self.oldpitch = self.pitch
                self.oldyaw = self.yaw
                self.interpolation_method = request.interpolation
                self.time_passed = 0.0
                self.result = False

                thread = threading.Thread(target=self.update_state)  # create update_state loop on a different thread
                thread.start()  # start thread
                thread.join()  # wait for the thread to stop

                if self.result:
                    response.operation = "Successfully interpolated with " + request.interpolation + " method! " + \
                                         "Absolute errors: " + str(self.err_poz)
                else:
                    response.operation = "No success! Interpolation wasn't precise enough with " + \
                                         request.interpolation + " method! " + \
                                         "Absolute errors: " + str(self.err_poz)

            elif request.interpolation == "":
                response.operation = "Please provide interpolation method e.g. 'linear' or 'spline'"

            else:
                response.operation = "Not known interpolation method. Available methods: 'linear', 'spline'"

        return response

    def interpolation_figure_params_callback(self, request, response):

        self.newx = request.newx + (request.a * 0.5)
        self.newy = request.newy
        self.newz = request.newz
        self.newroll = 0
        self.newpitch = 0
        self.newyaw = 0

        if request.time <= 0.0 and request.a > 0.0 and request.b > 0.0:
            response.operation = "Provided time of movement must be greater than zero"
        else:
            if request.figure == "ellipse" or request.figure == "rectangle":
                self.target_time = 2.0
                self.oldx = self.x
                self.oldy = self.y
                self.oldz = self.z
                self.oldroll = self.roll
                self.oldpitch = self.pitch
                self.oldyaw = self.yaw
                self.interpolation_method = "linear"
                self.time_passed = 0.0
                self.result = False

                thread = threading.Thread(target=self.update_state)  # create update_state loop on a different thread
                thread.start()  # start thread
                thread.join()  # wait for the thread to stop

                if request.figure == "rectangle":
                    self.draw_rectangle(0, 0.5 * request.b, request.a + request.b, request.time)
                    self.draw_rectangle(-request.a, 0, request.a + request.b, request.time)
                    self.draw_rectangle(0, -request.b, request.a + request.b, request.time)
                    self.draw_rectangle(request.a, 0, request.a + request.b, request.time)
                    self.draw_rectangle(0, 0.5 * request.b, request.a + request.b, request.time)

                if request.figure == "ellipse":
                    number_of_steps_ = int(request.time / self.time_period)
                    for i in range(1, number_of_steps_ + 1):
                        self.draw_ellipse(0.5 * request.a, 0.5 * request.b, request.newx, request.newz, i,
                                          number_of_steps_)
                        if self.limitReached:
                            self.limitReached = False
                            break

                if self.result:
                    response.operation = "Successfully interpolated with " + request.figure + " method! " + \
                                         "Absolute errors: " + str(self.err_poz)
                else:
                    response.operation = "No success! Interpolation wasn't precise enough with " + \
                                         request.figure + " figure! " + \
                                         "Absolute errors: " + str(self.err_poz)

            elif request.figure == "":
                response.operation = "Please provide figure method e.g. 'ellipse' or 'rectangle'"

            elif request.a <= 0.0:
                response.operation = "Value of 'a' must be bigger than 0!"

            elif request.b <= 0.0:
                response.operation = "Value of 'b' must be bigger than 0!"

            else:
                response.operation = "Not known figure method. Available methods: 'ellipse' or 'rectangle'"

        return response

    def draw_rectangle(self, a, b, ab, time):
        self.newx = self.x + a
        self.newz = self.z + b

        if a + b <= 0:
            flaga = -1
        else:
            flaga = 1

        self.target_time = time * 0.5 * ((a + b) / ab) * flaga
        self.oldx = self.x
        self.oldy = self.y
        self.oldz = self.z
        self.interpolation_method = "linear"
        self.time_passed = 0.0
        self.result = False

        thread = threading.Thread(target=self.update_state)  # create update_state loop on a different thread
        thread.start()  # start thread
        thread.join()  # wait for the thread to stop

    def draw_ellipse(self, a, b, start_x, start_z, i, number_of_steps, ):
        x = start_x + a * cos(2 * pi * i / number_of_steps)
        z = start_z + b * sin(2 * pi * i / number_of_steps)

        if sqrt(x ** 2 + self.y ** 2) <= 1:
            self.x = x
        else:
            self.get_logger().error("CALCULATED LIMIT OF POSITION X")
            self.limitReached = True

        if 0.4 <= z <= 1:
            self.z = z
        else:
            self.get_logger().error("CALCULATED LIMIT OF POSITION Z")
            self.limitReached = True

        time.sleep(self.time_period)

        # self.target_time = time
        # self.oldx = self.x
        # self.oldy = self.y
        # self.oldz = self.z
        # self.interpolation_method = "linear"
        # self.time_passed = 0.0
        # self.result = False

        # thread = threading.Thread(target=self.update_state)  # create update_state loop on a different thread
        # thread.start()  # start thread
        # thread.join()  # wait for the thread to stop

    # method to publish new state even if not changing (to keep connection with rviz)
    def publish_state(self):
        while True:
            try:
                # update joint_state
                now = self.get_clock().now()

                self.pose_stamped.pose.position.x = self.x
                self.pose_stamped.pose.position.y = self.y
                self.pose_stamped.pose.position.z = self.z
                self.pose_stamped.pose.orientation = euler_to_quaternion(self.roll, self.pitch, self.yaw)
                # self.pose_stamped.pose.orientation = Quaternion(w=0.0, x=self.roll, y=self.pitch, z=self.yaw)

                self.oint_path.header.stamp = now.to_msg()
                self.oint_path.header.frame_id = 'odom'

                self.pose_stamped.header.stamp = now.to_msg()
                self.pose_stamped.header.frame_id = 'odom'

                self.oint_path.poses.append(deepcopy(self.pose_stamped))

                self.odom_trans.header.stamp = now.to_msg()

                # send the joint state and transform                
                self.oint_pub.publish(self.pose_stamped)
                self.path_pub.publish(self.oint_path)
                # self.broadcaster.sendTransform(self.odom_trans)
                time.sleep(self.time_period)

            except KeyboardInterrupt:
                exit(0)

    # method to interpolate all new positions of joints
    def update_state(self):
        while True:

            # change params
            # X-Y WORKSPACE WITH LIMITS
            if self.x != self.newx or self.y != self.newy:
                x = self.x
                y = self.y
                if self.x != self.newx:
                    x = interpolate(self.oldx, self.newx, 0, self.target_time, self.time_passed,
                                    self.interpolation_method)
                if self.y != self.newy:
                    y = interpolate(self.oldy, self.newy, 0, self.target_time, self.time_passed,
                                    self.interpolation_method)
                if sqrt(x ** 2 + y ** 2) <= 1.0:
                    self.x = x
                    self.y = y
                else:
                    self.get_logger().error("CALCULATED LIMIT OF POSITION XY")
                    self.result = self.check_reached_position()  # remember if targeted accuracy is met (and save errors)
                    break

            # Z WORKSPACE WITH LIMITS
            if self.z != self.newz:
                z = interpolate(self.oldz, self.newz, 0, self.target_time, self.time_passed,
                                self.interpolation_method)
                if 0.4 <= z <= 1:
                    self.z = z
                else:
                    self.get_logger().error("CALCULATED LIMIT OF POSITION Z")
                    self.result = self.check_reached_position()  # remember if targeted accuracy is met (and save errors)
                    break

            if self.roll != self.newroll:
                self.roll = interpolate(self.oldroll, self.newroll, 0, self.target_time, self.time_passed,
                                        self.interpolation_method)

            if self.pitch != self.newpitch:
                self.pitch = interpolate(self.oldpitch, self.newpitch, 0, self.target_time, self.time_passed,
                                         self.interpolation_method)

            if self.yaw != self.newyaw:
                self.yaw = interpolate(self.oldyaw, self.newyaw, 0, self.target_time, self.time_passed,
                                       self.interpolation_method)

            # count time for interpolation
            time.sleep(self.time_period)

            if self.time_passed == self.target_time:  # reached target time of movement
                self.result = self.check_reached_position()  # remember if targeted accuracy is met (and save errors)
                break

            if self.time_passed + self.time_period > self.target_time:  # if timePassed will reach target time in next iteration
                self.time_passed = self.target_time  # zaokraglenie

            if self.time_passed < self.target_time:  # if timePassed won't reach target time in next iteration next
                self.time_passed += self.time_period

    # check if current positions are equal to targeted with given accuracy
    def check_reached_position(self):
        self.count_abs_error()
        return self.err_poz[0] <= self.max_error and self.err_poz[1] <= self.max_error and self.err_poz[
            2] <= self.max_error and self.err_poz[3] <= self.max_error and self.err_poz[4] <= self.max_error and \
               self.err_poz[5] <= self.max_error

    def count_abs_error(self):
        self.err_poz = [abs(self.x - self.newx), abs(self.y - self.newy), abs(self.z - self.newz),
                        abs(self.roll - self.newroll), abs(self.pitch - self.newpitch), abs(self.yaw - self.newyaw)]


# method to count linear interpolated position for given current time
def interpolate_linear(x0, x1, t0, t1, time_passed):
    return x0 + ((x1 - x0) / (t1 - t0)) * (time_passed - t0)


# method to count spline cubic interpolated position for given current time
def interpolate_spline_cubic(x0, x1, t0, t1, time_passed):
    tx = (time_passed - t0) / (t1 - t0)  # wzor z wiki
    k1 = 0  # pochodna rowna zero v(t0) = 0
    k2 = 0  # pochodna rowna zero v(t1) = 0
    a = k1 * (t1 - t0) - (x1 - x0)
    b = - k2 * (t1 - t0) + (x1 - x0)
    qx = (1 - tx) * x0 + tx * x1 + tx * (1 - tx) * ((1 - tx) * a + tx * b)
    return qx


# method to interpolate with given method
def interpolate(x0, x1, t0, t1, time_passed, method):
    if method == "linear":
        return interpolate_linear(x0, x1, t0, t1, time_passed)
    elif method == "spline":
        return interpolate_spline_cubic(x0, x1, t0, t1, time_passed)


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    node = Oint()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
