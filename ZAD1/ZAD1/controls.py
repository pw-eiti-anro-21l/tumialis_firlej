import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from curtsies import Input


class VelocityPub(Node):

    def __init__(self):
        # creating node with publisher
        super().__init__('velocityNode')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # setting up parameters and variables
        self.declare_parameter('goLeft', 'g')
        self.declare_parameter('goRight', 'j')
        self.declare_parameter('goUp', 'y')
        self.declare_parameter('goDown', 'h')
        self.goLeft = 'g'
        self.goRight = 'j'
        self.goUp = 'y'
        self.goDown = 'h'
        self.exitKey = 'q'

        self.get_logger().info(self.welcome_message())

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.velocity)
        self.i = 0
        self.running_start = 0  # timestamp when turtle has started a move
        self.running_period = 75  # how many iterations until move is considered finished # 5 * (0.1s + 0.1s) = 1s

        self.msg = Twist()
        self.velocity()

    # startup message
    def welcome_message(self):
        message = "\r\n\nWelcome!\nUse these (default) keys to move the turtle:\n" \
                  "LEFT - |" + self.goLeft + "|    RIGHT - |" + self.goRight \
                  + "|    FORWARD - |" + self.goUp + "|    BACKWARDS - |" + self.goDown + "|\n\n" \
                  "Use the |" + self.exitKey + "| button to exit\n "
        return message

    # assign parameters to variables
    def assign_params(self):
        self.goLeft = self.get_parameter('goLeft').get_parameter_value().string_value
        self.goRight = self.get_parameter('goRight').get_parameter_value().string_value
        self.goUp = self.get_parameter('goUp').get_parameter_value().string_value
        self.goDown = self.get_parameter('goDown').get_parameter_value().string_value

    # method to wait for expected input
    def keypress(self):
        with Input(keynames='curses', sigint_event=False) as input_generator:
            try:
                return input_generator.send(0.01)
            except KeyboardInterrupt:
                print('force quit')
                return self.exitKey

    def velocity(self):

        self.assign_params()

        # waiting for proper input
        key = self.keypress()

        # exiting if exitKey pressed
        if key == self.exitKey:
            exit()

        # angular velocity depending on pressed keys
        if key == self.goLeft:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 1.57
            self.running_start = self.i

        elif key == self.goRight:
            self.msg.linear.x = 0.0
            self.msg.angular.z = -1.57
            self.running_start = self.i

        elif self.running_start + self.running_period < self.i:
            self.msg.angular.z = 0.0

        # linear velocity depending on pressed keys
        if key == self.goUp:
            self.msg.angular.z = 0.0
            self.msg.linear.x = 1.5
            self.running_start = self.i

        elif key == self.goDown:
            self.msg.angular.z = 0.0
            self.msg.linear.x = -1.5
            self.running_start = self.i

        elif self.running_start + self.running_period < self.i:
            self.msg.linear.x = 0.0

        # publishing velocity
        self.publisher_.publish(self.msg)
        self.i += 1


def main():
    rclpy.init()
    velocity_pub = VelocityPub()
    rclpy.spin(velocity_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # velocity_pub.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
