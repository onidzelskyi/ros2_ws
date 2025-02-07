import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Robot, Motor
import time

forward_speed = .2
backward_speed = .2
turn_speed = .25
speed_scale = .2


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_smooth',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.robot = Robot(left=Motor(4, 14), 
                           right=Motor(18, 17))

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x * speed_scale
        angular_z = msg.angular.z

        # Example logic to control motors based on cmd_vel
        if linear_x > 0:
            self.robot.forward(linear_x)
        elif angular_z > 0:
            # self.robot.stop()
            # time.sleep(1)
            self.robot.backward(turn_speed, curve_right=1)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()