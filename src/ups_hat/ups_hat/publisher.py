# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import smbus

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('baterry_level_detection_publisher')
        self.publisher_ = self.create_publisher(Float32, 'INA219', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bus = smbus.SMBus(1)
        self.addr = 0x2d

    def timer_callback(self):
        data = self.bus.read_i2c_block_data(self.addr, 0x10, 0x06)
        msg = Float32()
        msg.data = float(data[2] | data[3] << 8)
        self.publisher_.publish(msg)
        self.get_logger().info("VBUS Current %5dmA" % (data[2] | data[3] << 8))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
