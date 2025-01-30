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

"""
ROS2 node for publishing HC-SR04 ultrasonic sensor data.

This module contains a ROS2 node that reads distance measurements from an HC-SR04
ultrasonic sensor using the gpiozero library and publishes them as sensor_msgs/Range messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor


class UltrasonicPublisher(Node):
    """ROS2 node for publishing HC-SR04 ultrasonic sensor data."""

    def __init__(self):
        """Initialize the UltrasonicPublisher node."""
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(Range, 'ultrasonic_range', 10)
        self.sensor = DistanceSensor(echo=23, trigger=24)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz

    def timer_callback(self):
        """Callback function to publish sensor data at regular intervals."""
        distance = self.sensor.distance * 100  # Convert to cm
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_sensor'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1  # Example value, adjust as needed
        msg.min_range = 0.02  # Example value, adjust as needed
        msg.max_range = 4.0  # Example value, adjust as needed
        msg.range = distance

        self.publisher_.publish(msg)


def main(args=None):
    """Main function to initialize and run the UltrasonicPublisher node."""
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
