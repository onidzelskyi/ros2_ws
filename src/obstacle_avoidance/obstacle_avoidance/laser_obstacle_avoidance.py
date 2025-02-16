import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.subscription = self.create_subscription(
            Range,
            '/ultrasonic_range',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_smooth = self.create_publisher(Twist, '/cmd_vel_smooth', 10)
        self.min_distance = 50.
        self.max_distance = 80.

    def scan_callback(self, msg):
        # Extract distance measurements from the laser scan data
        front_distance = msg.range

        twist = Twist()
        
        if front_distance < self.min_distance:
            # Obstacle detected in front, turn
            twist.angular.z = .5  # Turn rate
        else:
            # No obstacle, move forward
            twist.linear.x = 1.  # Forward speed
        
        self.publisher.publish(twist)

        # Smooth
        twist_smooth = Twist()
                
        if front_distance > self.max_distance:
            twist_smooth.linear.x = 1.
        elif front_distance < self.min_distance:
            twist_smooth.linear.x = .0
            twist_smooth.angular.z = .5
        else:
            twist_smooth.linear.x = (np.log(front_distance) - np.log(self.min_distance)) / (np.log(self.max_distance) - np.log(self.min_distance))

        self.publisher_smooth.publish(twist_smooth)


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    
    # Create an instance of the ObstacleAvoidance node
    obstacle_avoidance_node = ObstacleAvoidance()
    
    # Keep the node running and processing callbacks
    rclpy.spin(obstacle_avoidance_node)
    
    # Shutdown the node and ROS 2 properly
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()