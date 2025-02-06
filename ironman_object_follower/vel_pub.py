import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Point #added to import Twist message type

class VelocityPublisher(Node):
     def __init__(self):        
        super().__init__('velocity_publisher')
        
        # image_qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     durability=QoSDurabilityPolicy.VOLATILE,
        #     depth=1
        # )

        self._pixel_subscriber = self.create_subscription(
            Point,
            'detected_pixel',
            self.pixel_callback, 
            10)
        self._pixel_subscriber 
        
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 10)

     def pixel_callback(self, msg: Point):


        twist = Twist()
        twist.linear.x = 0.0


        if msg.x < msg.y - 50:
            twist.angular.z = 0.5
        elif msg.x < msg.y  + 50:
            twist.angular.z = -0.5
        else:
            twist.angular.z = 0.0

            
        self._vel_publish.publish(twist)

def main():
    rclpy.init()
    velocity_publisher = VelocityPublisher()


    while rclpy.ok():
        rclpy.spin_once(velocity_publisher)
    
    velocity_publisher.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()