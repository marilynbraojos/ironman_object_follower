#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from cv_bridge import CvBridge
import numpy as np
import cv2

# Import the custom message (adjust the package name accordingly)
from your_package_name.msg import DetectionPoint

###############################################################################
# Object Detection Node: Processes images, detects an object, and publishes
# a DetectionPoint message containing an action and the object's centroid.
###############################################################################
class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Create a QoS profile for the image topic
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscriber for the compressed image topic
        self._image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            image_qos
        )

        # Publisher for our custom detection message
        self._detection_publisher = self.create_publisher(
            DetectionPoint,
            'detection_point',
            10
        )

        self._bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to an OpenCV image
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("Image conversion failed: " + str(e))
            return

        self.find_object(frame)

    def find_object(self, frame):
        # Define an HSV range for the object color (example: green)
        lower_color = np.array([40, 75, 75])
        upper_color = np.array([80, 255, 255])
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        action = None
        detected_point = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 2000:  # Skip small contours
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2  # Centroid x-coordinate
            cy = y + h // 2  # Centroid y-coordinate

            # (Optional) Check the average intensity in the region of interest
            roi = mask[y:y+h, x:x+w]
            if np.mean(roi) / 255.0 < 0.35:
                continue

            detected_point = (cx, cy)

            # Determine the action based on the object's horizontal position
            image_center = frame.shape[1] // 2
            threshold = 50
            if cx < image_center - threshold:
                action = "left"
            elif cx > image_center + threshold:
                action = "right"
            else:
                action = "stop"
            # Process only the first qualifying contour
            break

        if detected_point is not None and action is not None:
            detection_msg = DetectionPoint()
            detection_msg.action = action
            detection_msg.point = Point(x=float(detected_point[0]),
                                        y=float(detected_point[1]),
                                        z=0.0)  # z-coordinate is 0 for 2D

            self._detection_publisher.publish(detection_msg)
            self.get_logger().info(f"Published detection: action={action}, point=({detected_point[0]}, {detected_point[1]})")

###############################################################################
# Velocity Publisher Node: Subscribes to the DetectionPoint message and publishes
# a corresponding Twist command to /cmd_vel.
###############################################################################
class VelocityPublisherNode(Node):
    def __init__(self):
        super().__init__('velocity_publisher_node')

        # Subscriber to the custom detection topic
        self._detection_subscriber = self.create_subscription(
            DetectionPoint,
            'detection_point',
            self.detection_callback,
            10
        )
        # Publisher for velocity commands
        self._velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def detection_callback(self, msg):
        action = msg.action
        twist = Twist()
        twist.linear.x = 0.0  # No forward/backward motion in this example

        if action == "left":
            twist.angular.z = 0.5  # Turn left
        elif action == "right":
            twist.angular.z = -0.5  # Turn right
        elif action == "stop":
            twist.angular.z = 0.0  # Stop rotating
        else:
            twist.angular.z = 0.0  # Default behavior

        self._velocity_publisher.publish(twist)
        self.get_logger().info(f"Received action: {action} -> Published Twist: angular.z = {twist.angular.z}")

###############################################################################
# Main: Initialize rclpy, create both nodes, add them to a MultiThreadedExecutor,
# and spin.
###############################################################################
def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    detection_node = ObjectDetectionNode()
    velocity_node = VelocityPublisherNode()

    # Use a MultiThreadedExecutor to run both nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(detection_node)
    executor.add_node(velocity_node)

    try:
        executor.spin()
    finally:
        # Cleanup: Destroy nodes and shutdown rclpy
        detection_node.destroy_node()
        velocity_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
