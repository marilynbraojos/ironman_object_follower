import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class MinimalVideoSubscriber(Node):

    def __init__(self):        
        super().__init__('minimal_video_subscriber')
        
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._video_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback, 
            image_qos_profile)
        self._video_subscriber 
        
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 10)

    def _image_callback(self, CompressedImage):    
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        self.find_object(self._imgBGR)

    def find_object(self, frame):
        lower_color = np.array([40, 75, 75])
        upper_color = np.array([80, 255, 255])

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bin_mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        contours, _ = cv2.findContours(bin_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < 2000:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w // 2, y + h // 2

            mask_roi = bin_mask[y:y + h, x:x + w]
            avg_intensity = np.mean(mask_roi) / 255.0

            if avg_intensity < 0.35:
                continue
            
            image_center_x = frame.shape[1] // 2
            if cx < image_center_x - 50:
                self.publish_velocity(turn_left=True)
            elif cx > image_center_x + 50:
                self.publish_velocity(turn_right=True)
            else:
                self.publish_velocity(stop=True)

    def publish_velocity(self, turn_left=False, turn_right=False, stop=False):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        if turn_left:
            twist.angular.z = 0.5
        elif turn_right:
            twist.angular.z = -0.5
        elif stop:
            twist.angular.z = 0.0

        self._vel_publish.publish(twist)


def main():
    rclpy.init()
    video_subscriber = MinimalVideoSubscriber()

    while rclpy.ok():
        rclpy.spin_once(video_subscriber)
    
    video_subscriber.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()