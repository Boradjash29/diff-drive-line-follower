#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector_node')
        self.subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.error_publisher = self.create_publisher(
            Point,
            '/line_error',
            10
        )
        self.image_publisher = self.create_publisher(
            Image,
            '/camera/image_processed',
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Line Detector Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Get image dimensions
            height, width, _ = cv_image.shape
            
            # Crop lower part of image for faster and more relevant processing
            crop_start = int(height * 0.7)
            cropped_image = cv_image[crop_start:height, 0:width]
            
            # Convert to grayscale
            gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            
            # Thresholding (Line is black, floor is white)
            # Lowered threshold to 60 to avoid picking up shadows as the line
            _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
            
            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            error_msg = Point()
            
            if len(contours) > 0:
                # Find the largest contour which is presumably the line
                c = max(contours, key=cv2.contourArea)
                
                # Calculate centroid
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Calculate error (image_center - line_center)
                    image_center = width / 2
                    error = image_center - cx
                    
                    error_msg.x = float(error)
                    error_msg.z = 1.0 # 1.0 means line is found
                    self.error_publisher.publish(error_msg)
                    
                    # Draw centroid for debugging
                    cv2.circle(thresh, (cx, cy), 5, (0, 0, 0), -1)
            else:
                error_msg.x = 0.0
                error_msg.z = 0.0 # 0.0 means line is lost
                self.error_publisher.publish(error_msg)
                
            # Publish the processed image so we can see what the robot sees
            processed_msg = self.bridge.cv2_to_imgmsg(thresh, 'mono8')
            self.image_publisher.publish(processed_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
