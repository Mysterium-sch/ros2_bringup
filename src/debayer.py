#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

class Debayer(Node):
    def __init__(self):
        super().__init__('debayer')
        self.bridge = CvBridge()
        self.declare_parameter("device", "")
        device = self.get_parameter('device').value
        self.image_pub_grey = self.create_publisher(Image, f'{device}/debayer/image_raw/grey', 10)
        self.image_pub_rgb = self.create_publisher(Image, f'{device}/debayer/image_raw/rgb', 10)
        self.image_sub = self.create_subscription(Image, f'/flir_camera/image_raw', self.im_callback, 10)
	self.image_com = self.create_publisher(CompressedImage, 'image/compressed', 10)
	
        self.image_sub

    def im_callback(self, msg):
        data = msg
        img_in_cv2 = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        gray = cv.cvtColor(img_in_cv2, cv.COLOR_BayerRG2GRAY)
        rgb = cv.cvtColor(img_in_cv2, cv.COLOR_BayerRG2BGR)
        gray_image = self.bridge.cv2_to_imgmsg(gray, "mono8")
        rgb_image = self.bridge.cv2_to_imgmsg(rgb, "rgb8")   
        img_msg = self.bridge.cv2_to_compressed_imgmsg(rgb)
        self.image_com.publish(img_msg)
        self.image_pub_grey.publish(gray_image)
        self.image_pub_rgb.publish(rgb_image)        

def main(args=None):
    rclpy.init(args=args)
    debayer_node = Debayer()
    rclpy.spin(debayer_node)
    debayer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
