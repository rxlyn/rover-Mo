#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from image_transport import ImageTransport

class stereoProcess(Node):
    def __init__(self):
        super().__init__('stereoProcess_node')
        self.get_logger().info("Stereo Process Node Initialized")
        
        self.bridge = CvBridge()
        
        # Publisher for the computed depth map
        self.depth_pub = self.create_publisher(Image, 'depth/image_raw', 10)
        
        it = ImageTransport(self)
        
        self.subLeft = it.subscribe('cameraLeft/image', self.cbLeft, 'compressed')
        self.subRight = it.subscribe('cameraRight/image', self.cbRight, 'compressed')
        
        self.frameLeft = None
        self.frameRight = None
        
        # StereoBM Matcher (might need to tweak values)
        num_disp = 64   # must be divisible by 16
        block = 15      # must be odd
        self.matcher = cv2.StereoBM_create(numDisparities=num_disp, blockSize=block)
        
        
    def cbLeft(self, msg):
        self.frameLeft = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()
        #cv2.waitKey(1)
    
    def cbRight(self, msg):
        self.frameRight = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()
        #cv2.waitKey(1)
        
    def process(self):
        if self.frameLeft is None or self.frameRight is None:
            return

        # convert to grayscale
        gray_l = cv2.cvtColor(self.frameLeft,  cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(self.frameRight, cv2.COLOR_BGR2GRAY)

        # compute disparity (depth map)
        disp = self.matcher.compute(gray_l, gray_r)  # 16-bit signed int

        # normalize to 8-bit for display
        disp_norm = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
        depth_img = np.uint8(disp_norm)

        # publish depth map
        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='mono8')
        self.depth_pub.publish(depth_msg)

        """
        # display
        stereo_pair = np.hstack((self.frameLeft, self.frameRight))
        cv2.imshow("Stereo Pair", stereo_pair)
        cv2.imshow("Depth Map", depth_img)
        cv2.waitKey(1)
        """
            
    def __del__(self):
        self.ssh_client.close()

def main():
    rclpy.init()
    node = stereoProcess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()