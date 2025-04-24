#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class ROS2GUI(Node):
    def __init__(self):
        super().__init__('GUI_bridge_node')

        self.bridge = CvBridge()
      
        # --- Publishers for compressed camera streams ---
        self.pub_cameraLeft = self.create_publisher(
            CompressedImage,
            '/camera0/image_raw/compressed',
            10
        )
        self.pub_cameraRight = self.create_publisher(
            CompressedImage,
            '/camera1/image_raw/compressed',
            10
        )


        # subscribe to your camera feeds 
        self.sub_cameraLeft = self.create_subscription(
            Image,
            '/cameraLeft/image',   
            self.cb_camLeft,
            10
        )
        self.sub_cameraRight = self.create_subscription(
            Image,
            '/cameraRight/image',
            self.cb_camRight,
            10
        )
      

        # --- Subscriber for GUI button presses ---
        self.sub_button = self.create_subscription(
            String,
            '/gui/button_press',
            self.on_button_press,
            10
        )


    def cb_camLeft(self, img_msg: Image):
        # convert ROS Image → OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # encode as JPEG
        comp_msg = CompressedImage()
        comp_msg.header   = img_msg.header
        comp_msg.format   = 'jpeg'
        comp_msg.data     = cv2.imencode('.jpg', cv_img)[1].tobytes()
        self.pub_camLeft.publish(comp_msg)

  def cb_camRight(self, img_msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        comp_msg = CompressedImage()
        comp_msg.header = img_msg.header
        comp_msg.format = 'jpeg'
        comp_msg.data   = cv2.imencode('.jpg', cv_img)[1].tobytes()
        self.pub_camRight.publish(comp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap0.release()
        node.cap1.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
