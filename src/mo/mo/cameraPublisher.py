#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Create a publisher for the camera feed on topic 'camera_feed'
        self.publisherCam1 = self.create_publisher(CompressedImage, 'camera1/image/compressed', 10)
        self.publisherCam2 = self.create_publisher(CompressedImage, 'camera2/image/compressed', 10)
        self.publisherCamALPR = self.create_publisher(Image, 'camera3/image/image', 10)
        
        # Publish at a rate of 10 Hz
        self.timer = self.create_timer(0.1, self.publish_images)
        
        # Open the default camera (device index 0)
        self.cap0 = cv2.VideoCapture(0)
        self.cap1 = cv2.VideoCapture(1)
        self.capALPR = cv2.VideoCapture(2)
        self.bridge = CvBridge()

    def publish_images(self):
        # Camera Feed 1
        ret0, frame0 = self.cap0.read()
        if ret0:
            # Convert the OpenCV image (BGR) to a ROS Image message
            ros_image0 = self.bridge.cv2_to_imgmsg(frame0, encoding="bgr8")
            self.publisherCam1.publish(ros_image0)
            self.get_logger().info('Published a camera frame from camera 1')
        else:
            self.get_logger().error('Failed to capture frame from camera 1')
        
        # Camera Feed 2
        ret1, frame1 = self.cap1.read()
        if ret1:
            # Convert the OpenCV image (BGR) to a ROS Image message
            ros_image1 = self.bridge.cv2_to_imgmsg(frame1, encoding="bgr8")
            self.publisherCam2.publish(ros_image1)
            self.get_logger().info('Published a camera frame from camera 2')
        else:
            self.get_logger().error('Failed to capture frame from camera 2')

        # Camera Feed ALPR
        retALPR, frameALPR = self.capALPR.read()
        if retALPR:
            # Convert the OpenCV image (BGR) to a ROS Image message
            ros_imageALPR = self.bridge.cv2_to_imgmsg(frameALPR, encoding="bgr8")
            self.publisherCamALPR.publish(ros_imageALPR)
            self.get_logger().info('Published a camera frame from camera ALPR')
        else:
            self.get_logger().error('Failed to capture frame from camera ALPR')


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.get_logger().info('Shutting down camera publisher node...')
    finally:
        camera_publisher.cap0.release()
        camera_publisher.cap1.release()
        camera_publisher.capALPR.release()
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()