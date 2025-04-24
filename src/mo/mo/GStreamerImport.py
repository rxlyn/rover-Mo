#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from image_transport import ImageTransport


class GStreamerImport(Node):
    def __init__(self):
        super().__init__("GStreamer_import_node")
        self.get_logger().info("GStreamer Import Node Initialized")
        
        it.ImageTransport(self)
        self.pubs = [
            it.advertise('cameraLeft/image', 'compressed'),
            it.advertise('cameraRight/image', 'compressed'),
        ]
        self.bridge = CvBridge()
        
        #Change this to match gstreamer feed
        gst_ports = [5000, 5001]
        gst_pipeline = (
            "udpsrc port={port} "
            "! application/x-rtp,encoding-name=JPEG,payload=26 "
            "! rtpjpegdepay ! jpegdec ! videoconvert ! appsink"
        )
        
        self.caps = [
            cv2.VideoCapture(gst_template.format(port=p), cv2.CAP_STREAMER)
            for p in gst_ports
        ]
        
        self.create_timer(1.0/30.0, self.publish_frames)
        
        
    def publish_frames(self):
        for idx, cap in enumerate(self.caps):
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"cam{idx+1} read failed")
                continue
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pubs[idx].publish(msg)
        
        
def main():
    rclpy.init()
    node = GStreamerImport()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()