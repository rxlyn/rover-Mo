#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
import cv2


class autonomy(Node):
    def __init__(self):
        super().__init__('autonomy_node')
        self.declare_parameter('depth_topic', '/stereo/depth')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('stop_distance', 0.5)  # meters
        
        depth_topic = self.get_parameter('depth_topic').value
        self.threshold = self.get_parameter('stop_distance').value


        self.cmd_pub = self.create_publisher(
            String,
            'motor_command',
            10
        )
        
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )
        
        self.stopped = False
        self.get_logger().info(
            f'Listening on "{depth_topic}" @ sensor_data QoS; stopping < {self.threshold} m'
        )
        
        # Subscribe to Depth Map
        self.sub = self.create_subscription(
                Image, 'depth/image_raw', self.cb_depth, 10
            )
                
    def depth_callback(self, msg: Image):
        # Convert depth Image (32FC1) → NumPy array of meters
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Mask out invalid (zero) readings
        valid = depth > 0.0
        if not np.any(valid):
            return   # no valid pixels in this frame

        # Fast min-distance check
        min_dist = float(np.nanmin(np.where(valid, depth, np.nan)))

        if min_dist < self.threshold and not self.stopped:
            self.get_logger().warn(f'Obstacle at {min_dist:.2f} m → STOP')
            self.publish_stop()
            self.stopped = True

        elif min_dist >= self.threshold and self.stopped:
            self.get_logger().info(f'Clear at {min_dist:.2f} m → RESUME')
            self.publish_drive()
            self.stopped = False
    
    def publish_drive(self):
        msg = String()
        msg.data = "drive"
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published: drive")
    
    def publish_stop(self):
        msg = String()
        msg.data = "stop"
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published: stop")
        
def main():
    rclpy.init()
    node = autonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()