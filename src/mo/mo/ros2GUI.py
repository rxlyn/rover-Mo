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

        #self.bridge = CvBridge()
        self.cmd_move = self.create_publisher(
            String,
            '/gui/move',
            10
        )

        self.cmd_override = self.create_publichser{
            String,
            '/gui/override',
            10
        }
        
        # ——— Subscriptions to five different GUI button topics ———
        buttons = ['FORWARD', 'REVERSE', 'CLOCK', 'COUNTER', 'STOP', 'SWITCH']
        for name in buttons:
            topic = f'/gui/{name}'
            # use a lambda to capture `name` in the loop
            self.create_subscription(
                String,
                topic,
                lambda msg, btn=name: self.on_button_press(btn, msg),
                10
            )

        self.button_to_cmd = {
            'FORWARD': (self.cmd_move, 'forward'),
            'REVERSE': (self.cmd_move, 'reverse'),
            'CLOCK': (self.cmd_move, 'clock'),
            'COUNTER': (self.cmd_move, 'counter'),
            'STOP': (self.cmd_move, 'stop'),
            'SWITCH': (self.cmd_override, 'switch'),
        }

    def on_button_press(self, button_name: str):
        pub, cmd_str = self.button_actions[button_name]
        pub.publish(String(data=cmd_str))
        self.get_logger().info(
            f"Button {button_name} → published '{cmd_str}'"
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = ROS2GUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
