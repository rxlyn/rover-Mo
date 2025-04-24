#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandLinePublisher(Node):
    def __init__(self):
        super().__init__('command_line_publisher')
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("Ready to publish messages. Type 'exit' to quit.")

    def publish_message(self, message_text):
        msg = String()
        msg.data = message_text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {message_text}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandLinePublisher()
    
    try:
        while True:
            user_input = input("Enter a message: ")
            if user_input.lower() == 'exit':
                break
            node.publish_message(user_input)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()