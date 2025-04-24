#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from messages import MotorCommand

class ControlSubscriber(Node):
    def __init__(self):
        super().__init__('control_subscriber')
        # Subscribe to the control topic published by the GUI.
        self.subscription = self.create_subscription(
            String,
            '/control_prompts',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Control Subscriber Node has been started.')
        
        
        self.publisher = self.create_publisher(MotorCommand, 'MotorDrive', 10)

    def listener_callback(self, msg):
        try:
            # Parse the JSON string from msg.data
            data = json.loads(msg.data)
            motors = data.get('motors', [])
            if motors:
                motor_cmd = motors[0]
                self.get_logger().info(
                    f"Received control command - Motor Left: {motor_cmd.get('motorLeft')}, Motor Right: {motor_cmd.get('motorRight')}"
                )
            else:
                self.get_logger().warn("No motor commands found in the message.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse control message: {e}")
        
    def sendMotorCommand(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        

def main(args=None):
    rclpy.init(args=args)
    control_subscriber = ControlSubscriber()
    rclpy.spin(control_subscriber)
    control_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()