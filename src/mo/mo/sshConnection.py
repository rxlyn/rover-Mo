#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import paramiko
from std_msgs.msg import String

class SSHRemoteNode(Node):
    def __init__(self):
        super().__init__('ssh_remote_node')
        self.get_logger().info("SSH Remote Node Initialized")

        # SSH parameters (replace with your Raspberry Pi details)
        hostname = '172.16.211.169'  # Raspberry Pi IP address
        username = 'capstone'        # Raspberry Pi username
        password = 'team5'      # Raspberry Pi password

        # Set up SSH client
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        try:
            self.ssh_client.connect(hostname=hostname, username=username, password=password)
            self.get_logger().info(f"Connected to {hostname} via SSH.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {hostname}: {e}")
            return

        # keep track of current state and its active subscriptions
        self.current_state = None
        self.state_subs = {}  # topic_name → subscription handle


        # define for each state which topics to subscribe to
        # format: state_name: [ (topic, msg_type, callback), ... ]
        self.state_topic_map = {
            'manual': [
                ('/motor_command_manual', String, self.sendControlManual),
            ],
            'auto': [
                ('/motor_command_auto', String, self.sendControlAuto),
            ],
        }

        self.create_subscription(
            String,
            '/gui/override',
            self.overrideState,
            10
        )

        self._update_state('manual')

    def overrideState(self, msg: String):
        if msg.data.strip().lower() != 'swap':
            self.get_logger().warn(f"Ignoring unknown state_cmd: '{msg.data}'")
            return

        # decide next state
        next_state = 'auto' if self.current_state == 'manual' else 'manual'
        self.get_logger().info(f"Swapping state: {self.current_state} → {next_state}")
        self._update_state(next_state)
        

    def __del__(self):
        self.ssh_client.close()

    def sendControlManual(self, msg):
        receivedString = msg.data
        self.get_logger().info(f"Received message: {receivedString}")
        if (receivedString == "forward"):
            self.ssh_client.exec_command("echo FORWARD > ~/Desktop/motor_state.txt")
        elif (receivedString == "backward"):
            self.ssh_client.exec_command("echo BACKWARD > ~/Desktop/motor_state.txt")
        elif (receivedString == "counter"):
            self.ssh_client.exec_command("echo COUNTER > ~/Desktop/motor_state.txt")
        elif (receivedString == "clock"):
            self.ssh_client.exec_command("echo CLOCK > ~/Desktop/motor_state.txt")
        elif (receivedString == "stop"):
            self.ssh_client.exec_command("echo STOP > ~/Desktop/motor_state.txt")

    
    def sendControlAuto(self, msg):
        receivedString = msg.data
        self.get_logger().info(f"Received message: {receivedString}")
        if (receivedString == "forward"):
            self.ssh_client.exec_command("echo FORWARD > ~/Desktop/motor_state.txt")
        elif (receivedString == "backward"):
            self.ssh_client.exec_command("echo BACKWARD > ~/Desktop/motor_state.txt")
        elif (receivedString == "counter"):
            self.ssh_client.exec_command("echo COUNTER > ~/Desktop/motor_state.txt")
        elif (receivedString == "clock"):
            self.ssh_client.exec_command("echo CLOCK > ~/Desktop/motor_state.txt")
        elif (receivedString == "stop"):
            self.ssh_client.exec_command("echo STOP > ~/Desktop/motor_state.txt")        

def main(args=None):
    rclpy.init(args=args)
    node = SSHRemoteNode()
    
    try:
        # Keep the node alive indefinitely
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
