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

        # Example: Execute a command on the Raspberry Pi to list active ROS2 nodes.
        # (For ROS1 use "rosnode list")
        command = "ros2 node list"
        stdin, stdout, stderr = self.ssh_client.exec_command(command)
        output = stdout.read().decode().strip()
        errors = stderr.read().decode().strip()

        if output:
            self.get_logger().info(f"ROS2 Nodes on remote Raspberry Pi:\n{output}")
        if errors:
            self.get_logger().error(f"Error output:\n{errors}")
            
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.action(), ##Change this
            10)
        self.subscription

    def __del__(self):
        self.ssh_client.close()
        
        
    def action(self, commandName):
        if (commandName == "drive"):
            self.ssh.client.exec_command("")
        elif (commandName == "reverse"):
            self.ssh.client.exec_command("")
        elif (commandName == "spinClock"):
            self.ssh.client.exec_command("")
        elif (commandName == "spinCounter"):
            self.ssh.client.exec_command("")
        elif (commandName == "stop"):
            self.ssh.client.exec_command("")
    
        

def main(args=None):
    rclpy.init(args=args)
    node = SSHRemoteNode()
    # Run a brief spin to allow the initialization messages to be logged.
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()