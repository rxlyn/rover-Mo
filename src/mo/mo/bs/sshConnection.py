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

            
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.action, ##Change this
            10)
        self.subscription

    def __del__(self):
        self.ssh_client.close()
        
    def listener_callback(self, msg):
        # Access the string from the received message (std_msgs/String stores the text in msg.data)
        received_string = msg.data
        self.get_logger().info(f"Received message: {received_string}")

        
    def action(self, msg):
        receivedString = msg.data
        self.get_logger().info(f"Received message: {receivedString}")
        if (receivedString == "drive"):
            self.ssh_client.exec_command("echo drive > ~/Desktop/motor_state.txt")
        elif (receivedString == "reverse"):
            self.ssh_client.exec_command("echo reverse > ~/Desktop/motor_state.txt")
        elif (receivedString == "spinClock"):
            self.ssh_client.exec_command("echo spinClock > ~/Desktop/motor_state.txt")
        elif (receivedString == "spinCounter"):
            self.ssh_client.exec_command("echo spinCounter > ~/Desktop/motor_state.txt")
        elif (receivedString == "stop"):
            self.ssh_client.exec_command("echo stop > ~/Desktop/motor_state.txt")
    
        

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