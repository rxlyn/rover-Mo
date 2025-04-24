import rclpy
import requests
from rclpy.node import Node

from std_msgs.msg import String
import time
import math
import serial
from threading import Lock
from messages import MotorCommand
from messages import EncoderValues
from messages import MotorValues



# Parameters (Might need to change them)
ENCODER_CPR = 0 #find value
LOOP_RATE = 0 #find value
SERIAL_PORT = "/dev/ttyUSB0" #???
BAUD_RATE = 57600 #???
ESP32_IP = "http://172.16.211.169" #Get the real one from ramesh


#ENABLE DEBUG
SERIAL_DEBUG_ENABLE = False

class MotorSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        
        self.mutex = Lock()
        
        self.declare_parameter('encoder_cpr', value=ENCODER_CPR)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING: ENCODER CPR is 0")
        
        self.declare_parameter('loop_rate', value=LOOP_RATE)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING: LOOP RATE is 0")
        
        self.declare_parameter('serial_port', value=SERIAL_PORT)
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=BAUD_RATE)
        self.baud_rate = self.get_parameter('baud_rate').value
        
        self.declare_parameter('esp32_ip', value=ESP32_IP)
        self.esp32_ip = self.get_parameter('esp32_ip').value
        
        self.declare_parameter('serial_debug', value=SERIAL_DEBUG_ENABLE)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")
        
        
        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motorCommandCallback, ##Change this
            10)
        self.subscription  # prevent unused variable warning


        self.speed_pub = self.create_publisher(String, 'motorValues', 10)

        self.encoder_pub = self.create_publisher(String, 'encoderValues', 10)
        
        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.motor1spd = 0.0
        self.motor2spd = 0.0
        


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        
    def sendPWM(self, motor1PWM, motor2PWM):
        self.mutex.acquire()
        esp32_ip = self.esp32_ip
        requests.get(esp32_ip+f"/control?var=o&val={int(motor1PWM)}_{int(motor2PWM)}")
        self.mutex.release()

    def sendFeedback(self, motor1CommTime, motor2CommTime):
        self.mutex.acquire()
        esp32_ip = self.esp32_ip
        requests.get(esp32_ip+f"/control?var=m&val={int(motor1CommTime)}_{int(motor2CommTime)}")
        self.mutex.release()
        
    def sendEncoderRead(self):
        self.mutex.acquire()
        esp32_ip = self.esp32_ip
        resp=requests.get(esp32_ip+f"/control?var=e&val={int(0)}_{int(0)}")
        self.mutex.release()
        #resp = list(map(int,((response.content).decode('utf-8')).split()))
        
        #resp = self.send_command(f"e")
        if resp:
            return list(map(int,((resp.content).decode('utf-8')).split()))
            #return [int(raw_enc) for raw_enc in resp.split()]
        return []


    def motorCommandCallback(self, motorCommand):
        if (motorCommand.is_pwm):
            self.sendPWM(motor_command.motor1Req, motor_command.motor2Req)
        else:
            scaler = (1 / (2*math.pi)) * self.get_parameter('encoder_cpr').value * (1 / self.get_parameter('loop_rate').value)
            motor1CommTime = motor_command.motor1Req * scaler
            motor2CommTime = motor_command.motor2Req * scaler
            self.sendFeedback(motor1CommTime, motor2CommTime)

    def check_encoders(self):
        resp = self.send_encoder_read_command()
        if (resp):

            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            m1_diff = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            rads_per_ct = 2*math.pi/self.get_parameter('encoder_cpr').value
            self.m1_spd = m1_diff*rads_per_ct/time_diff
            self.m2_spd = m2_diff*rads_per_ct/time_diff

            spd_msg = MotorValues()
            spd_msg.mot_1_rad_sec = self.motor1spd
            spd_msg.mot_2_rad_sec = self.motor2spd
            self.speed_pub.publish(spd_msg)

            enc_msg = EncoderValues()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub.publish(enc_msg)

    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            #self.conn.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            ## Adapted from original
            c = ''
            value = ''
            while c != '\r':
                #c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()    

def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = MotorSubscriber()

    rate = motor_subscriber.create_rate(2)
    while rcply.ok():

        rclpy.spin_once(motor_subscriber)
        motor_subscriber.check_encoders()

    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()