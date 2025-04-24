import numpy as np

class MotorCommand:
    motor1Req = np.float32
    motor2Req = np.float32
    is_pwm = bool
    
class MotorValues:
    mot_1_rad_sec = np.float32
    mot_2_rad_sec = np.float32
    
class EncoderValues:
    mot_1_enc_val = np.int32
    mot_2_enc_val = np.int32