import time
import os


# Generated Placeholder Code
STATE_FILE = '/tmp/motor_state.txt'

def read_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, 'r') as f:
            return f.read().strip()
    return None

def set_motor_state(state):
    # Replace this print with your actual motor control logic.
    print(f"Motor state changed to: {state}")

def main():
    last_state = None
    print("Motor control script running. Monitoring state changes...")
    while True:
        state = read_state()
        if state and state != last_state:
            set_motor_state(state)
            last_state = state
        time.sleep(0.1)  # Polling interval

if __name__ == '__main__':
    main()
