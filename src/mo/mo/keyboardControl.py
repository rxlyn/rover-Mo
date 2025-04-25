#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # publisher for the current direction
        self.pub = self.create_publisher(String, '/motor_command_manual', 10)

        # set of currently pressed keys (w, a, s, d)
        self.pressed = set()
        self.last_direction = None

        # start listening to keyboard in a background thread
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # timer to check/make a publish every 100 ms
        self.create_timer(0.1, self.publish_direction)

    def on_press(self, key):
        try:
            c = key.char.lower()
            if c in ('w', 'a', 's', 'd'):
                self.pressed.add(c)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            c = key.char.lower()
            if c in self.pressed:
                self.pressed.remove(c)
        except AttributeError:
            pass

    def publish_direction(self):
        # map the pressed-set to one of the 8 directions
        p = self.pressed
        if p == {'w'}:
            direction = 'forward'
        elif p == {'w', 'd'}:
            direction = 'twistRight'
        elif p == {'d'}:
            direction = 'clock'
        elif p == {'s'}:
            direction = 'backward'
        elif p == {'a'}:
            direction = 'counter'
        elif p == {'w', 'a'}:
            direction = 'twistLeft'
        else:
            direction = 'stop'

        # only publish when it actually changes
        if direction != self.last_direction:
            msg = String(data=direction)
            self.pub.publish(msg)
            self.get_logger().info(f'Published direction → {direction}')
            self.last_direction = direction

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
