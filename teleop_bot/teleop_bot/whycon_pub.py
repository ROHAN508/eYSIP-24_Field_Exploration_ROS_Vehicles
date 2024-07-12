#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import String message type from std_msgs package
import sys, tty, termios

class whyconController(Node):
    def __init__(self):
        super().__init__('teleop_controller')
        self.publisher = self.create_publisher(String, 'ackerman_commands', 10)  # Publish String messages
        self.timer_period = 4 # seconds
        self.timer = self.create_timer(self.timer_period, self.whycon_input)
        self.shut_node = False
        self.i = 0

    def whycon_input(self):

        msg = String()

        if self.i == 0 :# Read whycon input
            command = 'Forward'
            
        if self.i == 1 :

            command = 'Halt'
            self.timer.cancel()
            self.shut_node = True

        msg.data = command
        self.publisher.publish(msg)
        print(msg)

        self.i += 1

def main(args=None):

    rclpy.init(args=args)
    whycon_controller = whyconController()

    while not whycon_controller.shut_node:
        rclpy.spin_once(whycon_controller)
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()