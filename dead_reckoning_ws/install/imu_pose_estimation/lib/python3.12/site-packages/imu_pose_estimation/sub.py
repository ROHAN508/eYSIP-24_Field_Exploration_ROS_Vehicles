import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from std_msgs.msg import Int32
import struct

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ackerman_commands',
            self.command_callback,
            10)
        self.publisher= self.create_publisher(Int32, '/pwm_val', 10)
        self.throttle = 0
        self.steering_angles = [1800,1345,2243]
        self.servo_angle = 0
        self.previous_throttle = 0 # 90 degrees 
        self.serial_port = '/dev/ttyUSB0'  # Adjust port as needed
        self.serial_baudrate = 9600
        self.serial = serial.Serial(self.serial_port, self.serial_baudrate, timeout=1)
        self.get_logger().info('Serial subscriber node initialized')
        self.pwm_val = Int32()

    def command_callback(self, msg):
        command = msg.data

        if command == "Forward":
            self.throttle = 20
            self.previous_throttle = self.throttle
            self.servo_angle = self.steering_angles[0]

        elif command == "Backward" :
            self.throttle = -20
            self.previous_throttle = self.throttle
            self.servo_angle = self.steering_angles[0]
        
        elif command == "Right":
            self.throttle = self.previous_throttle
            self.servo_angle = self.steering_angles[1]
        
        elif command == "Left" :
            self.throttle = self.previous_throttle
            self.servo_angle = self.steering_angles[2]
        
        elif command == "Halt":
            self.throttle = 0
            self.servo_angle = self.steering_angles[0]

        elif command == "Stop":
            self.throttle = 0
            self.servo_angle = self.steering_angles[0]
         
        else : 
            print ( "Invalid commands")

        self.pwm_val.data=self.throttle
        self.publisher.publish(self.pwm_val)
        self.get_logger().info(f'Received command: {command}')
        self.send_serial_command(command)

    def send_serial_command(self, command):
        command = f'{self.throttle},{self.servo_angle}\n'
        self.get_logger().info(f'Sending command to ESP: {command}')
        self.serial.write(command.encode())


def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = SerialSubscriber()
    rclpy.spin(serial_subscriber)
    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
