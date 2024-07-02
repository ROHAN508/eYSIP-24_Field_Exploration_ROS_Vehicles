import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import time
import math
import serial
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from custom_msgs.msg import Goal  
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

i=0

class GoTOGoal(Node):
    def __init__(self):
        super().__init__('Go_to_goal_nav')
        self.sub_goal = self.create_subscription(Goal, '/goal_topic', self.goal_callback, 10)
        self.sub_odom = self.create_subscription(Pose2D, '/odom', self.OdomCallback, 10)
        self.publisher= self.create_publisher(Int32MultiArray, '/pwm_val', 10)
        self.sample_angles = np.array([0, 20,40 ,45, 60, 80, 90, 100, 120,135, 140, 160, 180])*(math.pi/180)
        self.angle_pwms = np.array([0, 441, 841, 975, 1064, 1627, 1800, 2133, 2243,2675, 2747, 3205, 3781 ])
        self.throttle = 0
        self.steering_limit= 0.8727
        self.servo_angle = 1800
        self.serial_port = '/dev/ttyUSB0'  # Adjust port as needed
        self.serial_baudrate = 9600
        self.serial = serial.Serial(self.serial_port, self.serial_baudrate, timeout=1)
        self.get_logger().info('Serial subscriber node initialized')
        self.pwm_val = Int32MultiArray()
        self.command = String()
        self.bot_x = 0.0
        self.bot_y = 0.0
        self.bot_theta = 0.0
        self.bot_goal_x = []
        self.bot_goal_y = []
        self.dist_error = 0.0
        self.theta_error = 0.0
        self.e_theta = 0.0

    def goal_callback(self, msg: Goal):
        self.bot_goal_x = msg.x_coordinates
        self.bot_goal_y = msg.y_coordinates
    
    def OdomCallback(self, msg1: Pose2D):
        global i
        self.bot_x = msg1.x
        self.bot_y = msg1.y
        self.bot_theta = msg1.theta

        if self.bot_goal_x == [] or self.bot_goal_y == []:
            self.get_logger().info("Waiting for Goals... ")
        
        elif len(self.bot_goal_x) != len(self.bot_goal_y):
            self.get_logger().info('invalid goals')
        
        elif i<len(self.bot_goal_x):
            self.get_logger().info(f'no of goals: {len(self.bot_goal_x)}')
            self.get_logger().info(f'goal number: {i+1}')
            self.dist_error = ((self.bot_goal_x[i] - self.bot_x)**2 + (self.bot_goal_y[i] - self.bot_y)**2)**0.5
            self.theta_error= math.atan2((self.bot_goal_y[i] - self.bot_y),(self.bot_goal_x[i] - self.bot_x)) - self.bot_theta
            self.e_theta = math.atan2(math.sin(self.theta_error), math.cos(self.theta_error))
            self.steering_angle = math.pi/2 + max(min(self.e_theta, self.steering_limit), -self.steering_limit)
            self.get_logger().info(f'theta goals: {math.atan2((self.bot_goal_y[i] - self.bot_y),(self.bot_goal_x[i] - self.bot_x))*(180/math.pi)}')
            self.get_logger().info(f'current thetA: {self.bot_theta*(180/math.pi)}')
            # self.get_logger().info(f'thetA error: {self.e_theta*(180/math.pi)}')
            

            if abs(self.e_theta) < 0.19:
                self.servo_angle = 1800
            else:
                self.servo_angle = np.interp(self.steering_angle , self.sample_angles , self.angle_pwms)

            # if abs(self.e_theta) > 0.19:
            #     if self.e_theta < 0:
            #         self.servo_angle = self.steering_angles[1]
            #     if self.e_theta > 0:
            #         self.servo_angle = self.steering_angles[2]
            # else:
            #     self.servo_angle = self.steering_angles[0]

            if abs(self.dist_error) > 0.3:
                self.throttle = 23
            else:
                i=i+1
        else:
            self.throttle = 0
            self.servo_angle = 1800
            self.get_logger().info('All goals reached')

        self.pwm_val.data=[self.throttle,int(self.servo_angle)]
        self.publisher.publish(self.pwm_val)

        self.command = f'{self.throttle},{self.servo_angle}\n'
        self.get_logger().info(f'Sending command to ESP: {self.command}')
        self.serial.write(self.command.encode())




def main(args=None):
    rclpy.init(args=args)
    go_to_goal = GoTOGoal()
    rclpy.spin(go_to_goal)
    go_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()