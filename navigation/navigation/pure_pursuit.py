import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import math
import serial
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D, PoseStamped
from custom_msgs.msg import Goal  
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R

i = 0
k = 0

class GoTOGoal(Node):
    def __init__(self):
        super().__init__('Go_to_goal_nav')

        # Subscribe to the goal topic to receive goals
        self.sub_goal = self.create_subscription(Goal, '/goal_topic', self.goal_callback, 10)
        
        # Subscribe to the odometry topic to receive odometry data
        self.sub_odom_map = self.create_subscription(Odometry, '/odom', self.OdomCallback, 10)

        # Publisher to send PWM values for controlling the robot
        self.publisher = self.create_publisher(Int32MultiArray, '/pwm_val', 10)

        # Timer for the control loop to run at 10 Hz
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Arrays for steering angles and corresponding PWM values
        self.sample_angles = np.array([0, 20, 40, 45, 60, 80, 90, 100, 120, 135, 140, 160, 180]) * (math.pi / 180)
        self.angle_pwms = np.array([0, 441, 841, 975, 1064, 1627, 1800, 2133, 2243, 2675, 2747, 3205, 3781])

        self.throttle = 0
        self.steering_limit = 0.8727
        self.servo_angle = 1800

        # Serial port configuration for communication with the robot
        self.serial_port = '/dev/ttyUSB0'
        self.serial_baudrate = 9600
        self.serial = serial.Serial(self.serial_port, self.serial_baudrate, timeout=1)
        self.get_logger().info('Serial subscriber node initialized')

        self.pwm_val = Int32MultiArray()
        self.command = String()
        
        # Initialize robot position and goal variables
        self.bot_x = 0.0
        self.bot_y = 0.0
        self.bot_theta = 0.0
        self.bot_goal_x = []
        self.bot_goal_y = []

        self.dist_error = 0.0
        self.theta_error = 0.0
        self.e_theta = 0.0

        self.lookahead_dist = 0.9
        self.wheelbase = 1

    def goal_callback(self, msg: Goal):
        # Receive the goal coordinates from the Goal message
        self.bot_goal_x = msg.x_coordinates
        self.bot_goal_y = msg.y_coordinates

    def callback2(self, msg1: PoseStamped):
        global k, initial_x, initial_y
        if k == 0:
            # Initialize initial position
            initial_x = msg1.pose.position.x
            initial_y = msg1.pose.position.y
            k += 1
        else:
            # Update robot position relative to the initial position
            self.bot_x = (msg1.pose.position.x - initial_x)
            self.bot_y = (msg1.pose.position.y - initial_y)
            orientation_q = msg1.pose.orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            r = R.from_quat(quaternion)
            euler = r.as_euler('xyz', degrees=False)
            self.bot_theta = euler[2]

    def OdomCallback(self, msg1: Odometry):
        # Update robot position and orientation from odometry data
        self.bot_x = msg1.pose.pose.position.x
        self.bot_y = msg1.pose.pose.position.y

        orientation_q = msg1.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=False)
        self.bot_theta = euler[2]

    def control_loop(self):
        global i

        if not self.bot_goal_x or not self.bot_goal_y:
            self.get_logger().info("Waiting for Goals... ")
        
        elif len(self.bot_goal_x) != len(self.bot_goal_y):
            self.get_logger().info('Invalid goals')
        
        elif i < len(self.bot_goal_x):
            self.get_logger().info(f'Number of goals: {len(self.bot_goal_x)}')
            self.get_logger().info(f'Current goal number: {i+1}')

            # Calculate distance and angle errors
            self.dist_error = math.sqrt((self.bot_goal_x[i] - self.bot_x)**2 + (self.bot_goal_y[i] - self.bot_y)**2)
            self.theta_error = math.atan2((self.bot_goal_y[i] - self.bot_y), (self.bot_goal_x[i] - self.bot_x)) - self.bot_theta
            self.e_theta = math.atan2(math.sin(self.theta_error), math.cos(self.theta_error))
            
            # Calculate steering angle using pure pursuit control
            self.steering_angle = math.pi/2 + max(min(math.atan2(2 * self.wheelbase * math.sin(self.e_theta) / (self.dist_error + 0.01), 1.0), self.steering_limit), -self.steering_limit)
            self.servo_angle = np.interp(self.steering_angle, self.sample_angles, self.angle_pwms)

            if i < len(self.bot_goal_x) - 1:
                if abs(self.dist_error) > self.lookahead_dist:
                    self.throttle = 30
                else:
                    i += 1
            elif i == len(self.bot_goal_x) - 1:
                if abs(self.dist_error) > 0.15:
                    self.throttle = 30
                else:
                    i += 1
            else:
                pass
        else:
            self.throttle = 0
            self.servo_angle = 1800
            self.get_logger().info('All goals reached')

        self.pwm_val.data = [self.throttle, int(self.servo_angle)]
        self.publisher.publish(self.pwm_val)

        self.command = f'{self.throttle},{self.servo_angle}\n'
        self.get_logger().info(f'Sending command to ESP: {self.command}')
        self.serial.write(self.command.encode())

def main(args=None):
    rclpy.init(args=args)
    go_to_goal = GoTOGoal()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(go_to_goal)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        go_to_goal.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
