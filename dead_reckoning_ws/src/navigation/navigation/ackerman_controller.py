import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist ,Pose2D
import numpy as np

class AckermannWaypointController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        self.declare_parameter('wheelbase', 0.5)  # update this parameter
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('max_steering_angle', np.pi / 4)   #  change this angle accordingly 
        self.declare_parameter('constant_velocity', 25)    # constant PWM value     
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.constant_velocity = self.get_parameter('constant_velocity').value
        
        self.odom_sub = self.create_subscription(Pose2D, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)    # change this accordingly 
        
        self.goal = None
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        
        self.get_goal_from_user()

    def get_goal_from_user(self):
        self.goal = np.array([float(input("Enter goal x: ")), float(input("Enter goal y: ")), float(input("Enter goal yaw: "))])
    
    def odom_callback(self, msg):
        position_x = msg.x
        position_y = msg.y
        yaw = msg.theta 
        current_position = np.array([position_x, position_y])
        current_yaw = yaw

        distance_error, yaw_error = self.compute_errors(current_position, current_yaw)
        control_steering_angle = self.compute_steering_angle(distance_error, yaw_error)
        
        cmd_msg = Twist()
        cmd_msg.linear.x = self.constant_velocity
        cmd_msg.angular.z = control_steering_angle
        self.cmd_pub.publish(cmd_msg)

    def compute_errors(self, current_position, current_yaw):
        goal_position = self.goal[:2]
        goal_yaw = self.goal[2]
        
        goal_vector = goal_position - current_position
        distance_to_goal = np.linalg.norm(goal_vector)
        goal_heading = np.arctan2(goal_vector[1], goal_vector[0])
        
        yaw_error = goal_heading - current_yaw
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))  # Normalize to [-pi, pi]
        
        return distance_to_goal, yaw_error

    def compute_steering_angle(self, distance_to_goal, yaw_error):
        # Calculate the desired steering angle using Ackermann steering formula
        steering_angle = np.arctan(2 * self.wheelbase * np.sin(yaw_error) / distance_to_goal)
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)    # check this once 
        
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    node = AckermannWaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
