import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import math
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Pose2D
from custom_msgs.msg import Goal  

# Initialize global variables for storing coordinates
x_imu = [0.0]
y_imu = [0.0]
x_vicon = [0.0]
y_vicon = [0.0]
goal_x = []
goal_y = []
x_imu_last = 0.0
y_imu_last = 0.0
x_vicon_last = 0.0
y_vicon_last = 0.0
initial_x = 0.0
initial_y = 0.0
i = 0

class Plotter(Node):
    def __init__(self):
        # Initialize the node with the name 'vicon_plotter'
        super().__init__('vicon_plotter')
        
        # Subscription to Odometry messages from the IMU
        self.sub1 = self.create_subscription(Odometry, '/odom', self.callback1, 10)
        
        # Subscription to PoseStamped messages from the Vicon system
        self.sub2 = self.create_subscription(PoseStamped, '/vicon/RCCar/RCCar/pose', self.callback2, 10)
        
        # Subscription to custom Goal messages
        self.sub3 = self.create_subscription(Goal, '/goal_topic', self.callback3, 10)

    def callback1(self, msg: Odometry):
        global x_imu, y_imu, x_imu_last, y_imu_last
        
        # Append new IMU data if the distance from the last point is greater than 0.05
        if self.dist(msg.pose.pose.position.x, msg.pose.pose.position.y, x_imu_last, y_imu_last) > 0.05:
            x_imu.append(msg.pose.pose.position.x)
            y_imu.append(msg.pose.pose.position.y)
            x_imu_last = msg.pose.pose.position.x
            y_imu_last = msg.pose.pose.position.y

    def callback2(self, msg1: PoseStamped):
        global x_vicon, y_vicon, x_vicon_last, y_vicon_last, initial_x, initial_y, i
        
        # Store the initial position on the first callback
        if i == 0:
            initial_x = msg1.pose.position.x
            initial_y = msg1.pose.position.y
            i += 1
        
        # Append new Vicon data if the distance from the last point is greater than 0.05
        if self.dist(msg1.pose.position.x, msg1.pose.position.y, x_vicon_last, y_vicon_last) > 0.05:
            x_vicon.append(msg1.pose.position.x - initial_x)
            y_vicon.append(msg1.pose.position.y - initial_y)
            x_vicon_last = msg1.pose.position.x - initial_x
            y_vicon_last = msg1.pose.position.y - initial_y

    def callback3(self, msg: Goal):
        global goal_x, goal_y
        
        # Store goal coordinates from the Goal message
        goal_x = msg.x_coordinates
        goal_y = msg.y_coordinates

    def dist(self, x1, y1, x2, y2):
        # Calculate Euclidean distance between two points
        return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

def main():
    # Initialize the ROS client library
    rclpy.init()  
    
    # Create an instance of the Plotter node
    plot = Plotter()
    
    try:
        # Keep the node running until it is interrupted
        rclpy.spin(plot)
    except KeyboardInterrupt:
        pass  # Allows for graceful shutdown on Ctrl+C
    finally:
        # Ensure node destruction and ROS shutdown
        if rclpy.ok():
            plot.destroy_node()
            rclpy.shutdown()

    # Plotting after ROS shutdown
    plt.figure(figsize=(10, 8))
    
    # Plot IMU data
    plt.plot(x_imu, y_imu, 'b-', label='Expected Path')
    
    # Plot Vicon data
    plt.plot(x_vicon, y_vicon, 'r-', label='Ground Truth')
    
    # Plot goal path
    plt.plot(goal_x, goal_y, 'g-', label='Goal path')
    
    # Set plot title and labels
    plt.title('IMU vs Vicon Path Comparison')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    
    # Show legend and grid
    plt.legend()
    plt.grid(True)
    
    # Display the plot
    plt.show()

if __name__ == "__main__":
    main()
