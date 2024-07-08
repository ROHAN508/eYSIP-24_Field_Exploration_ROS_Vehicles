import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose2D , TransformStamped
import tf2_ros
IMU_ACC_OFFSET = 0.0
IMU_YAW_OFFSET =0.0
i=0
j=0



class PoseEstimator(Node):
    def __init__(self):
        super().__init__('Dead_reckoning_node')
        self.sub_pwm = self.create_subscription(Int32MultiArray, '/pwm_val', self.pwm_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/raw', self.callBack, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10))
        self.br = tf2_ros.TransformBroadcaster(self)
        self.pub = self.create_publisher(Pose2D, '/odom', 10)
        self.Pose = Pose2D()
        self.bias_list=np.zeros(50)
        self.bias_yaw_list=np.zeros(50)
        self.pwm=0.0
        self.bias=0.0
        self.pwm_val = np.array([-30, -25 , -20 , 0 , 20, 25, 30, 35, 40])
        self.speed = np.array([-0.707, -0.545, -0.36 ,0, 0.40, 0.57, 0.74, 0.89,1.06 ])
        self.x = np.array([[0.0], [0.0]])
        self.P = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.Q = np.array([[0.001, 0.0], [0.0, 0.003]])
        self.R = np.array([[0.5, 0.0], [0.0, 0.5]])
        self.H = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.I = np.eye(2)
        self.last_time = time.time()
        self.dt=0.0
        self.acc_x=0.0
        self.z=0
        self.yaw = 0.0
        self.prev_dist = 0.0
        self.vector_length =0.0
        self.pwm_throttle = 0
        self.pwm_servo = 1800
        # print(self.x)
        
    def pwm_callback(self, msg1: Int32MultiArray):
        self.pwm = msg1.data
        self.pwm_throttle = self.pwm[0]
        self.pwm_servo = self.pwm[1]


    def callBack(self, msg: Imu):
        global i, j, IMU_ACC_OFFSET, IMU_YAW_OFFSET
        self.linear_acc = msg.linear_acceleration 
        # self.quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        self.angular_vel = msg.angular_velocity
        # self.euler = list(R.from_quat(self.quaternion).as_euler('xyz'))
        # self.roll, self.pitch, self.yaw = euler_from_quaternion(self.quaternion)
        # self.get_logger().info(f'yaw : {self.yaw}')
        self.current_time=time.time()
        self.dt=self.current_time-self.last_time
        self.last_time=time.time()
        # if self.pwm_servo==1800:
        #     pass
        # else:
        #     self.yaw = self.yaw + self.angular_vel.z*self.dt
        if j < 50:
            self.bias_yaw_list[j] = msg.angular_velocity.z
            self.get_logger().info(f'bias: {msg.angular_velocity.z}')
            j=j+1
            if  j == 49:
                IMU_YAW_OFFSET = np.sum(self.bias_yaw_list)/50
                self.get_logger().info(f'IMU bias calculated: {IMU_YAW_OFFSET}')
        else:
            # self.yaw_rate = msg.angular_velocity.z - self.IMU_YAW_OFFSET

            self.yaw = self.yaw + (self.angular_vel.z-IMU_YAW_OFFSET)*self.dt

            if self.pwm_throttle==0.0:
                self.x[1, 0] = 0.0
                self.acc_x=0.0
                if i< 50:
                    self.bias_list[i]= self.linear_acc.x
                    IMU_ACC_OFFSET=np.sum(self.bias_list)/50
                    i=i+1
                else:
                    i=0

            else:
                self.acc_x=self.linear_acc._x-IMU_ACC_OFFSET
                self.F = np.array([[1, self.dt], [0, 1]])
                self.B = np.array([[0.5 * self.dt**2], [self.dt]])
                self.u = np.array([[self.acc_x]])
                self.x_pred = self.F @ self.x + self.B @ self.u
                self.P_pred = self.F @ self.P @ self.F.T + self.Q
                self.z = np.array([[self.x[0, 0] + np.interp(self.pwm_throttle,self.pwm_val,self.speed)*self.dt], [np.interp(self.pwm_throttle,self.pwm_val,self.speed)]])
                self.y = self.z - (self.H @ self.x_pred)
                self.S = self.H @ self.P_pred @ self.H.T + self.R  
                self.K = self.P_pred @ self.H.T @ np.linalg.inv(self.S)
                self.x = self.x_pred + self.K @ self.y 
                self.P = (self.I - self.K @ self.H) @ self.P_pred
                self.vector_length = self.x[0,0] - self.prev_dist
                self.prev_dist = self.x[0,0]
                self.Pose.x = self.Pose.x + self.vector_length*math.cos(self.yaw)
                self.Pose.y = self.Pose.y + self.vector_length*math.sin(self.yaw)
                self.Pose.theta =math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # self.get_logger().info(f'Distance: {self.x[0, 0]}, Speed: {self.x[1, 0]}, Acc: {self.acc_x}, yaw: {round(self.yaw, 2)}')
            self.get_logger().info(f'X: {self.Pose.x}, Y: {self.Pose.y}, yaw: {round(self.Pose.theta, 2)}')
            self.get_logger().info(f'acc off: {IMU_ACC_OFFSET}')
            self.pub.publish(self.Pose)
            self.publish_transform()

            # self.get_logger().info(f' yaw: {round(self.yaw, 2)}')
        # self.get_logger().info(f'Distance: {self.x[0, 0]}, Speed: {self.x[1, 0]}, Acc: {self.acc_x}, yaw: {self.angular_vel.z}')

        # self.get_logger().info(f'P {self.P}, T: {self.dt}, Z: {self.z}')
    def publish_transform(self):                          # Function to publish TF of 'odom' w.r.t to 'base_link' 
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.Pose.x
        t.transform.translation.y = self.Pose.y
        t.transform.translation.z = 0.0
        q = R.from_euler('z', self.Pose.theta).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)
        
        self.get_logger().info(f'angle is {self.yaw}')

        # self.get_logger().info(f'P {self.P}, T: {self.dt}, Z: {self.z}')





def main():
    rclpy.init()  # Initializing ROS
    pose_est = PoseEstimator()
    rclpy.spin(pose_est)
    pose_est.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()