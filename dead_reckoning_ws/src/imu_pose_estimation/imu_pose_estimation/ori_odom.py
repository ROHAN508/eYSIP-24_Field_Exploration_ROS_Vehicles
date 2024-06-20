import rclpy
from rclpy.node import Node
import time
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

IMU_ACC_OFFSET = 0.0
i=0



class PoseEstimator(Node):
    def __init__(self):
        super().__init__('imu_pose_estimator')
        self.sub_pwm = self.create_subscription(Int32, '/pwm_val', self.pwm_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.callBack, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10))
        
        self.bias_list=np.zeros(50)
        self.pwm=0.0
        self.bias=0.0
        self.pwm_val = np.array([-40, -35 , -30, -25 , -20 , 0 , 20, 25, 30, 35, 40])
        self.speed = np.array([-0.945 , - 0.79 , -0.656, -0.508, -0.35, 0, 0.33, 0.529, 0.7165, 0.8675,1.028 ])
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
        # print(self.x)
        
    def pwm_callback(self, msg1: Int32):
        self.pwm = msg1.data


    def callBack(self, msg: Imu):
        global i, IMU_ACC_OFFSET
        self.linear_acc = msg.linear_acceleration 
        if self.pwm==0.0:
            self.current_time=time.time()
            self.dt=self.current_time-self.last_time
            self.x[1, 0] = 0.0
            self.acc_x=0.0
            if i< 50:
                self.bias_list[i]= self.linear_acc.x
                IMU_ACC_OFFSET=np.sum(self.bias_list)/50
                i=i+1
            else:
                i=0

            self.last_time=time.time()
        else:
            self.current_time=time.time()
            self.dt=self.current_time-self.last_time
            self.acc_x=self.linear_acc._x-IMU_ACC_OFFSET
            self.F = np.array([[1, self.dt], [0, 1]])
            self.B = np.array([[0.5 * self.dt**2], [self.dt]])
            self.u = np.array([[self.acc_x]])
            self.x_pred = self.F @ self.x + self.B @ self.u
            self.P_pred = self.F @ self.P @ self.F.T + self.Q
            self.z = np.array([[self.x[0, 0] + np.interp(self.pwm,self.pwm_val,self.speed)*self.dt], [np.interp(self.pwm,self.pwm_val,self.speed)]])
            self.y = self.z - (self.H @ self.x_pred)
            self.S = self.H @ self.P_pred @ self.H.T + self.R  
            self.K = self.P_pred @ self.H.T @ np.linalg.inv(self.S)
            self.x = self.x_pred + self.K @ self.y 
            self.P = (self.I - self.K @ self.H) @ self.P_pred 
            self.last_time = time.time()
        
        self.get_logger().info(f'Distance: {self.x[0, 0]}, Speed: {self.x[1, 0]}, Acc: {self.acc_x}')
        # self.get_logger().info(f'P {self.P}, T: {self.dt}, Z: {self.z}')





def main():
    rclpy.init()  # Initializing ROS
    pose_est = PoseEstimator()
    rclpy.spin(pose_est)
    pose_est.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()