import numpy as np  
import rospy  
from sensor_msgs.msg import Imu  
import tf.transformations as tft  
import matplotlib.pyplot as plt  
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas  
from matplotlib.figure import Figure 

class EstState:
    def __init__(self):
        # 初始化状态估计类，用于存储速度、加速度、角度和角速度
        self.velocity = np.zeros(3)  # 速度
        self.acc = np.zeros(3)       # 加速度
        self.angle = np.zeros(3)     # 角度
        self.ang_vel = np.zeros(3)   # 角速度


class KalmanFilter_Move:         #用于对加速度和速度进行滤波的类
    def __init__(self):
        self.dt = 0.02  # 时间步长

        self._xhat = np.zeros((6, 1))  # 状态估计向量

        self._u = np.zeros((3, 1))  # 输入向量

        # 系统矩阵
        self._A = np.eye(6)  # 状态转移矩阵
        self._A[:3,3 :] = np.eye(3) * self.dt
        self._B = np.zeros((6, 3))  # 输入矩阵
        self._B[3:, :] = np.eye(3) * self.dt  # 将时间步长乘以单位矩阵

        # 测量矩阵
        self._C = np.array([[0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        # 协方差矩阵
        self._P = np.eye(6)  # 预测协方差矩阵
        self._Q = np.array([[0.2, 0, 0, 0, 0, 0],
                            [0, 0.2, 0, 0, 0, 0],
                            [0, 0, 0.2, 0, 0, 0],
                            [0, 0, 0, 0.2, 0, 0],
                            [0, 0, 0, 0, 0.2, 0],
                            [0, 0, 0, 0, 0, 0.2]])  # 过程噪声协方差矩阵
        self._R = np.array([[0.0001, 0, 0, 0, 0, 0],
                            [0, 0.0001, 0, 0, 0, 0],
                            [0, 0, 0.0001, 0, 0, 0],
                            [0, 0, 0, 0.2, 0, 0],
                            [0, 0, 0, 0, 0.2, 0],
                            [0, 0, 0, 0, 0, 0.2]])  # 测量噪声协方差矩阵

    def kalman_run(self, est, IMU_acc):
        # 预测步骤
        self._xhat = np.dot(self._A, self._xhat) + np.dot(self._B, self._u)
        self._P = np.dot(np.dot(self._A, self._P), self._A.T) + self._Q

        # 更新步骤
        _y = np.array([[est.velocity[0]], [est.velocity[1]], [est.velocity[2]], [IMU_acc[0]], [IMU_acc[1]], [IMU_acc[2]]])
        _yhat = np.dot(self._C, self._xhat)
        K = np.dot(np.dot(self._P, self._C.T), np.linalg.inv(np.dot(np.dot(self._C, self._P), self._C.T) + self._R))
        self._xhat += np.dot(K, (_y - _yhat))
        self._P = np.dot(np.eye(6) - np.dot(K, self._C), self._P)

        # 更新估计状态
        est.velocity = self._xhat[:3, 0]
        est.acc = self._xhat[3:6, 0]

class KalmanFilter_Rotate:           #用于对角度和角速度进行滤波的类
    def __init__(self):
        self.dt = 0.02

        self._xhat = np.zeros((6, 1))

        self._u = np.zeros((3, 1))

        self._A = np.eye(6)
        self._A[:3,3 :] = np.eye(3) * self.dt

        self._B = np.zeros((6, 3))
        self._B[3:, :] = np.eye(3) * self.dt
        self._C = np.eye(6)

        self._P = np.eye(6)
        self._Q = np.array([[0.2, 0, 0, 0, 0, 0],
                            [0, 0.2, 0, 0, 0, 0],
                            [0, 0, 0.2, 0, 0, 0],
                            [0, 0, 0, 0.2, 0, 0],
                            [0, 0, 0, 0, 0.2, 0],
                            [0, 0, 0, 0, 0, 0.2]])
        self._R = np.array([[0.3, 0, 0, 0, 0, 0],
                            [0, 0.3, 0, 0, 0, 0],
                            [0, 0, 0.3, 0, 0, 0],
                            [0, 0, 0, 0.5, 0, 0],
                            [0, 0, 0, 0, 0.5, 0],
                            [0, 0, 0, 0, 0, 0.5]])

    def kalman_run(self, est, IMU_angle, IMU_ang_vel):
        # 预测步骤
        self._xhat = np.dot(self._A, self._xhat) + np.dot(self._B, self._u)
        self._P = np.dot(np.dot(self._A, self._P), self._A.T) + self._Q

        # 更新步骤
        _y = np.array([[IMU_angle[0]], [IMU_angle[1]], [IMU_angle[2]], [IMU_ang_vel[0]], [IMU_ang_vel[1]], [IMU_ang_vel[2]]])
        _yhat = np.dot(self._C, self._xhat)
        K = np.dot(np.dot(self._P, self._C.T), np.linalg.inv(np.dot(np.dot(self._C, self._P), self._C.T) + self._R))
        self._xhat += np.dot(K, (_y - _yhat))
        self._P = np.dot(np.eye(6) - np.dot(K, self._C), self._P)

        # 更新估计状态
        est.angle = self._xhat[:3, 0]
        est.ang_vel = self._xhat[3:6, 0]

acc_ori = []
acc_filter = []
vel_ori = []
vel_filter = []
angle_ori = []
angle_filter = []

class IMU:
    def __init__(self):
        self.kf_move = KalmanFilter_Move()
        self.kf_rotate = KalmanFilter_Rotate()
        self.est_State = EstState()
        self.ori_acc = [0, 0, 0]
        self.ori_angle = [0, 0, 0]
        self.ori_angular_vel = [0, 0, 0]
        self.acc_remove_g = [0, 0, 0]
        rospy.Subscriber("/imu_data", Imu, self.IMU_data_callback)

    def print_imu_data(self):
        print('ori_acc_x:',self.ori_acc[0], 'acc_remove_g:' ,self.acc_remove_g[0], 'filter_vel_x:',self.est_State.velocity[0])
        acc_ori.append(self.ori_acc[0])
        vel_filter.append(self.est_State.velocity[0])
        angle_ori.append(self.ori_angle[0])
        angle_filter.append(self.est_State.angle[0])

    def get_imu_filter_data(self):
        return self.est_State
    
    def Euler_to_rotation_matrix(self, phi, theta, psi):
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)]
        ])

        R_y = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

        R_z = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def IMU_data_callback(self,ori_imu_data):
       
        q = (ori_imu_data.orientation.x, ori_imu_data.orientation.y, ori_imu_data.orientation.z, ori_imu_data.orientation.w);
        rpy = tft.euler_from_quaternion(q)
        self.ori_angle = [rpy[0], rpy[1], rpy[2]]
        self.ori_angular_vel = [ori_imu_data.angular_velocity.x, ori_imu_data.angular_velocity.y, ori_imu_data.angular_velocity.z]
        self.kf_rotate.kalman_run(self.est_State, self.ori_angle, self.ori_angular_vel)
        
        Rotation_Matrix = self.Euler_to_rotation_matrix(rpy[0], rpy[1], rpy[2])

        self.ori_acc = [ori_imu_data.linear_acceleration.x, ori_imu_data.linear_acceleration.y, ori_imu_data.linear_acceleration.z]
        self.acc_remove_g = np.dot(Rotation_Matrix, self.ori_acc) + [0, 0, -9.81]
        self.kf_move.kalman_run(self.est_State, self.acc_remove_g)
        




# Robot_State = EstState()
# kf_move = KalmanFilter_Move()
# kf_rotate = KalmanFilter_Rotate()

# vel_ori = []
# vel_filter = []
# angle_ori = []
# angle_filter = []

# plt.ion()

# def imu_data_filter_callback(ori_imu_data):
#     # IMU_acc = [0.1, 0.2, 0.3]
#     IMU_acc = [ori_imu_data.linear_acceleration.x, ori_imu_data.linear_acceleration.y, ori_imu_data.linear_acceleration.z]
#     kf_move.kalman_run(Robot_State, IMU_acc)
#     print("速度:", Robot_State.velocity)
#     print("加速度:", Robot_State.acc)
#     q = (ori_imu_data.orientation.x, ori_imu_data.orientation.y, ori_imu_data.orientation.z, ori_imu_data.orientation.w);
#     rpy = tft.euler_from_quaternion(q)
#     IMU_angle = [rpy[0], rpy[1], rpy[2]]
#     IMU_ang_vel = [ori_imu_data.angular_velocity.x, ori_imu_data.angular_velocity.y, ori_imu_data.angular_velocity.z]
#     kf_rotate.kalman_run(Robot_State, IMU_angle, IMU_ang_vel)

#     angle_ori.append(rpy[0])
#     angle_filter.append(Robot_State.angle[0])
#     print("角度:", Robot_State.angle)
#     print("角速度:", Robot_State.ang_vel)
#     # 更新原始和滤波后的角度列表  
#     angle_ori.append(rpy[0])  
#     angle_filter.append(Robot_State.angle[0])  



# 使用示例
# 主程序  
if __name__ == '__main__':  
    rospy.init_node('imu_filter', anonymous=True)
    # rospy.Subscriber("/imu_data", Imu, imu_data_filter_callback)  
    imu = IMU()
    rate = rospy.Rate(50)  # 50 Hz  
    while not rospy.is_shutdown():
        print('------------------------------')
        imu.print_imu_data()
    
        plt.clf()
 #       plt.plot(acc_ori, label='Original ', color='blue')  # 绘制angle_ori，标签为'Original Angle'，颜色为蓝色  
 #       plt.plot(vel_filter, label='Filtered ', color='red')  # 绘制angle_filter，标签为'Filtered Angle'，颜色为红色  
        plt.plot(angle_ori, label='Original ', color='blue')  # 绘制angle_ori，标签为'Original Angle'，颜色为蓝色  
        plt.plot(angle_filter, label='Filtered ', color='red')  # 绘制angle_filter，标签为'Filtered Angle'，颜色为红色  
        plt.pause(0.001)
        plt.ioff()
        rate.sleep()  

 
  

