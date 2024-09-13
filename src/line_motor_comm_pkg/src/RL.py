import rospy
from line_motor_comm_pkg.msg import allMotorsMsgBack
from line_motor_comm_pkg.msg import allMotorsMsgCmd
from enum import Enum
import random

class JointData:
    def __init__(self, position, speed, torque):
        self.position = position
        self.speed = speed
        self.torque = torque

class MotorName(Enum):
    ankle_motor_ll = 0
    ankle_motor_lr = 1
    ankle_motor_rl = 2
    ankle_motor_rr = 3
    hip_motor_r_pitch = 4
    hip_motor_r_rotate = 5
    hip_motor_l_pitch = 6
    hip_motor_l_rotate = 7
    line_motor_l = 8
    line_motor_r = 9

# 得到的数据包含10个电机，每个电机分别包含位置、速度、力矩三个量。
class RL:
    def __init__(self):
        self.motor_data = [JointData(0, 0, 0) for i in range(10)]
        rospy.Subscriber('/motor_to_rl', allMotorsMsgBack, self.motor_data_callback)
        self.MotorCmdPubliser = rospy.Publisher('/rl_to_motor', allMotorsMsgCmd, queue_size=1)

    def motor_data_callback(self, data):
        self.motor_data[MotorName.ankle_motor_ll.value].position = data.ankle_motor_ll_back.pos
        self.motor_data[MotorName.ankle_motor_ll.value].speed = data.ankle_motor_ll_back.spd
        self.motor_data[MotorName.ankle_motor_ll.value].torque = data.ankle_motor_ll_back.tau

        self.motor_data[MotorName.ankle_motor_lr.value].position = data.ankle_motor_lr_back.pos
        self.motor_data[MotorName.ankle_motor_lr.value].speed = data.ankle_motor_lr_back.spd
        self.motor_data[MotorName.ankle_motor_lr.value].torque = data.ankle_motor_lr_back.tau

        self.motor_data[MotorName.ankle_motor_rl.value].position = data.ankle_motor_rl_back.pos
        self.motor_data[MotorName.ankle_motor_rl.value].speed = data.ankle_motor_rl_back.spd
        self.motor_data[MotorName.ankle_motor_rl.value].torque = data.ankle_motor_rl_back.tau

        self.motor_data[MotorName.ankle_motor_rr.value].position = data.ankle_motor_rr_back.pos
        self.motor_data[MotorName.ankle_motor_rr.value].speed = data.ankle_motor_rr_back.spd
        self.motor_data[MotorName.ankle_motor_rr.value].torque = data.ankle_motor_rr_back.tau

        self.motor_data[MotorName.hip_motor_r_pitch.value].position = data.hip_motor_r_pitch_back.q
        self.motor_data[MotorName.hip_motor_r_pitch.value].speed = data.hip_motor_r_pitch_back.dq
        self.motor_data[MotorName.hip_motor_r_pitch.value].torque = data.hip_motor_r_pitch_back.tau

        self.motor_data[MotorName.hip_motor_r_rotate.value].position = data.hip_motor_r_rotate_back.q
        self.motor_data[MotorName.hip_motor_r_rotate.value].speed = data.hip_motor_r_rotate_back.dq
        self.motor_data[MotorName.hip_motor_r_rotate.value].torque = data.hip_motor_r_rotate_back.tau

        self.motor_data[MotorName.hip_motor_l_pitch.value].position = data.hip_motor_l_pitch_back.q
        self.motor_data[MotorName.hip_motor_l_pitch.value].speed = data.hip_motor_l_pitch_back.dq
        self.motor_data[MotorName.hip_motor_l_pitch.value].torque = data.hip_motor_l_pitch_back.tau

        self.motor_data[MotorName.hip_motor_l_rotate.value].position = data.hip_motor_l_rotate_back.q
        self.motor_data[MotorName.hip_motor_l_rotate.value].speed = data.hip_motor_l_rotate_back.dq
        self.motor_data[MotorName.hip_motor_l_rotate.value].torque = data.hip_motor_l_rotate_back.tau

        self.motor_data[MotorName.line_motor_l.value].position = data.line_motor_l_back.x
        self.motor_data[MotorName.line_motor_l.value].speed = 0
        self.motor_data[MotorName.line_motor_l.value].torque = 0

        self.motor_data[MotorName.line_motor_r.value].position = data.line_motor_r_back.x
        self.motor_data[MotorName.line_motor_r.value].speed = 0
        self.motor_data[MotorName.line_motor_r.value].torque = 0

    def get_motor_data(self):
        return self.motor_data

    def print_motor_data(self):
        for i in range(10):
            print('Motor', i, 'position:', self.motor_data[i].position, 'speed:', self.motor_data[i].speed, 'torque:', self.motor_data[i].torque)

    def pub_motor_cmd(self, motor_cmd):
        self.motor_cmd = motor_cmd
        msg = allMotorsMsgCmd()
        msg.ankle_motor_ll_cmd.pos = motor_cmd[MotorName.ankle_motor_ll.value].position
        msg.ankle_motor_ll_cmd.spd = motor_cmd[MotorName.ankle_motor_ll.value].speed
        msg.ankle_motor_ll_cmd.tau = motor_cmd[MotorName.ankle_motor_ll.value].torque

        msg.ankle_motor_lr_cmd.pos = motor_cmd[MotorName.ankle_motor_lr.value].position
        msg.ankle_motor_lr_cmd.spd = motor_cmd[MotorName.ankle_motor_lr.value].speed
        msg.ankle_motor_lr_cmd.tau = motor_cmd[MotorName.ankle_motor_lr.value].torque

        msg.ankle_motor_rl_cmd.pos = motor_cmd[MotorName.ankle_motor_rl.value].position
        msg.ankle_motor_rl_cmd.spd = motor_cmd[MotorName.ankle_motor_rl.value].speed
        msg.ankle_motor_rl_cmd.tau = motor_cmd[MotorName.ankle_motor_rl.value].torque

        msg.ankle_motor_rr_cmd.pos = motor_cmd[MotorName.ankle_motor_rr.value].position
        msg.ankle_motor_rr_cmd.spd = motor_cmd[MotorName.ankle_motor_rr.value].speed
        msg.ankle_motor_rr_cmd.tau = motor_cmd[MotorName.ankle_motor_rr.value].torque

        msg.hip_motor_r_pitch_cmd.q = motor_cmd[MotorName.hip_motor_r_pitch.value].position
        msg.hip_motor_r_pitch_cmd.dq = motor_cmd[MotorName.hip_motor_r_pitch.value].speed
        msg.hip_motor_r_pitch_cmd.tau = motor_cmd[MotorName.hip_motor_r_pitch.value].torque

        msg.hip_motor_r_rotate_cmd.q = motor_cmd[MotorName.hip_motor_r_rotate.value].position
        msg.hip_motor_r_rotate_cmd.dq = motor_cmd[MotorName.hip_motor_r_rotate.value].speed
        msg.hip_motor_r_rotate_cmd.tau = motor_cmd[MotorName.hip_motor_r_rotate.value].torque

        msg.hip_motor_l_pitch_cmd.q = motor_cmd[MotorName.hip_motor_l_pitch.value].position
        msg.hip_motor_l_pitch_cmd.dq = motor_cmd[MotorName.hip_motor_l_pitch.value].speed
        msg.hip_motor_l_pitch_cmd.tau = motor_cmd[MotorName.hip_motor_l_pitch.value].torque

        msg.hip_motor_l_rotate_cmd.q = motor_cmd[MotorName.hip_motor_l_rotate.value].position
        msg.hip_motor_l_rotate_cmd.dq = motor_cmd[MotorName.hip_motor_l_rotate.value].speed
        msg.hip_motor_l_rotate_cmd.tau = motor_cmd[MotorName.hip_motor_l_rotate.value].torque

        msg.line_motor_l_cmd.x = motor_cmd[MotorName.line_motor_l.value].position
        msg.line_motor_r_cmd.x = motor_cmd[MotorName.line_motor_r.value].position
        self.MotorCmdPubliser.publish(msg)

    
def __main__():
    rospy.init_node('RL')
    rl = RL()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rl.print_motor_data()
        print('-------------------')
        # 发送例程
        motor_cmd = [JointData(random.uniform(-1, 1), random.uniform(-1, 1), 0) for i in range(10)]
        rl.pub_motor_cmd(motor_cmd)
        rate.sleep()

if __name__ == '__main__':
    __main__()


