#include "Modbus_Comm.h"
#include <cmath>
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "B1MotorControl.h"
#include "conio.h"
#include <sensor_msgs/Imu.h>
#include "tf/tf.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include "util.h"
#include "ankle_motor_node.h"
#include "ankle_comm.h"
#include "robot_main_node.h"



#include "line_motor_comm_pkg/hipMotorMsgBack.h"
#include "line_motor_comm_pkg/hipMotorMsgCmd.h"
#include "line_motor_comm_pkg/ankleMotorMsgBack.h"
#include "line_motor_comm_pkg/ankleMotorMsgCmd.h"
#include "line_motor_comm_pkg/linemotorMsgBack.h"
#include "line_motor_comm_pkg/linemotorMsgCmd.h"
#include "line_motor_comm_pkg/allMotorsMsgCmd.h"
#include "line_motor_comm_pkg/allMotorsMsgBack.h"

/* 本文件中的全局变量 */
// 名称
std::vector<std::string> hip_motor_name_ = { 
								"hip_lb",
                                "hip_ll",  
                                "hip_rb",
                                "hip_rr", 
                                };
// 踝关节电机命名
std::vector<std::string> ankle_motor_name = {
                                "ankle_motor_ll",
                                "ankle_motor_lr",
                                "ankle_motor_rl",
                                "ankle_motor_rr"
                                };
// 所有电机数据通信包
line_motor_comm_pkg::allMotorsMsgCmd all_motors_cmd;        // 所有电机控制数据
line_motor_comm_pkg::allMotorsMsgBack all_motors_state;     // 所有电机返回数据
// 踝关节电机控制命令通信包 
line_motor_comm_pkg::ankleMotorMsgCmd ankle_motor_ll_msg, ankle_motor_lr_msg;
line_motor_comm_pkg::ankleMotorMsgCmd ankle_motor_rl_msg, ankle_motor_rr_msg;
// 髋关节电机控制命令通信包
line_motor_comm_pkg::linemotorMsgCmd linemotor_left_cmd_msg, linemotor_right_cmd_msg;
// 踝关节电机返回值
float current_pos[4] = {0};
float current_spd[4] = {0};
float current_tau[4] = {0};
// 膝关节电机返回值
volatile uint16_t status_word = 0x00;
volatile uint8_t go_zero_status_1 = 0;
volatile uint8_t go_zero_status_2 = 0;
volatile uint8_t drv_ready = 0;
float RTPos_left;//左腿绝对位置,1mm对应50编码器值
float RTPos_right;//右腿绝对位置
// 髋关节电机数据
const float StartPos_motorL_0 = 0.26889;		// 0.288463
const float StartPos_motorL_2 = 0.529233;		// 0.402892
const float StartPos_motorR_0 = 0.32637;		// 0.341737
const float StartPos_motorR_2 = 0.122134;		// 0.257421
// 姿态传感器返回值
volatile double angular_vel[3];
volatile double acceleration[3];
double angular[3];
// 机器人状态
RL_MotorsCmd rl_motor_cmd;
enum ROBOT_STATE robot_state = ROBOT_START_MODE;
// 电机状态
enum MOTOR_STATE motor_ref_state = MOTOR_INIT_STATE;

/* 本文件中的函数 */
void Receive_Linemotor_Fcn(int CANIndex);
void Receive_Zero_Fcn(int CANIndex);
void imu_feedback(sensor_msgs::Imu imu_data);
int8_t CanPort_Open(void);

void LineMotorCmdCallback(const line_motor_comm_pkg::linemotorMsgBack::ConstPtr& msg, int linemotorID); // 直线电机回调函数，读取直线电机状态
void AnkleMotorPubCallback(const line_motor_comm_pkg::ankleMotorMsgBack::ConstPtr& msg, int anklemotorID); // 髋关节旋转电机回调函数，读取旋转电机状体
void allMotorsMsgBack_Pub();
// 踝关节电机指令输入函数
void AnkleMotorCMD(int motor_num, int motor_state, float kp, float kd, float pos, float spd, float tau, float limit_current);

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_main_node");
    ros::NodeHandle nh;
	ros::Rate loop_rate(200);

    // 膝关节发布与订阅
    ros::Publisher linemotor_left_cmd_pub = nh.advertise<line_motor_comm_pkg::linemotorMsgCmd>("linemotor_0_cmd", 1);
	ros::Publisher linemotor_right_cmd_pub = nh.advertise<line_motor_comm_pkg::linemotorMsgCmd>("linemotor_1_cmd", 1);
	int linemotorID = 0;
    ros::Subscriber linemotor_left_state_sub = nh.subscribe<line_motor_comm_pkg::linemotorMsgBack>("left_knee_state", 1, boost::bind(&LineMotorCmdCallback, _1, linemotorID));
	linemotorID = 1;
	ros::Subscriber linemotor_right_state_sub = nh.subscribe<line_motor_comm_pkg::linemotorMsgBack>("right_knee_state", 1, boost::bind(&LineMotorCmdCallback, _1, linemotorID));
    // 踝关节发布与订阅
    ros::Publisher ankle_motor_ll_cmd_pub = nh.advertise<line_motor_comm_pkg::ankleMotorMsgCmd>(ankle_motor_name[0]+"_cmd", 1);
	ros::Publisher ankle_motor_lr_cmd_pub = nh.advertise<line_motor_comm_pkg::ankleMotorMsgCmd>(ankle_motor_name[1]+"_cmd", 1);
	ros::Publisher ankle_motor_rl_cmd_pub = nh.advertise<line_motor_comm_pkg::ankleMotorMsgCmd>(ankle_motor_name[2]+"_cmd", 1);
	ros::Publisher ankle_motor_rr_cmd_pub = nh.advertise<line_motor_comm_pkg::ankleMotorMsgCmd>(ankle_motor_name[3]+"_cmd", 1);
	int anklemotorID = 0;
	ros::Subscriber ankle_motor_ll_state_sub = nh.subscribe<line_motor_comm_pkg::ankleMotorMsgBack>(ankle_motor_name[0]+"_state", 1, boost::bind(&AnkleMotorPubCallback, _1, anklemotorID));
	anklemotorID = 1;
	ros::Subscriber ankle_motor_lr_state_sub = nh.subscribe<line_motor_comm_pkg::ankleMotorMsgBack>(ankle_motor_name[1]+"_state", 1, boost::bind(&AnkleMotorPubCallback, _1, anklemotorID));
	anklemotorID = 2;
	ros::Subscriber ankle_motor_rl_state_sub = nh.subscribe<line_motor_comm_pkg::ankleMotorMsgBack>(ankle_motor_name[2]+"_state", 1, boost::bind(&AnkleMotorPubCallback, _1, anklemotorID));
	anklemotorID = 3;
	ros::Subscriber ankle_motor_rr_state_sub = nh.subscribe<line_motor_comm_pkg::ankleMotorMsgBack>(ankle_motor_name[3]+"_state", 1, boost::bind(&AnkleMotorPubCallback, _1, anklemotorID));
    // 与强化学习节点通信的
    // 创建IMU消息订阅者
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu_data", 10, imu_feedback);
    // 髋关节电机命名
    JointMotor hipmotorL_0(0, MotorType::B1, nh, hip_motor_name_[0]);
    JointMotor hipmotorL_2(2, MotorType::B1, nh, hip_motor_name_[1]);
    JointMotor hipmotorR_0(0, MotorType::B1, nh, hip_motor_name_[2]);
    JointMotor hipmotorR_2(2, MotorType::B1, nh, hip_motor_name_[3]);

    while(ros::ok())
    {
        ros::spinOnce();
         // 开始运行
        switch(robot_state)
        {
            case ROBOT_START_MODE: // 电机复位
            {
                hipmotorL_0.BrakMode(1);
		        hipmotorL_2.BrakMode(1);
		        hipmotorR_0.BrakMode(2);
		        hipmotorR_2.BrakMode(2);

                AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
		        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
                AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
		        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
                ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                if(hipmotorL_0.PosBias() <= 0.1 && hipmotorL_2.PosBias() <= 0.1 && hipmotorR_0.PosBias() <= 0.1 && hipmotorR_2.PosBias() <= 0.1)
                {
                    hipmotorL_0.motor_init_ret.q = hipmotorL_0.motor_ret.q;
                    hipmotorL_2.motor_init_ret.q = hipmotorL_2.motor_ret.q;
                    hipmotorR_0.motor_init_ret.q = hipmotorR_0.motor_ret.q;
                    hipmotorR_2.motor_init_ret.q = hipmotorR_2.motor_ret.q; // 保存初始值

                    robot_state = ROBOT_INIT_MODE;
                    motor_ref_state = MOTOR_RESET_STATE;
                }
            break;
            }
            case ROBOT_CALIBRATION_MODE:
            {
                AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, ZEROS_SETTING_STEP, 0, 0, 0, 0, 0, 5);
                AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, ZEROS_SETTING_STEP, 0, 0, 0, 0, 0, 5);
                AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, ZEROS_SETTING_STEP, 0, 0, 0, 0, 0, 5);
                AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, ZEROS_SETTING_STEP, 0, 0, 0, 0, 0, 5);
                ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);
            break;
            }
            case ROBOT_INIT_MODE: // 到给定的第一个点
            {
                // 髋关节到第一个点
                hipmotorL_0.MixedMode(0, 0, StartPos_motorL_0, 0.5, 0.5, 1);
                hipmotorL_2.MixedMode(0, 0, StartPos_motorL_2, 0.5, 0.5, 1);
                hipmotorR_0.MixedMode(0, 0, StartPos_motorR_0, 0.5, 0.5, 1);
                hipmotorR_2.MixedMode(0, 0, StartPos_motorR_2, 0.5, 0.5, 1);
                // 踝关节到第一个点
                AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
		        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
                AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
		        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, POS_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
                ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);
                // 膝关节到第一个点
                //linemotor_left_cmd_msg.
            break;
            }
            case ROBOT_RL_MODE:

            break;
        }
    }

    return 0;
}

void LineMotorCmdCallback(const line_motor_comm_pkg::linemotorMsgBack::ConstPtr& msg, int linemotorID) {
	switch(linemotorID) {
		case 0: {
			RTPos_left = msg->x;
			std::cout << "RTPOS_LEFT: " << RTPos_left << std::endl;
			break;
		}
		case 1: {
			RTPos_right = msg->x;
			break;
		}
		default:
			break;
	}
}

void AnkleMotorPubCallback(const line_motor_comm_pkg::ankleMotorMsgBack::ConstPtr& msg, int anklemotorID)
{
	switch(anklemotorID)
	{
		case 0:
		{
			current_pos[0] = msg->pos / (2.0*PI) * 360.0;
			current_spd[0] = msg->spd * 60 / (2*PI); // r/min
			current_tau[0] = msg->tau;
			std::cout<<"当前左腿左侧电机位置值为："<<current_pos[0]<<std::endl;
			std::cout<<"当前左腿左侧电机速度值为："<<current_spd[0]<<std::endl;
			break;
		}
		case 1:
		{
			current_pos[1] = msg->pos / (2.0*PI) * 360.0;
			current_spd[1] = msg->spd * 60 / (2*PI);
			current_tau[1] = msg->tau;
			std::cout<<"当前左腿右侧电机位置值为："<<current_pos[0]<<std::endl;
			std::cout<<"当前左腿右侧电机速度值为："<<current_spd[1]<<std::endl;
			break;
		}
		case 2:
		{
			current_pos[2] = msg->pos / (2.0*PI) * 360.0;
			current_spd[2] = msg->spd * 60 / (2*PI);
			current_tau[2] = msg->tau;
			std::cout<<"当前右腿左侧电机速度值为："<<current_spd[2]<<std::endl;
			break;
		}
		case 3:
		{
			current_pos[3] = msg->pos / (2.0*PI) * 360.0; // 角度制
			current_spd[3] = msg->spd * 60 / (2*PI);
			current_tau[3] = msg->tau;
			std::cout<<"当前右腿右侧电机速度值为："<<current_spd[3]<<std::endl;
			break;
		}
	}
}

void AnkleMotorCMD(int motor_num, int motor_state, float kp, float kd, float pos, float spd, float tau, float limit_current)
{
	switch(motor_num)
	{
		case ANKLE_LL_MOTOR_NUM: // 左腿左侧电机
			ankle_motor_ll_msg.motor_state = motor_state;
			ankle_motor_ll_msg.kp = kp;
			ankle_motor_ll_msg.kd = kd;
			ankle_motor_ll_msg.pos = pos;
			ankle_motor_ll_msg.spd = spd;
			ankle_motor_ll_msg.tau = tau;
			ankle_motor_ll_msg.limit_current = limit_current;
		break;
		case ANKLE_LR_MOTOR_NUM: // 左腿右侧电机
			ankle_motor_lr_msg.motor_state = motor_state;
			ankle_motor_lr_msg.kp = kp;
			ankle_motor_lr_msg.kd = kd;
			ankle_motor_lr_msg.pos = pos;
			ankle_motor_lr_msg.spd = spd;
			ankle_motor_lr_msg.tau = tau;
			ankle_motor_lr_msg.limit_current = limit_current;
		break;
		case ANKLE_RL_MOTOR_NUM: // 右腿左侧电机
			ankle_motor_rl_msg.motor_state = motor_state;
			ankle_motor_rl_msg.kp = kp;
			ankle_motor_rl_msg.kd = kd;
			ankle_motor_rl_msg.pos = pos;
			ankle_motor_rl_msg.spd = spd;
			ankle_motor_rl_msg.tau = tau;
			ankle_motor_rl_msg.limit_current = limit_current;
		break;
		case ANKLE_RR_MOTOR_NUM: // 右腿右侧电机
			ankle_motor_rr_msg.motor_state = motor_state;
			ankle_motor_rr_msg.kp = kp;
			ankle_motor_rr_msg.kd = kd;
			ankle_motor_rr_msg.pos = pos;
			ankle_motor_rr_msg.spd = spd;
			ankle_motor_rr_msg.tau = tau;
			ankle_motor_rr_msg.limit_current = limit_current;
		break;
	}
}

void imu_feedback(sensor_msgs::Imu imu_data)
{
	if(imu_data.orientation_covariance[0] < 0)
	{
		return;
	}
	else
	{
		tf::Quaternion quaternion(
			imu_data.orientation.x, \
			imu_data.orientation.y, \
			imu_data.orientation.z, \
			imu_data.orientation.w \
		);

		tf::Matrix3x3(quaternion).getRPY(angular[0],angular[1],angular[2]);

		angular_vel[0] = imu_data.angular_velocity.x;
		angular_vel[1] = imu_data.angular_velocity.y;
		angular_vel[2] = imu_data.angular_velocity.z;

		acceleration[0] = imu_data.linear_acceleration.x;
		acceleration[1] = imu_data.linear_acceleration.y;
		acceleration[2] = imu_data.linear_acceleration.z;
	}
}

void allMotorsMsgBack_Pub(JointMotor hip_motor_l_pitch, JointMotor hip_motor_l_rotate, JointMotor hip_motor_r_pitch, JointMotor hip_motor_r_rotate)
{
    // 左腿左侧踝关节电机
    all_motors_state.ankle_motor_ll_back.tau = current_tau[0];
    all_motors_state.ankle_motor_ll_back.spd = current_spd[0];
    all_motors_state.ankle_motor_ll_back.pos = current_pos[0];
     // 左腿右侧踝关节电机
    all_motors_state.ankle_motor_lr_back.tau = current_tau[1];
    all_motors_state.ankle_motor_lr_back.spd = current_spd[1];
    all_motors_state.ankle_motor_lr_back.pos = current_pos[1];
     // 右腿左侧踝关节电机
    all_motors_state.ankle_motor_rl_back.tau = current_tau[2];
    all_motors_state.ankle_motor_rl_back.spd = current_spd[2];
    all_motors_state.ankle_motor_rl_back.pos = current_pos[2];
     // 右腿右侧踝关节电机
    all_motors_state.ankle_motor_rr_back.tau = current_tau[3];
    all_motors_state.ankle_motor_rr_back.spd = current_spd[3];
    all_motors_state.ankle_motor_rr_back.pos = current_pos[3];
    // 髋关节电机
    all_motors_state.hip_motor_r_pitch_back.id = hip_motor_l_pitch.motor_ret.motor_id;
    all_motors_state.hip_motor_r_pitch_back.tau = hip_motor_l_pitch.motor_ret.tau;
    all_motors_state.hip_motor_r_pitch_back.q = hip_motor_l_pitch.motor_ret.q;
    all_motors_state.hip_motor_r_pitch_back.dq = hip_motor_l_pitch.motor_ret.dq;

    all_motors_state.hip_motor_r_pitch_back.id = hip_motor_r_pitch.motor_ret.motor_id;
    all_motors_state.hip_motor_r_pitch_back.tau = hip_motor_r_pitch.motor_ret.tau;
    all_motors_state.hip_motor_r_pitch_back.q = hip_motor_r_pitch.motor_ret.q;
    all_motors_state.hip_motor_r_pitch_back.dq = hip_motor_r_pitch.motor_ret.dq;

    all_motors_state.hip_motor_r_rotate_back.id = hip_motor_l_rotate.motor_ret.motor_id;
    all_motors_state.hip_motor_r_rotate_back.tau = hip_motor_l_rotate.motor_ret.tau;
    all_motors_state.hip_motor_r_rotate_back.q = hip_motor_l_rotate.motor_ret.q;
    all_motors_state.hip_motor_r_rotate_back.dq = hip_motor_l_rotate.motor_ret.dq;

    all_motors_state.hip_motor_r_rotate_back.id = hip_motor_r_rotate.motor_ret.motor_id;
    all_motors_state.hip_motor_r_rotate_back.tau = hip_motor_r_rotate.motor_ret.tau;
    all_motors_state.hip_motor_r_rotate_back.q = hip_motor_r_rotate.motor_ret.q;
    all_motors_state.hip_motor_r_rotate_back.dq = hip_motor_r_rotate.motor_ret.dq;
    //膝关节电机
    all_motors_state.line_motor_l_back.x = RTPos_left;
    all_motors_state.line_motor_r_back.x = RTPos_right;
    // 发布到强化学习节点

}

void allMotorsMsgCmd_Sub()
{
    rl_motor_cmd.ankle_motor[0] = all_motors_cmd.ankle_motor_ll_cmd.pos;
    rl_motor_cmd.ankle_motor[1] = all_motors_cmd.ankle_motor_lr_cmd.pos;
    rl_motor_cmd.ankle_motor[2] = all_motors_cmd.ankle_motor_rl_cmd.pos;
    rl_motor_cmd.ankle_motor[3] = all_motors_cmd.ankle_motor_rr_cmd.pos; 

    rl_motor_cmd.hip_motor[0] = all_motors_cmd.hip_motor_l_pitch_cmd.q;
    rl_motor_cmd.hip_motor[1] = all_motors_cmd.hip_motor_l_rotate_cmd.q;
    rl_motor_cmd.hip_motor[2] = all_motors_cmd.hip_motor_r_pitch_cmd.q;
    rl_motor_cmd.hip_motor[3] = all_motors_cmd.hip_motor_r_rotate_cmd.q;

    rl_motor_cmd.knee_motor[0] = all_motors_cmd.line_motor_l_cmd.x; 
    rl_motor_cmd.knee_motor[1] = all_motors_cmd.line_motor_r_cmd.x; 
}