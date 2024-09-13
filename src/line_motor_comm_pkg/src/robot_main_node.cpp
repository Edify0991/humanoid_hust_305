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
// std::vector<std::string> hip_motor_name_ = { 
// 								"hip_lb",
//                                 "hip_ll",  
//                                 "hip_rb",
//                                 "hip_rr", 
//                                 };
std::vector<std::string> hip_motor_name_ = { 
                                // "imu",
                                "hip_left_roll",
								"hip_left_pitch",
                                "hip_right_roll",
                                "hip_right_pitch",
                                };
// // 踝关节电机命名
// std::vector<std::string> ankle_motor_name = {
//                                 "ankle_motor_ll",
//                                 "ankle_motor_lr",
//                                 "ankle_motor_rl",
//                                 "ankle_motor_rr"
//                                 };
// 踝关节电机命名
std::vector<std::string> ankle_motor_name = {
                                "ankle_motor_ld",
                                "ankle_motor_lu",
                                "ankle_motor_rd",
                                "ankle_motor_ru"
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
float current_pos[4] = {0.0f};
float current_spd[4] = {0.0f};
float current_tau[4] = {0.0f};
// 膝关节电机返回值
volatile uint16_t status_word = 0x00;
volatile uint8_t go_zero_status_1 = 0;
volatile uint8_t go_zero_status_2 = 0;
volatile uint8_t drv_ready = 0;
float RTPos_left;//左腿绝对位置,1mm对应50编码器值
float RTPos_right;//右腿绝对位置
// 髋关节电机数据
// 定义宏变量isrl，表示是否为强化学习模式
#define isrl 0

#if isrl == 1
// 强化学习离线轨迹的第一行电机角度
const float StartPos_motorL_0 = 0.26889;		// 0.288463
const float StartPos_motorL_2 = 0.529233;		// 0.402892
const float StartPos_motorR_0 = 0.32637;		// 0.341737
const float StartPos_motorR_2 = 0.122134;		// 0.257421
#else
// 开环步态初始角度
const float StartPos_motorL_0 = 0.0693481;		// 0.288463
const float StartPos_motorL_2 = 0.608191;		// 0.402892
const float StartPos_motorR_0 = 0.339611;		// 0.341737
const float StartPos_motorR_2 = 0.0553988;		// 0.257421
#endif

// 姿态传感器返回值
volatile double angular_vel[3];
volatile double acceleration[3];
double angular[3];
// 机器人状态
RL_MotorsCmd rl_motor_cmd;
enum ROBOT_STATE robot_state = ROBOT_START_MODE;
// enum ROBOT_STATE robot_state = ROBOT_RL_MODE;
// 电机状态
enum MOTOR_STATE motor_ref_state = MOTOR_INIT_STATE;
float kp_hip = 0.1;
float kd_hip = 0.1;
float kp_ankle = 50; // 0～500
float kd_ankle = 1; // 0～5

const int number_begin = 1080;
const int number_period = 960;
const int pause_time = 0;

#define hip_roll_target 0.094//弧度制，最早能成功站立的一次是0.07,发改委来的时候是0.094

float left_hip_begin[number_begin] = {0};
float right_hip_begin[number_begin] = {0};
float kneel_begin[number_begin] = {0};
float kneer_begin[number_begin] = {0};
float anklel_begin[number_begin] = {0};
float ankler_begin[number_begin] = {0};

float left_hip_period[number_period] = {0};
float right_hip_period[number_period] = {0};
float kneel_period[number_period] = {0};
float kneer_period[number_period] = {0};
float anklel_period[number_period] = {0};
float ankler_period[number_period] = {0};
float to_left_factor = 1.0;
float to_right_factor = 1.0;

/* 本文件中的函数 */
void imu_feedback(sensor_msgs::Imu imu_data);

void LineMotorCmdCallback(const line_motor_comm_pkg::linemotorMsgBack::ConstPtr& msg, int linemotorID); // 直线电机回调函数，读取直线电机状态
void AnkleMotorPubCallback(const line_motor_comm_pkg::ankleMotorMsgBack::ConstPtr& msg, int anklemotorID); // 髋关节旋转电机回调函数，读取旋转电机状体
void allMotorsMsgCmd_Sub(const line_motor_comm_pkg::allMotorsMsgCmd::ConstPtr& msg);
void allMotorsMsgBack_Pub(JointMotor &hip_motor_l_pitch, JointMotor &hip_motor_l_rotate, JointMotor &hip_motor_r_pitch, JointMotor &hip_motor_r_rotate);
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
    // 与强化学习节点通信的回调函数
    ros::Subscriber all_motors_cmd_sub = nh.subscribe<line_motor_comm_pkg::allMotorsMsgCmd>("rl_to_motor", 1, allMotorsMsgCmd_Sub);
    ros::Publisher all_motors_state_pub = nh.advertise<line_motor_comm_pkg::allMotorsMsgBack>("motor_to_rl", 1);
    // 创建IMU消息订阅者
	// ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu_data", 10, imu_feedback);
    // 髋关节电机命名
    JointMotor hipmotorL_0(0, MotorType::B1, nh, hip_motor_name_[0]);
    JointMotor hipmotorL_2(2, MotorType::B1, nh, hip_motor_name_[1]);
    JointMotor hipmotorR_0(0, MotorType::B1, nh, hip_motor_name_[2]);
    JointMotor hipmotorR_2(2, MotorType::B1, nh, hip_motor_name_[3]);

	std::string data_begin_filename = "/home/humanoid/Code/unitree_actuator_sdk/python/data_begin.csv";
	std::string data_period_filename = "/home/humanoid/Code/unitree_actuator_sdk/python/data_period.csv";

    bool ret_readdata_begin = util::readdata(left_hip_begin, kneel_begin, anklel_begin, right_hip_begin, kneer_begin, ankler_begin, data_begin_filename);
	if (ret_readdata_begin == false) {
		exit(-1);
	}

	bool ret_readdata_period = util::readdata(left_hip_period, kneel_period, anklel_period, right_hip_period, kneer_period, ankler_period, data_period_filename);
	if (ret_readdata_period == false) {
		exit(-1);
	}

    int unit_time=480;	// 循环步态中的半个周期
	// 定义三个关节在单个运动过程中前后摆位置数组
	// 髋关节侧摆（左右同步）
	float hip_roll[2 * unit_time];//这个+1用来在循环取余时对齐位置

	// 计算侧摆
	float t = 0;
	float delta = M_PI / unit_time;
	for (int i = 0 ; i < 2 * unit_time ; i++) {
		hip_roll[i] = hip_roll_target * std::sin(t);
		t = t + delta;
	}

    while(ros::ok())
    {
        static uint32_t count = 0;
        float l_pitch_hip_motor=0.0f;
        float l_rotate_hip_motor=0.0f;
        float r_pitch_hip_motor=0.0f;
        float r_rotate_hip_motor=0.0f;
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

                AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, 0, 0, 0, 2.4, 0, 5);
		        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, 0, 0, 0, 2.4, 0, 5);
                AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, 0, 0, 0, 2.4, 0, 5);
		        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, 0, 0, 0, 2.4, 0, 5);
                ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                // std::cout<<"L_0: "<<hipmotorL_0.motor_ret.q<<std::endl;
                // std::cout<<"L_2: "<<hipmotorL_2.motor_ret.q<<std::endl;
                // std::cout<<"R_0: "<<hipmotorR_0.motor_ret.q<<std::endl;
                // std::cout<<"R_2: "<<hipmotorR_2.motor_ret.q<<std::endl;

                hipmotorL_0.motor_init_ret.q = hipmotorL_0.motor_ret.q;
                hipmotorL_2.motor_init_ret.q = hipmotorL_2.motor_ret.q;
                hipmotorR_0.motor_init_ret.q = hipmotorR_0.motor_ret.q;
                hipmotorR_2.motor_init_ret.q = hipmotorR_2.motor_ret.q; // 保存初始值

                std::cout<<"L_0: "<<hipmotorL_0.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;
                std::cout<<"L_2: "<<hipmotorL_2.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;
                std::cout<<"R_0: "<<hipmotorR_0.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;
                std::cout<<"R_2: "<<hipmotorR_2.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;

                // rl_motor_cmd.hip_motor_pos[0] = hipmotorL_0.motor_init_ret.q / queryGearRatio(MotorType::B1);
                // rl_motor_cmd.hip_motor_pos[1] = hipmotorL_2.motor_init_ret.q / queryGearRatio(MotorType::B1);
                // rl_motor_cmd.hip_motor_pos[2] = hipmotorR_0.motor_init_ret.q / queryGearRatio(MotorType::B1);
                // rl_motor_cmd.hip_motor_pos[3] = hipmotorR_2.motor_init_ret.q / queryGearRatio(MotorType::B1);
                rl_motor_cmd.hip_motor_pos[0] = 0.0f;
                rl_motor_cmd.hip_motor_pos[1] = 0.0f;
                rl_motor_cmd.hip_motor_pos[2] = 0.0f;
                rl_motor_cmd.hip_motor_pos[3] = 0.0f;
                count++;
                if(count >= 100000)
                {
                    // robot_state = ROBOT_RL_MODE;    // 原始版本，开环测试时这里换成另一个state
                    robot_state = ROBOT_INIT_MODE;      // 髋关节与膝关节需要到达指定位置
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
                robot_state = ROBOT_NOTHING_MODE;
                break;
            }
            case ROBOT_INIT_MODE: // 到给定的第一个点
            {
                ros::Rate init_loop_rate(20);
                // 髋关节到第一个点
                // 因为电机启动时初始角度与轨迹起始点初始角度相差可能较大，B1电机需要将该过程细分
                int InitPoint_num = 200;
                for(int i = 0 ; i <= InitPoint_num ; i++)
                {
                    ros::spinOnce();
                    hipmotorL_0.MixedMode(0, 0, hipmotorL_0.motor_init_ret.q / queryGearRatio(MotorType::B1) + (StartPos_motorL_0 - hipmotorL_0.motor_init_ret.q / queryGearRatio(MotorType::B1)) * i / (float)InitPoint_num, 3, 1, 1);
                    hipmotorL_2.MixedMode(0, 0, hipmotorL_2.motor_init_ret.q / queryGearRatio(MotorType::B1) + (StartPos_motorL_2 - hipmotorL_2.motor_init_ret.q / queryGearRatio(MotorType::B1)) * i / (float)InitPoint_num, 3, 1, 1);
                    hipmotorR_0.MixedMode(0, 0, hipmotorR_0.motor_init_ret.q / queryGearRatio(MotorType::B1) + (StartPos_motorR_0 - hipmotorR_0.motor_init_ret.q / queryGearRatio(MotorType::B1)) * i / (float)InitPoint_num, 3, 1, 1);
                    hipmotorR_2.MixedMode(0, 0, hipmotorR_2.motor_init_ret.q / queryGearRatio(MotorType::B1) + (StartPos_motorR_2 - hipmotorR_2.motor_init_ret.q / queryGearRatio(MotorType::B1)) * i / (float)InitPoint_num, 3, 1, 1);
                    init_loop_rate.sleep();
                }

                // hipmotorL_0.MixedMode(0, 0, StartPos_motorL_0, 0.5, 0.5, 1);
                // hipmotorL_2.MixedMode(0, 0, StartPos_motorL_2, 0.5, 0.5, 1);
                // hipmotorR_0.MixedMode(0, 0, StartPos_motorR_0, 0.5, 0.5, 1);
                // hipmotorR_2.MixedMode(0, 0, StartPos_motorR_2, 0.5, 0.5, 1);

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
                // linemotor_left_cmd_msg.x
                // linemotor_left_cmd_pub

                robot_state = ROBOT_OPENLOOP_MODE;  // 开始开环步态
                break;
            }
            case ROBOT_RL_MODE:
                // 订阅强化学习数据并发布到电机
                l_rotate_hip_motor = rl_motor_cmd.hip_motor_pos[0]+hipmotorL_0.motor_init_ret.q / queryGearRatio(MotorType::B1);
                l_pitch_hip_motor = rl_motor_cmd.hip_motor_pos[1]+hipmotorL_2.motor_init_ret.q / queryGearRatio(MotorType::B1);
                r_rotate_hip_motor = rl_motor_cmd.hip_motor_pos[2]+hipmotorR_0.motor_init_ret.q / queryGearRatio(MotorType::B1);
                r_pitch_hip_motor = rl_motor_cmd.hip_motor_pos[3]+hipmotorR_2.motor_init_ret.q / queryGearRatio(MotorType::B1);

                hipmotorL_0.MixedMode(0, rl_motor_cmd.hip_motor_spd[0], l_rotate_hip_motor, kp_hip, kd_hip, 1);
                hipmotorL_2.MixedMode(0, rl_motor_cmd.hip_motor_spd[1], l_pitch_hip_motor, kp_hip, kd_hip, 1);
                hipmotorR_0.MixedMode(0, rl_motor_cmd.hip_motor_spd[2], r_rotate_hip_motor, kp_hip, kd_hip, 1);
                hipmotorR_2.MixedMode(0, rl_motor_cmd.hip_motor_spd[3], r_pitch_hip_motor, kp_hip, kd_hip, 1);

                AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_tau2pos[0], rl_motor_cmd.ankle_motor_spd[0], 0, 15);
		        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_tau2pos[1], rl_motor_cmd.ankle_motor_spd[1], 0, 15);
                AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_tau2pos[2], rl_motor_cmd.ankle_motor_spd[2], 0, 15);
		        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_tau2pos[3], rl_motor_cmd.ankle_motor_spd[3], 0, 15);
                ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                // AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_pos[0], rl_motor_cmd.ankle_motor_spd[0], 0, 15);
		        // AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_pos[1], rl_motor_cmd.ankle_motor_spd[1], 0, 15);
                // AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_pos[2], rl_motor_cmd.ankle_motor_spd[2], 0, 15);
		        // AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, rl_motor_cmd.ankle_motor_pos[3], rl_motor_cmd.ankle_motor_spd[3], 0, 15);
                // ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                // ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                // ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                // ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                linemotor_left_cmd_msg.x = rl_motor_cmd.knee_motor_pos[0];
                linemotor_left_cmd_msg.dx = rl_motor_cmd.knee_motor_spd[0];
                linemotor_right_cmd_msg.x = rl_motor_cmd.knee_motor_pos[1];
                linemotor_right_cmd_msg.dx = rl_motor_cmd.knee_motor_spd[1];
                linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
                linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

                // 发布到强化学习
                allMotorsMsgBack_Pub(hipmotorL_2, hipmotorL_0, hipmotorR_2, hipmotorR_0);
                all_motors_state_pub.publish(all_motors_state);
            break;
            case ROBOT_NOTHING_MODE:

            break;

            case ROBOT_OPENLOOP_MODE: {
                int k_ankle_hip_roll=1;//踝关节侧摆差动大小与髋侧摆的关系
	            int TempCnt1 = 0;
                ros::Rate start_loop_rate(200);
                for (int i = 0 ; i < 600 ; i++) {
                    AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_begin[i], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_begin[i], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_begin[i], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_begin[i], 20, 0, 30);
                    ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                    ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                    ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                    ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                    l_pitch_hip_motor = left_hip_begin[i] + StartPos_motorL_2;
                    r_pitch_hip_motor = -right_hip_begin[i] + StartPos_motorR_2;

                    hipmotorL_2.MixedMode(0, 0, l_pitch_hip_motor, 3, 1, 1);
                    hipmotorR_2.MixedMode(0, 0, r_pitch_hip_motor, 3, 1, 1);

                    if (i % 6 == 0) {
                        TempCnt1 += 6;
                    }

                    linemotor_left_cmd_msg.x = kneel_begin[TempCnt1];
                    linemotor_left_cmd_msg.dx = 2000;
                    linemotor_right_cmd_msg.x = kneer_begin[TempCnt1];
                    linemotor_right_cmd_msg.dx = 2000;

                    linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
                    linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

                    l_rotate_hip_motor = StartPos_motorL_0;
                    r_rotate_hip_motor = StartPos_motorR_0;

                    hipmotorL_0.MixedMode(0, 0, l_rotate_hip_motor, 0.5, 1, 1);
                    hipmotorR_0.MixedMode(0, 0, r_rotate_hip_motor, 0.5, 1, 1);

                    // 发布到强化学习
                    allMotorsMsgBack_Pub(hipmotorL_2, hipmotorL_0, hipmotorR_2, hipmotorR_0);
                    all_motors_state_pub.publish(all_motors_state);

                    ros::spinOnce();
                    start_loop_rate.sleep();
                }

                int CntPitch = 600;       // 往前迈步时的数组元素位置
                // 由直立向左侧摆
                for(int i = 0 ; i < 0.5 * unit_time ; i++) {      //每次走0.5个unit_time
                    l_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorL_0;
                    r_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorR_0;

                    hipmotorL_0.MixedMode(0, 0, l_rotate_hip_motor, 6, 10, 1);
                    hipmotorR_0.MixedMode(0, 0, r_rotate_hip_motor, 6, 10, 1);

                    AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                    ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                    ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                    ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);
                    ros::spinOnce();
                    start_loop_rate.sleep();
                }

                int CntRoll = 0.5 * unit_time;
                int TempCnt2 = CntPitch;
                float V_LeftHip2, V_RightHip2, V_LeftAnkle, V_RightAnkle;
                float Kp = 1;
                float Kd = 10;
                // 右腿迈半步
                for(int i = CntPitch ; i < number_begin ; i++) {  
                    if (i < number_begin - 1) {
                        V_LeftHip2 = (left_hip_begin[i + 1] - left_hip_begin[i]) / 0.03;    // 一个周期大概0.03s，之后增快后需要调整
                        V_RightHip2 = (right_hip_begin[i + 1] - right_hip_begin[i]) / 0.03;
                        V_LeftAnkle = (anklel_begin[i + 1] - anklel_begin[i]) / 0.03;
                        V_RightAnkle = (ankler_begin[i + 1] - ankler_begin[i]) / 0.03;
                    }
                    else {
                        V_LeftHip2 = 0;
                        V_RightHip2= 0;	
                        V_LeftAnkle = 0;	
                        V_RightAnkle = 0;
                    }
                    AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                    ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                    ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                    ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                    ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                    l_pitch_hip_motor = left_hip_begin[i] + StartPos_motorL_2;
                    r_pitch_hip_motor = -right_hip_begin[i] + StartPos_motorR_2;

                    hipmotorL_2.MixedMode(0, V_LeftHip2, l_pitch_hip_motor, 3, 30, 1);
                    hipmotorR_2.MixedMode(0, -V_RightHip2, r_pitch_hip_motor, 3, 30, 1);

                    if (i % 6 == 0) {	
                        TempCnt2 += 6;
                        if(TempCnt2 == number_begin) {
                            TempCnt2--;
                        }
                    }
                    
                    linemotor_left_cmd_msg.x = kneel_begin[TempCnt2];
                    linemotor_left_cmd_msg.dx = 2000;
                    linemotor_right_cmd_msg.x = kneer_begin[TempCnt2];
                    linemotor_right_cmd_msg.dx = 2000;

                    linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
                    linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

                    ros::spinOnce();
                    start_loop_rate.sleep();
                }

                for(int i = 0.5 * unit_time ; i < unit_time ; i++) {      // 侧摆回正
                    l_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorL_0;
                    r_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorR_0;

                    hipmotorL_0.MixedMode(0, 0, l_rotate_hip_motor, 6, 10, 1);
                    hipmotorR_0.MixedMode(0, 0, r_rotate_hip_motor, 6, 10, 1);

                    AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                    ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                    ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                    ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                    ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);
                    ros::spinOnce();
                    start_loop_rate.sleep();
                }

                // 开始周期步态
                int loop_num;
                std::cout << "Please input the steps you want to execute: ";
                std::cin >> loop_num;
                for (int j = 0 ; j < loop_num ; j++) {
                    ros::Rate inner_loop_rate(200);
                    ros::Rate roll_loop_rate(200);
                    // 由直立侧摆向右
                    for(int i = unit_time ; i < 1.5 * unit_time ; i++) {   
                        l_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorL_0;
                        r_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorR_0;

                        hipmotorL_0.MixedMode(0, 0, l_rotate_hip_motor, 6, 10, 1);
                        hipmotorR_0.MixedMode(0, 0, r_rotate_hip_motor, 6, 10, 1);

                        AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_begin[number_begin - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_begin[number_begin - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_begin[number_begin - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_begin[number_begin - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                        ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                        ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                        ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);  

                        ros::spinOnce();
                        roll_loop_rate.sleep();
                    }
                    sleep(pause_time);
                    int CntRoll = 1.5 * unit_time - 1;
                    int TempCnt2 = 0;
                    float Kp = 1;
                    float Kd = 10;

                    // 迈左腿
                    for(int i = 0 ; i < number_period / 2 ; i++) {
                        // 粗略计算期望速度
                        if (i < number_period / 2 - 1) {
                            V_LeftHip2 = (left_hip_period[i + 1] - left_hip_period[i]) / 0.03;    // 一个周期大概0.03s，之后增快后需要调整
                            V_RightHip2 = (right_hip_period[i + 1] - right_hip_period[i]) / 0.03;
                            V_LeftAnkle = (anklel_period[i + 1] - anklel_period[i]) / 0.03;
                            V_RightAnkle = (ankler_period[i + 1] - ankler_period[i]) / 0.03;
                        }
                        else {
                            V_LeftHip2 = 0;
                            V_RightHip2= 0;	
                            V_LeftAnkle = 0;	
                            V_RightAnkle = 0;
                        }

                        AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_period[i] + to_right_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_period[i] + to_right_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_period[i] + to_right_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_period[i] + to_right_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                        ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                        ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                        ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                        l_pitch_hip_motor = left_hip_period[i] + StartPos_motorL_2;
                        r_pitch_hip_motor = -right_hip_period[i] + StartPos_motorR_2;

                        hipmotorL_2.MixedMode(0, V_LeftHip2, l_pitch_hip_motor, 3, 30, 1);
                        hipmotorR_2.MixedMode(0, -V_RightHip2, r_pitch_hip_motor, 3, 30, 1);

                        if (i % 6 == 0) {	
                            TempCnt2 += 6;
                            if(TempCnt2 == number_period / 2) {
                                TempCnt2--;
                            }
                        }
                        linemotor_left_cmd_msg.x = kneel_period[TempCnt2];
                        linemotor_left_cmd_msg.dx = 2000;
                        linemotor_right_cmd_msg.x = kneer_period[TempCnt2];
                        linemotor_right_cmd_msg.dx = 2000;

                        linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
                        linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);
                        // std::cout << "迈左腿linemotor_right_cmd_msg\t" << linemotor_left_cmd_msg.x << std::endl;

                        ros::spinOnce();
                        inner_loop_rate.sleep();
                    }
                    sleep(pause_time);

                    // 由右侧向左侧摆至中央
                    for(int i = 1.5 * unit_time ; i < 2 * unit_time ; i++) { 
                        l_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorL_0;
                        r_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorR_0;

                        hipmotorL_0.MixedMode(0, 0, l_rotate_hip_motor, 6, 10, 1);
                        hipmotorR_0.MixedMode(0, 0, r_rotate_hip_motor, 6, 10, 1);

                        AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_period[number_period / 2 - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_period[number_period / 2 - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_period[number_period / 2 - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_period[number_period / 2 - 1] + to_right_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                        ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                        ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                        ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg); 
                        
                        ros::spinOnce();
                        roll_loop_rate.sleep();
                    }

                    // 由中央向左侧摆至左侧
                    for(int i = 2 * unit_time ; i < 2.5 * unit_time ; i++) {   
                        l_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorL_0;
                        r_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorR_0;

                        hipmotorL_0.MixedMode(0, 0, l_rotate_hip_motor, 6, 10, 1);
                        hipmotorR_0.MixedMode(0, 0, r_rotate_hip_motor, 6, 10, 1);

                        AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_period[number_period / 2 - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_period[number_period / 2 - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_period[number_period / 2 - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_period[number_period / 2 - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                        ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                        ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                        ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);   

                        ros::spinOnce();
                        roll_loop_rate.sleep();
                    }
                    sleep(pause_time);
                    CntRoll = 0.5 * unit_time - 1;
                    TempCnt2 = number_period / 2;

                    // 迈右腿
                    for(int i = number_period / 2 ; i < number_period ; i++) {
                        // 粗略计算期望速度
                        if (i < number_period - 1) {
                            V_LeftHip2 = (left_hip_period[i + 1] - left_hip_period[i]) / 0.03;    // 一个周期大概0.03s，之后增快后需要调整
                            V_RightHip2 = (right_hip_period[i + 1] - right_hip_period[i]) / 0.03;
                            V_LeftAnkle = (anklel_period[i + 1] - anklel_period[i]) / 0.03;
                            V_RightAnkle = (ankler_period[i + 1] - ankler_period[i]) / 0.03;
                        }
                        else {
                            V_LeftHip2 = 0;
                            V_RightHip2= 0;	
                            V_LeftAnkle = 0;	
                            V_RightAnkle = 0;
                        }

                        AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_period[i] + to_left_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_period[i] + to_left_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_period[i] + to_left_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_period[i] + to_left_factor * k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 20, 0, 30);
                        ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                        ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                        ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                        ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg);

                        l_pitch_hip_motor = left_hip_period[i] + StartPos_motorL_2;
                        r_pitch_hip_motor = -right_hip_period[i] + StartPos_motorR_2;

                        hipmotorL_2.MixedMode(0, V_LeftHip2, l_pitch_hip_motor, 3, 30, 1);
                        hipmotorR_2.MixedMode(0, -V_RightHip2, r_pitch_hip_motor, 3, 30, 1);
                        
                        if (i % 6 == 0) {	
                            TempCnt2 += 6;
                            if(TempCnt2 == number_period) {
                                TempCnt2--;
                            }
                        }

                        linemotor_left_cmd_msg.x = kneel_period[TempCnt2];
                        linemotor_left_cmd_msg.dx = 2000;
                        linemotor_right_cmd_msg.x = kneer_period[TempCnt2];
                        linemotor_right_cmd_msg.dx = 2000;

                        linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
                        linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);
                        // std::cout << "迈右腿linemotor_right_cmd_msg\t" << linemotor_left_cmd_msg.x << std::endl;

                        ros::spinOnce();
                        inner_loop_rate.sleep();
                        
                        // left_leg.Clear_PosCmd();
                        // left_leg.RelPos_Set(kneel_period[TempCnt2], 500);
                        // right_leg.Clear_PosCmd();
                        // right_leg.RelPos_Set(kneer_period[TempCnt2], 500);
                    }
                    sleep(pause_time);

                    // 由左侧向右侧摆至直立
                    for(int i = 0.5 * unit_time ; i < unit_time ; i++) {    
                        l_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorL_0;
                        r_rotate_hip_motor = -hip_roll[i % (2 * unit_time)] + StartPos_motorR_0;

                        hipmotorL_0.MixedMode(0, 0, l_rotate_hip_motor, 6, 10, 1);
                        hipmotorR_0.MixedMode(0, 0, r_rotate_hip_motor, 6, 10, 1);

                        AnkleMotorCMD(ANKLE_LL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -anklel_period[number_period - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_LR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, anklel_period[number_period - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RL_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, -ankler_period[number_period - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        AnkleMotorCMD(ANKLE_RR_MOTOR_NUM, MIXED_CONTROL_STEP, kp_ankle, kd_ankle, ankler_period[number_period - 1] + to_left_factor * k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 20, 0, 30);
                        ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
                        ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
                        ankle_motor_rl_cmd_pub.publish(ankle_motor_rl_msg);
                        ankle_motor_rr_cmd_pub.publish(ankle_motor_rr_msg); 
                        ros::spinOnce();
                        roll_loop_rate.sleep();
                    }
                }
                break;
            }
        }
    }

    return 0;
}

void LineMotorCmdCallback(const line_motor_comm_pkg::linemotorMsgBack::ConstPtr& msg, int linemotorID) {
	switch(linemotorID) {
		case 0: {
			RTPos_left = msg->x;
			// std::cout << "RTPOS_LEFT: " << RTPos_left << std::endl;
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
			current_pos[0] = msg->pos; // 弧度
			current_spd[0] = msg->spd; // 弧度/s
			current_tau[0] = msg->tau;
			// std::cout<<"当前左腿左侧电机位置值为："<<current_pos[0]<<std::endl;
			// std::cout<<"当前左腿左侧电机速度值为："<<current_spd[0]<<std::endl;
			break;
		}
		case 1:
		{
			current_pos[1] = msg->pos;
			current_spd[1] = msg->spd;
			current_tau[1] = msg->tau;
			// std::cout<<"当前左腿右侧电机位置值为："<<current_pos[0]<<std::endl;
			// std::cout<<"当前左腿右侧电机速度值为："<<current_spd[1]<<std::endl;
			break;
		}
		case 2:
		{
			current_pos[2] = msg->pos;
			current_spd[2] = msg->spd;
			current_tau[2] = msg->tau;
			// std::cout<<"当前右腿左侧电机速度值为："<<current_spd[2]<<std::endl;
			break;
		}
		case 3:
		{
			current_pos[3] = msg->pos; // 弧度制
			current_spd[3] = msg->spd;
			current_tau[3] = msg->tau;
			// std::cout<<"当前右腿右侧电机速度值为："<<current_spd[3]<<std::endl;
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

void allMotorsMsgBack_Pub(JointMotor &hip_motor_l_pitch, JointMotor &hip_motor_l_rotate, JointMotor &hip_motor_r_pitch, JointMotor &hip_motor_r_rotate)
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
    all_motors_state.hip_motor_l_pitch_back.id = hip_motor_l_pitch.motor_ret.motor_id;
    all_motors_state.hip_motor_l_pitch_back.tau = hip_motor_l_pitch.motor_ret.tau * queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_l_pitch_back.q = (hip_motor_l_pitch.motor_ret.q - hip_motor_l_pitch.motor_init_ret.q) / queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_l_pitch_back.dq = hip_motor_l_pitch.motor_ret.dq / queryGearRatio(MotorType::B1);

    all_motors_state.hip_motor_r_pitch_back.id = hip_motor_r_pitch.motor_ret.motor_id;
    all_motors_state.hip_motor_r_pitch_back.tau = hip_motor_r_pitch.motor_ret.tau * queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_r_pitch_back.q = (hip_motor_r_pitch.motor_ret.q - hip_motor_r_pitch.motor_init_ret.q) / queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_r_pitch_back.dq = hip_motor_r_pitch.motor_ret.dq / queryGearRatio(MotorType::B1);

    all_motors_state.hip_motor_l_rotate_back.id = hip_motor_l_rotate.motor_ret.motor_id;
    all_motors_state.hip_motor_l_rotate_back.tau = hip_motor_l_rotate.motor_ret.tau * queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_l_rotate_back.q = (hip_motor_l_rotate.motor_ret.q - hip_motor_l_rotate.motor_init_ret.q) / queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_l_rotate_back.dq = hip_motor_l_rotate.motor_ret.dq / queryGearRatio(MotorType::B1);

    all_motors_state.hip_motor_r_rotate_back.id = hip_motor_r_rotate.motor_ret.motor_id;
    all_motors_state.hip_motor_r_rotate_back.tau = hip_motor_r_rotate.motor_ret.tau * queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_r_rotate_back.q = (hip_motor_r_rotate.motor_ret.q - hip_motor_r_rotate.motor_init_ret.q) / queryGearRatio(MotorType::B1);
    all_motors_state.hip_motor_r_rotate_back.dq = hip_motor_r_rotate.motor_ret.dq / queryGearRatio(MotorType::B1);
    //膝关节电机
    all_motors_state.line_motor_l_back.x = RTPos_left;
    all_motors_state.line_motor_r_back.x = RTPos_right;
}

void allMotorsMsgCmd_Sub(const line_motor_comm_pkg::allMotorsMsgCmd::ConstPtr& msg)
{
    // 位置控制时踝关节目标位置
    rl_motor_cmd.ankle_motor_pos[0] = msg->ankle_motor_ll_cmd.pos;
    rl_motor_cmd.ankle_motor_pos[1] = msg->ankle_motor_lr_cmd.pos;
    rl_motor_cmd.ankle_motor_pos[2] = msg->ankle_motor_rl_cmd.pos;
    rl_motor_cmd.ankle_motor_pos[3] = msg->ankle_motor_rr_cmd.pos; 
    // 髋关节目标位置
    rl_motor_cmd.hip_motor_pos[0] = msg->hip_motor_l_pitch_cmd.q;
    rl_motor_cmd.hip_motor_pos[1] = msg->hip_motor_l_rotate_cmd.q;
    rl_motor_cmd.hip_motor_pos[2] = msg->hip_motor_r_pitch_cmd.q;
    rl_motor_cmd.hip_motor_pos[3] = msg->hip_motor_r_rotate_cmd.q;
    // 膝关节目标位置
    rl_motor_cmd.knee_motor_pos[0] = msg->line_motor_l_cmd.x; 
    rl_motor_cmd.knee_motor_pos[1] = msg->line_motor_r_cmd.x; 
    // 踝关节目标速度
    rl_motor_cmd.ankle_motor_spd[0] = msg->ankle_motor_ll_cmd.spd;
    rl_motor_cmd.ankle_motor_spd[1] = msg->ankle_motor_lr_cmd.spd;
    rl_motor_cmd.ankle_motor_spd[2] = msg->ankle_motor_rl_cmd.spd;
    rl_motor_cmd.ankle_motor_spd[3] = msg->ankle_motor_rr_cmd.spd; 
    // 髋关节目标速度
    rl_motor_cmd.hip_motor_spd[0] = msg->hip_motor_l_pitch_cmd.dq;
    rl_motor_cmd.hip_motor_spd[1] = msg->hip_motor_l_rotate_cmd.dq;
    rl_motor_cmd.hip_motor_spd[2] = msg->hip_motor_r_pitch_cmd.dq;
    rl_motor_cmd.hip_motor_spd[3] = msg->hip_motor_r_rotate_cmd.dq;
    // 膝关节目标速度
    rl_motor_cmd.knee_motor_spd[0] = msg->line_motor_l_cmd.dx; 
    rl_motor_cmd.knee_motor_spd[1] = msg->line_motor_r_cmd.dx;
    // 混合控制时踝关节目标位置. tff + kp(pos_target - pos_current) + kd(spd_target - spd_current) = tau
    rl_motor_cmd.ankle_motor_tau2pos[0] = (msg->ankle_motor_ll_cmd.tau - \
                                          kd_ankle * (rl_motor_cmd.ankle_motor_spd[0]-all_motors_state.ankle_motor_ll_back.spd)) / kp_ankle + \
                                          all_motors_state.ankle_motor_ll_back.pos;
    rl_motor_cmd.ankle_motor_tau2pos[1] = (msg->ankle_motor_lr_cmd.tau - \
                                          kd_ankle * (rl_motor_cmd.ankle_motor_spd[1]-all_motors_state.ankle_motor_lr_back.spd)) / kp_ankle + \
                                          all_motors_state.ankle_motor_lr_back.pos;
    rl_motor_cmd.ankle_motor_tau2pos[2] = (msg->ankle_motor_rl_cmd.tau - \
                                          kd_ankle * (rl_motor_cmd.ankle_motor_spd[2]-all_motors_state.ankle_motor_rl_back.spd)) / kp_ankle + \
                                          all_motors_state.ankle_motor_rl_back.pos;
    rl_motor_cmd.ankle_motor_tau2pos[3] = (msg->ankle_motor_rr_cmd.tau - \
                                          kd_ankle * (rl_motor_cmd.ankle_motor_spd[3]-all_motors_state.ankle_motor_rr_back.spd)) / kp_ankle + \
                                          all_motors_state.ankle_motor_rr_back.pos;
    // 混合控制时髋关节目标位置
    rl_motor_cmd.hip_motor_tau2pos[0] = (msg->hip_motor_l_pitch_cmd.tau - \
                                        kd_hip * kd_hip * (rl_motor_cmd.hip_motor_spd[0]-all_motors_state.hip_motor_l_pitch_back.dq)) / \
                                        (kp_hip * kp_hip) + \
                                        all_motors_state.hip_motor_l_pitch_back.q;
    rl_motor_cmd.hip_motor_tau2pos[1] = (msg->hip_motor_l_rotate_cmd.tau - \
                                        kd_hip * kd_hip * (rl_motor_cmd.hip_motor_spd[1]-all_motors_state.hip_motor_l_rotate_back.dq)) / \
                                        (kp_hip * kp_hip) + \
                                        all_motors_state.hip_motor_l_rotate_back.q;
    rl_motor_cmd.hip_motor_tau2pos[2] = (msg->hip_motor_r_pitch_cmd.tau - \
                                        kd_hip * kd_hip * (rl_motor_cmd.hip_motor_spd[2]-all_motors_state.hip_motor_r_pitch_back.dq)) / \
                                        (kp_hip * kp_hip) + \
                                        all_motors_state.hip_motor_r_pitch_back.q;
    rl_motor_cmd.hip_motor_tau2pos[3] = (msg->hip_motor_r_rotate_cmd.tau - \
                                        kd_hip * kd_hip * (rl_motor_cmd.hip_motor_spd[3]-all_motors_state.hip_motor_r_rotate_back.dq)) / \
                                        (kp_hip * kp_hip) + \
                                        all_motors_state.hip_motor_r_rotate_back.q;

    // std::cout <<"ankle_motor_1:\t"<<rl_motor_cmd.ankle_motor_pos[0]<<std::endl;
    // std::cout <<"ankle_motor_2:\t"<<rl_motor_cmd.ankle_motor_pos[1]<<std::endl;
    // std::cout <<"ankle_motor_3:\t"<<rl_motor_cmd.ankle_motor_pos[2]<<std::endl;
}