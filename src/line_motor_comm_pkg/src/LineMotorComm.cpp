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
#include "line_motor_comm_pkg/linemotorMsgBack.h"
#include "line_motor_comm_pkg/linemotorMsgCmd.h"

#include "ankle_motor_node.h"
#include "ankle_comm.h"
#include "line_motor_comm_pkg/ankleMotorMsgCmd.h"
#include "line_motor_comm_pkg/ankleMotorMsgBack.h"
#define hip_pitch_target 0.10//弧度制
#define hip_roll_target 0.07//弧度制，最早能成功站立的一次是0.07,发改委来的时候是0.094
#define knee_target 30//单位mm

// LineMotorComm(8.29-多线程原展示开环任意步数)

using namespace std;
std::vector<std::string> motor_name_ = { 
								// "imu",
								"hip_lb",
                                "hip_ll",  
                                "hip_rb",
                                "hip_rr", 
                                "ankle_ll",
                                "ankle_lr",
								"ankle_rr",
                                "ankle_rl",
                                };
// 踝关节电机命名
std::vector<std::string> ankle_motor_name = {
                                "ankle_motor_ll",
                                "ankle_motor_lr",
                                "ankle_motor_rl",
                                "ankle_motor_rr"
                                };

// 该节点的全局变量
volatile uint16_t status_word = 0x00;
volatile uint8_t go_zero_status_1 = 0;
volatile uint8_t go_zero_status_2 = 0;
volatile uint8_t drv_ready = 0;
volatile double angular_vel[3];
volatile double acceleration[3];
float RTPos_left;//左腿绝对位置,1mm对应50编码器值
float RTPos_right;//右腿绝对位置
const int usleep_time = 2000;

// 踝关节电机返回值
float current_pos[4] = {0};
float current_spd[4] = {0};
float current_tau[4] = {0};
// 踝关节电机控制命令通信包 
line_motor_comm_pkg::ankleMotorMsgCmd ankle_motor_ll_msg, ankle_motor_lr_msg;
// line_motor_comm_pkg::ankleMotorMsgCmd ankle_motor_rl_msg, ankle_motor_rr_msg;
// 踝关节电机指令输入函数
void AnkleMotorCMD(int motor_num, int motor_state, float kp, float kd, float pos, float spd, float tau, float limit_current);

// 绝对起点位置
// 4个A1电机
const float StartPos_motorL_0_a = 0.527665;	// 0.0559651
const float StartPos_motorL_1_a = 0.118968;		// 0.302414
const float StartPos_motorR_0_a = 0.597285;		// 0.192464
const float StartPos_motorR_1_a = 0.407012;		// 0.254962
// 4个B1电机
const float StartPos_motorL_0 = 0.26889;		// 0.288463
const float StartPos_motorL_2 = 0.529233;		// 0.402892
const float StartPos_motorR_0 = 0.32637;		// 0.341737
const float StartPos_motorR_2 = 0.122134;		// 0.257421

float motorL_0_ORI;//记录B1电机上电初始值,0是侧摆电机，2是前后摆电机
float motorL_2_ORI;
float motorR_0_ORI;
float motorR_2_ORI;
float motorL_0_ORI_a;
float motorL_1_ORI_a;
float motorR_0_ORI_a;
float motorR_1_ORI_a;


int CntPitch=0;//往前迈步时的数组元素位置
int CntRoll=0;//侧摆腿时的数组元素位置
int Pitch_Roll_waittime=0;//侧摆到位后等待的长度

const int number = 3478;
const int number_begin = 1080;
const int number_period = 960;

// float left_kuan[number] = {0};
// float right_kuan[number] = {0};
// float kneel[number] = {0};
// float kneer[number] = {0};
// float anklel[number] = {0};
// float ankler[number] = {0};
// float kneelrel[number] = {0}, kneerrel[number] = {0};

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

double angular[3];
ros::Publisher linemotor_state_pub;

void Receive_Linemotor_Fcn(int CANIndex);
void Receive_Zero_Fcn(int CANIndex);
void imu_feedback(sensor_msgs::Imu imu_data);
int8_t CanPort_Open(void);

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

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "line_motor_comm_node");
    ros::NodeHandle nh;
	ros::Rate loop_rate(200);

	string filename = "cur_data.csv";
	string data_total_filename = "/home/humanoid/Code/unitree_actuator_sdk/python/data.csv";
	string data_begin_filename = "/home/humanoid/Code/unitree_actuator_sdk/python/data_begin.csv";
	string data_period_filename = "/home/humanoid/Code/unitree_actuator_sdk/python/data_period.csv";
	//从文件中读数据
	// bool ret_readdata_total = util::readdata(left_kuan, kneel, anklel, right_kuan, kneer, ankler, data_total_filename);
	// if (ret_readdata_total == false) {
	// 	exit(-1);
	// }

	bool ret_readdata_begin = util::readdata(left_hip_begin, kneel_begin, anklel_begin, right_hip_begin, kneer_begin, ankler_begin, data_begin_filename);
	if (ret_readdata_begin == false) {
		exit(-1);
	}

	bool ret_readdata_period = util::readdata(left_hip_period, kneel_period, anklel_period, right_hip_period, kneer_period, ankler_period, data_period_filename);
	if (ret_readdata_period == false) {
		exit(-1);
	}
	//以下为发送电机指令的部分

	ros::Publisher linemotor_left_cmd_pub = nh.advertise<line_motor_comm_pkg::linemotorMsgCmd>("linemotor_0_cmd", 1);
	ros::Publisher linemotor_right_cmd_pub = nh.advertise<line_motor_comm_pkg::linemotorMsgCmd>("linemotor_1_cmd", 1);
	int linemotorID = 0;
    ros::Subscriber linemotor_left_state_sub = nh.subscribe<line_motor_comm_pkg::linemotorMsgBack>("left_knee_state", 1, boost::bind(&LineMotorCmdCallback, _1, linemotorID));
	linemotorID = 1;
	ros::Subscriber linemotor_right_state_sub = nh.subscribe<line_motor_comm_pkg::linemotorMsgBack>("right_knee_state", 1, boost::bind(&LineMotorCmdCallback, _1, linemotorID));

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

	// 创建IMU消息订阅者
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu_data", 10, imu_feedback);

	// 创建踝关节电机消息订阅和发布
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

    int steps = 0;
    // 新踝关节电机测试
	while(ros::ok())
	{
		ros::spinOnce();
		AnkleMotorCMD(1, SPEED_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
		AnkleMotorCMD(2, SPEED_CONTROL_STEP, 0, 0, 0, 20, 0, 5);
		// if(steps == 0)
		// {
		// 	steps = 1;
		// 	ankle_motor_ll_msg.motor_state = ZEROS_SETTING_STEP;
		// 	ankle_motor_lr_msg.motor_state = ZEROS_SETTING_STEP;
		// }
		// else if(steps == 1)
		// {
		// 	ankle_motor_ll_msg.motor_state = POS_CONTROL_STEP;
		// 	ankle_motor_ll_msg.pos = 150;
		// 	ankle_motor_ll_msg.spd = 200;
		// 	ankle_motor_ll_msg.limit_current = 50;
		// 	ankle_motor_lr_msg.motor_state = POS_CONTROL_STEP;
		// 	ankle_motor_lr_msg.pos = 150;
		// 	ankle_motor_lr_msg.spd = 200;
		// 	ankle_motor_lr_msg.limit_current = 50;
		// }	
		ankle_motor_ll_cmd_pub.publish(ankle_motor_ll_msg);
		ankle_motor_lr_cmd_pub.publish(ankle_motor_lr_msg);
		loop_rate.sleep();
	}

	//B1电机测试
	JointMotor motorL_0(0, MotorType::B1, nh, motor_name_[0]);//, test_motor_2(0, MotorType::A1);
	JointMotor motorL_2(2, MotorType::B1, nh, motor_name_[1]);
	JointMotor motorR_0(0, MotorType::B1, nh, motor_name_[2]);
	JointMotor motorR_2(2, MotorType::B1, nh, motor_name_[3]);

	// A1电机测试
	JointMotor motorL_0_a(0, MotorType::A1, nh, motor_name_[4]);//, test_motor_2(0, MotorType::A1);
	JointMotor motorL_1_a(1, MotorType::A1, nh, motor_name_[5]);
	JointMotor motorR_0_a(0, MotorType::A1, nh, motor_name_[6]);
	JointMotor motorR_1_a(1, MotorType::A1, nh, motor_name_[7]);

	// 旋转电机读取初始编码值
	for(int i = 0; i < 2000; i++)
	// while(1)
	{
		ros::spinOnce();
		motorL_0.BrakMode(1);
		motorL_2.BrakMode(1);
		motorR_0.BrakMode(2);
		motorR_2.BrakMode(2);
		motorL_0_a.BrakMode(3);
		motorL_1_a.BrakMode(3);
		motorR_0_a.BrakMode(4);
		motorR_1_a.BrakMode(4);
		usleep(usleep_time);
	}
	motorL_0_ORI = motorL_0.motor_ret.q / queryGearRatio(MotorType::B1);	//记录上电初始位置，之后在这个位置的基础上做增量
	motorL_2_ORI = motorL_2.motor_ret.q / queryGearRatio(MotorType::B1);
	motorR_0_ORI = motorR_0.motor_ret.q / queryGearRatio(MotorType::B1);
	motorR_2_ORI = motorR_2.motor_ret.q / queryGearRatio(MotorType::B1);
	
	motorL_0_ORI_a = motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1);	//记录上电初始位置，之后在这个位置的基础上做增量
	motorL_1_ORI_a = motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1);
	motorR_0_ORI_a = motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1);
	motorR_1_ORI_a = motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1);
	
	std::cout << "motorL_2_ORI   " << motorL_2_ORI << std::endl;
	std::cout << "motorR_2_ORI   " << motorR_2_ORI << std::endl;
	std::cout << "motorL_0_ORI   " << motorL_0_ORI << std::endl;
	std::cout << "motorR_0_ORI   " << motorR_0_ORI << std::endl;
	std::cout << "motorL_0_ORI_a   " << motorL_0_ORI_a << std::endl;
	std::cout << "motorL_1_ORI_a   " << motorL_1_ORI_a << std::endl;
	std::cout << "motorR_0_ORI_a   " << motorR_0_ORI_a << std::endl;
	std::cout << "motorR_1_ORI_a   " << motorR_1_ORI_a << std::endl;
	// }	

	// 到绝对位置起点
	// A1电机复位
	int InitPoint_num = 1000;
	for(int i = 0 ; i <= InitPoint_num ; i++)
	{
		ros::spinOnce();
		motorL_0.PosMode(3, 1, motorL_0_ORI + (StartPos_motorL_0 - motorL_0_ORI) * i / (float)InitPoint_num, 1);
		motorL_2.PosMode(3, 1, motorL_2_ORI + (StartPos_motorL_2 - motorL_2_ORI) * i / (float)InitPoint_num, 1);
		motorR_0.PosMode(3, 1, motorR_0_ORI + (StartPos_motorR_0 - motorR_0_ORI) * i / (float)InitPoint_num, 2);
		motorR_2.PosMode(3, 1, motorR_2_ORI + (StartPos_motorR_2 - motorR_2_ORI) * i / (float)InitPoint_num, 2);
		motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a + (StartPos_motorL_0_a - motorL_0_ORI_a) * i / (float)InitPoint_num, 3);
		motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a + (StartPos_motorL_1_a - motorL_1_ORI_a) * i / (float)InitPoint_num, 3);
		motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a + (StartPos_motorR_0_a - motorR_0_ORI_a) * i / (float)InitPoint_num, 4);
		motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a + (StartPos_motorR_1_a - motorR_1_ORI_a) * i / (float)InitPoint_num, 4);
		usleep(usleep_time);
	}
	
	// 给初始位置赋值
	motorL_0_ORI = StartPos_motorL_0;
	motorL_2_ORI = StartPos_motorL_2;
	motorR_0_ORI = StartPos_motorR_0;
	motorR_2_ORI = StartPos_motorR_2;
	motorL_0_ORI_a = StartPos_motorL_0_a;
	motorL_1_ORI_a = StartPos_motorL_1_a;
	motorR_0_ORI_a = StartPos_motorR_0_a;
	motorR_1_ORI_a = StartPos_motorR_1_a;

	std::cout << "L-0" << motorL_0.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "L-2" << motorL_2.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "R-0" << motorR_0.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "R-2" << motorR_2.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "L-0-A" << motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;
	std::cout << "L-1-A" << motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;
	std::cout << "R-0-A" << motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;
	std::cout << "R-1-A" << motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;

	line_motor_comm_pkg::linemotorMsgCmd linemotor_left_cmd_msg, linemotor_right_cmd_msg;

	int k_ankle_hip_roll=1;//踝关节侧摆差动大小与髋侧摆的关系

	int TempCnt1 = 0;
	// 下蹲 + 暂停
	for (int i = 0 ; i < 600 ; i++) {
		
		motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a - anklel_begin[i], 3);
		motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a + anklel_begin[i], 3);
		motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a + ankler_begin[i], 4);
		motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a - ankler_begin[i], 4);
		motorR_2.PosMode(3, 1, motorR_2_ORI - right_hip_begin[i], 2);
		motorL_2.PosMode(3, 1, motorL_2_ORI + left_hip_begin[i], 1);
		if (i % 6 == 0) {
			TempCnt1 += 6;
		}
		linemotor_left_cmd_msg.x = kneel_begin[TempCnt1];
		linemotor_left_cmd_msg.dx = 2000;
		linemotor_right_cmd_msg.x = kneer_begin[TempCnt1];
		linemotor_right_cmd_msg.dx = 2000;

		linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
		linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);
		// left_leg.Clear_PosCmd();
		// left_leg.RelPos_Set(kneel_begin[TempCnt1], 2000);       //此处的RelPos已变成绝对位置模式
		// right_leg.Clear_PosCmd();
		// right_leg.RelPos_Set(kneer_begin[TempCnt1], 2000);
		motorL_0.PosMode(0.5, 1, motorL_0_ORI, 1);//锁住侧摆关节
		motorR_0.PosMode(0.5, 1, motorR_0_ORI, 2);
		ros::spinOnce();
		loop_rate.sleep();
	}

	int CntPitch = 600;       // 往前迈步时的数组元素位置
	// 由直立向左侧摆
	for(int i = 0 ; i < 0.5 * unit_time ; i++) {      //每次走0.5个unit_time
		
		motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);//“加”为王梦迪的
		motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
		motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
		motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
		motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
		motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
		ros::spinOnce();
		loop_rate.sleep();
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

		motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a - anklel_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
		// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],8 ,Kp,  Kd);			// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],8 ,30,  3);
		motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a + anklel_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
		motorR_0_a.MixedMode(0, V_RightAnkle, motorR_0_ORI_a + ankler_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], Kp, Kd, 4);		// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],3, 30,  4);
		// motorR_0_a.PosMode(0.6, 10, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
		motorR_1_a.MixedMode(0, -V_RightAnkle, motorR_1_ORI_a - ankler_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], Kp, Kd, 4);		// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],3, 30,  4);
		// motorR_1_a.PosMode(0.6, 10, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
		// motorR_2.PosMode(1, 10, motorR_2_ORI-right_kuan[CntPitch], 2);
		motorR_2.MixedMode(0, -V_RightHip2 ,motorR_2_ORI - right_hip_begin[i], 3, 30,  2);  //之前刚度是3，阻尼是30
		// motorL_2.PosMode(1, 10, motorL_2_ORI+left_kuan[CntPitch], 1);
		motorL_2.MixedMode(0, V_LeftHip2, motorL_2_ORI + left_hip_begin[i], 3, 30, 1);

		if (i % 6 == 0) {	
			TempCnt2 += 6;
			if(TempCnt2 == number_begin) {
				TempCnt2--;
			}
		}
		std::cout << "TempCnt2:   " << TempCnt2 << std::endl;
		
		linemotor_left_cmd_msg.x = kneel_begin[TempCnt2];
		linemotor_left_cmd_msg.dx = 2000;
		linemotor_right_cmd_msg.x = kneer_begin[TempCnt2];
		linemotor_right_cmd_msg.dx = 2000;

		linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
		linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

		ros::spinOnce();
		loop_rate.sleep();

		// left_leg.Clear_PosCmd();
		// left_leg.RelPos_Set(kneel_begin[TempCnt2], 500);
		// right_leg.Clear_PosCmd();
		// right_leg.RelPos_Set(kneer_begin[TempCnt2], 500);
	}

	for(int i = 0.5 * unit_time ; i < unit_time ; i++) {      // 侧摆回正
		motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);//“加”为王梦迪的
		motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
		motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
		motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
		motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
		motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
		ros::spinOnce();
		loop_rate.sleep();
	}

	// 开始周期步态
	int loop_num;
	std::cout << "Please input the steps you want to execute: ";
	std::cin >> loop_num;
	for (int j = 0 ; j < loop_num ; j++) {
		ros::Rate inner_loop_rate(200);
		// 由直立侧摆向右
		for(int i = unit_time ; i < 1.5 * unit_time ; i++) {      
			motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);
			motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
			motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
			motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
			motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
			motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
			ros::spinOnce();
			inner_loop_rate.sleep();
		}

		int CntRoll = 1.5 * unit_time;
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

			motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a - anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);  //手扶用0.6
			// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
			motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a + anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
			// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
			motorR_0_a.PosMode(Kp, Kd, motorR_0_ORI_a + ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
			// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
			motorR_1_a.PosMode(Kp, Kd, motorR_1_ORI_a - ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
			motorR_2.PosMode(3, 30, motorR_2_ORI - right_hip_period[i], 2); //手扶用0.5
			// motorR_2.MixedMode(0,-V_RightHip2,motorR_2_ORI-right_kuan[CntPitch],0.5, 30,  2);
			motorL_2.PosMode(3, 30, motorL_2_ORI + left_hip_period[i], 1);

			if (i % 6 == 0) {	
				TempCnt2 += 6;
				if(TempCnt2 == number_period / 2) {
					TempCnt2--;
				}
			}
			std::cout << "TempCnt2:   " << TempCnt2 << std::endl;
			linemotor_left_cmd_msg.x = kneel_period[TempCnt2];
			linemotor_left_cmd_msg.dx = 2000;
			linemotor_right_cmd_msg.x = kneer_period[TempCnt2];
			linemotor_right_cmd_msg.dx = 2000;

			linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
			linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

			ros::spinOnce();
			inner_loop_rate.sleep();

			// left_leg.Clear_PosCmd();
			// left_leg.RelPos_Set(kneel_period[TempCnt2], 500);
			// right_leg.Clear_PosCmd();
			// right_leg.RelPos_Set(kneer_period[TempCnt2], 500);
		}

		// 由右侧向左侧摆至左侧
		for(int i = 1.5 * unit_time ; i < 2.5 * unit_time ; i++) {      
			motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);
			motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
			motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
			motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
			motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
			motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
			ros::spinOnce();
			inner_loop_rate.sleep();
		}

		CntRoll = 0.5 * unit_time;
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

			motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a - anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);  //手扶用0.6
			// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
			motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a + anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
			// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
			motorR_0_a.PosMode(Kp, Kd, motorR_0_ORI_a + ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
			// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
			motorR_1_a.PosMode(Kp, Kd, motorR_1_ORI_a - ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
			motorR_2.PosMode(3, 30, motorR_2_ORI - right_hip_period[i], 2); //手扶用0.5
			// motorR_2.MixedMode(0,-V_RightHip2,motorR_2_ORI-right_kuan[CntPitch],0.5, 30,  2);
			motorL_2.PosMode(3, 30, motorL_2_ORI + left_hip_period[i], 1);

			if (i % 6 == 0) {	
				TempCnt2 += 6;
				if(TempCnt2 == number_period) {
					TempCnt2--;
				}
			}
			std::cout << "TempCnt2:   " << TempCnt2 << std::endl;
			linemotor_left_cmd_msg.x = kneel_period[TempCnt2];
			linemotor_left_cmd_msg.dx = 2000;
			linemotor_right_cmd_msg.x = kneer_period[TempCnt2];
			linemotor_right_cmd_msg.dx = 2000;

			linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
			linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

			ros::spinOnce();
			inner_loop_rate.sleep();
			
			// left_leg.Clear_PosCmd();
			// left_leg.RelPos_Set(kneel_period[TempCnt2], 500);
			// right_leg.Clear_PosCmd();
			// right_leg.RelPos_Set(kneer_period[TempCnt2], 500);
		}

		// 由左侧向右侧摆至直立
		for(int i = 0.5 * unit_time ; i < unit_time ; i++) {      
			motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);
			motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
			motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
			motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
			motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
			motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
			ros::spinOnce();
			inner_loop_rate.sleep();
		}
	}




	// util::begin_gait(motorL_0_a, motorL_1_a, motorR_0_a, motorR_1_a, motorL_0, motorL_2, motorR_0, motorR_2, 
	// 				motorL_0_ORI_a, motorL_1_ORI_a, motorR_0_ORI_a, motorR_1_ORI_a, motorL_0_ORI, motorL_2_ORI, motorR_0_ORI, motorR_2_ORI,
	// 				left_hip_begin, kneel_begin, anklel_begin, right_hip_begin, kneer_begin, ankler_begin, 
	// 				left_leg, right_leg,
	// 				hip_roll, unit_time, k_ankle_hip_roll, number_begin,
	// 				linemotor_left_cmd_msg, linemotor_right_cmd_msg, linemotor_left_cmd_pub, linemotor_right_cmd_pub);

	int num = 0;
	while(num < number_begin)
	{
		ros::spinOnce();
		motorL_0.BrakMode(1);
		// usleep(5000);//之前是50000
		motorL_2.BrakMode(1);
		// usleep(5000);
		motorR_0.BrakMode(2);
		// usleep(5000);
		motorR_2.BrakMode(2);
		// usleep(5000);	
		motorL_0_a.BrakMode(3);
		// usleep(5000);
		motorL_1_a.BrakMode(3);
		// usleep(5000);
		motorR_0_a.BrakMode(4);
		// usleep(5000);
		motorR_1_a.BrakMode(4);
		// usleep(5000);

		linemotor_left_cmd_msg.x = kneel_begin[num];
		linemotor_left_cmd_msg.dx = 4000;
		linemotor_right_cmd_msg.x = kneer_begin[num];
		linemotor_right_cmd_msg.dx = 4000;

		linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
		linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

		num++;

		// std::cout<<"motorL_0:\t"<<motorL_0.motor_ret.q<<std::endl;
		// std::cout<<"motorL_2:\t"<<motorL_2.motor_ret.q<<std::endl;
		// std::cout<<"motorR_0:\t"<<motorR_0.motor_ret.q<<std::endl;
		// std::cout<<"motorR_2:\t"<<motorR_2.motor_ret.q<<std::endl;
		// std::cout<<"motorL_0_a:\t"<<motorL_0_a.motor_ret.q<<std::endl;
		// std::cout<<"motorL_1_a:\t"<<motorL_1_a.motor_ret.q<<std::endl;
		// std::cout<<"motorR_0_a:\t"<<motorR_0_a.motor_ret.q<<std::endl;
		// std::cout<<"motorR_1_a:\t"<<motorR_1_a.motor_ret.q<<std::endl;

		loop_rate.sleep();
	}

	drv_ready = 1;
	// left_leg.Clear_PosCmd();
	// right_leg.Clear_PosCmd();
	// left_leg.DrvPosMode();
	// right_leg.DrvPosMode();
	// left_leg.NMT_Enable(1);
	// right_leg.NMT_Enable(2);

	// uint8_t count = 0;

	// //以下为仲的侧摆分离步态，先侧摆到极值点再迈腿

	// std::vector<float> vel_hip_r_2, vel_hip_l_2, vel_ankle_l_0, vel_ankle_l_1, vel_ankle_r_0, vel_ankle_r_1;  
	// float Kp = 1;
	// float Kd = 10;

	// std::vector<int> durations;//保存每次运行的时间

	// util::begin_gait(motorL_0_a, motorL_1_a, motorR_0_a, motorR_1_a, motorL_0, motorL_2, motorR_0, motorR_2, 
	// 				motorL_0_ORI_a, motorL_1_ORI_a, motorR_0_ORI_a, motorR_1_ORI_a, motorL_0_ORI, motorL_2_ORI, motorR_0_ORI, motorR_2_ORI,
	// 				left_hip_begin, kneel_begin, anklel_begin, right_hip_begin, kneer_begin, ankler_begin, 
	// 				left_leg, right_leg,
	// 				hip_roll, unit_time, k_ankle_hip_roll, number_begin);

	// int loop_num;
	// std::cout << "Please input the steps you want to execute: ";
	// std::cin >> loop_num;
	// util::period_gait(motorL_0_a, motorL_1_a, motorR_0_a, motorR_1_a, motorL_0, motorL_2, motorR_0, motorR_2, 
	// 				motorL_0_ORI_a, motorL_1_ORI_a, motorR_0_ORI_a, motorR_1_ORI_a, motorL_0_ORI, motorL_2_ORI, motorR_0_ORI, motorR_2_ORI,
	// 				left_hip_period, kneel_period, anklel_period, right_hip_period, kneer_period, ankler_period, 
	// 				anklel_begin, ankler_begin, 
	// 				left_leg, right_leg,
	// 				hip_roll, unit_time, k_ankle_hip_roll, number_begin, number_period, loop_num);

	
	// // ros::Rate loop_rate(10); // 10Hz，周期为100ms

    // // while(ros::ok())
    // // {
	// 	// ROS_INFO("roll = %f pitch = %f yaw = %f",angular[0],angular[1],angular[2]);
	// 	// ROS_INFO("angular_vel_x = %f angular_vel_y = %f angular_vel_z = %f", angular_vel[0], angular_vel[1], angular_vel[2]);
	// 	// ROS_INFO("acceleration_x = %f acceleration_y = %f acceleration_z = %f", acceleration[0], acceleration[1],acceleration[2]);
	// 	// if(count < 20)
	// 	// {
	// 	// 	left_leg.RelPos_Set(1,300);
	// 	// 	count++;
	// 	// }		
	// 	// else
	// 	// {

	// 	// }

	// // 	ros::spinOnce();
    // //     loop_rate.sleep();
    // // }
    // // 写入 CSV 文件
    // std::ofstream csv_file("durations.csv");
    // if (csv_file.is_open()) {
    //     // 写入 CSV 头部
    //     csv_file << "Duration_Microseconds" << std::endl;

    //     // 写入每个测量结果
    //     for (int duration : durations) {
    //         csv_file << duration << std::endl;
    //     }

    //     csv_file.close();
    //     std::cout << "数据已写入 CSV 文件。" << std::endl;
    // } else {
    //     std::cerr << "无法打开 CSV 文件进行写入。" << std::endl;
    // }

	return 0;
}

void AnkleMotorCMD(int motor_num, int motor_state, float kp, float kd, float pos, float spd, float tau, float limit_current)
{
	switch(motor_num)
	{
		case 1: // 左腿左侧电机
			ankle_motor_ll_msg.motor_state = motor_state;
			ankle_motor_ll_msg.kp = kp;
			ankle_motor_ll_msg.kd = kd;
			ankle_motor_ll_msg.pos = pos;
			ankle_motor_ll_msg.spd = spd;
			ankle_motor_ll_msg.tau = tau;
			ankle_motor_ll_msg.limit_current = limit_current;
		break;
		case 2: // 左腿右侧电机
			ankle_motor_lr_msg.motor_state = motor_state;
			ankle_motor_lr_msg.kp = kp;
			ankle_motor_lr_msg.kd = kd;
			ankle_motor_lr_msg.pos = pos;
			ankle_motor_lr_msg.spd = spd;
			ankle_motor_lr_msg.tau = tau;
			ankle_motor_lr_msg.limit_current = limit_current;
		break;
		// case 3: // 右腿左侧电机
		// 	ankle_motor_rl_msg.motor_state = motor_state;
		// 	ankle_motor_rl_msg.kp = kp;
		// 	ankle_motor_rl_msg.kd = kd;
		// 	ankle_motor_rl_msg.pos = pos;
		// 	ankle_motor_rl_msg.spd = spd;
		// 	ankle_motor_rl_msg.tau = tau;
		// 	ankle_motor_rl_msg.limit_current = limit_current;
		// break;
		// case 4: // 右腿右侧电机
		// 	ankle_motor_rr_msg.motor_state = motor_state;
		// 	ankle_motor_rr_msg.kp = kp;
		// 	ankle_motor_rr_msg.kd = kd;
		// 	ankle_motor_rr_msg.pos = pos;
		// 	ankle_motor_rr_msg.spd = spd;
		// 	ankle_motor_rr_msg.tau = tau;
		// 	ankle_motor_rr_msg.limit_current = limit_current;
		// break;
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

void Receive_Zero_Fcn(int CANIndex)
{
	VCI_CAN_OBJ rec[300];//帧内容结构体，这里预留了300个帧的内容
	unsigned int reclen = 0;//接收到的帧的数量
	// int *run = (int*)param;
	// while((*run) & 0x0f)
	while(1)
	{
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, CANIndex, rec, 300, 1000)>0)//接收到超过一帧，接着往下走
		{
			for(int j = 0; j < reclen; j++)
			{
				if(rec[j].ID & 0x581 == 0x581) // 接收到回零状态
				{
					switch(rec[j].Data[0])
					{
						case 0x4b: // 16位数据
						{
							uint16_t index = ((uint16_t)rec[j].Data[2] << 8) | rec[j].Data[1]; 
							if(index = 0x6041) // 状态字
							{
								uint16_t status = ((uint16_t)rec[j].Data[5]<<8)|rec[j].Data[4];
								uint8_t temp_status;
								temp_status = (uint8_t)(status & 0x0001);
								if(temp_status)
								{
									go_zero_status_1 = temp_status;
								}
							}
							break;
						}
						default:
						{
							break;
						}
					}
				}
				else if(rec[j].ID == 1410) // 接收到回零状态
				{
					switch(rec[j].Data[0])
					{
						case 0x4b: // 16位数据
						{
							uint16_t index = ((uint16_t)rec[j].Data[2] << 8) | rec[j].Data[1]; 
							if(index = 0x6041) // 状态字
							{
								uint16_t status = ((uint16_t)rec[j].Data[5]<<8)|rec[j].Data[4];
								uint8_t temp_status;
								temp_status = (uint8_t)(status & 0x0001);
								if(temp_status)
								{
									go_zero_status_2 = temp_status;
								}
							}
							break;
						}
						default:
						{
							break;
						}
					}
				}
			}
		}
		if (go_zero_status_1 && go_zero_status_2)
		{
			break;
	}
		
	}
	// pthread_exit(0);
}


void Receive_Linemotor_Fcn(int CANIndex)
{
	VCI_CAN_OBJ rec[3000];
	unsigned int reclen = 0;
	// int *run = (int*)param;
	//while((*run) & 0x0f)
	while(1)
	{
		
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, CANIndex, rec, 3000, 100)>0)
		{
			// ROS_INFO("RECEIVED！ ");
			for(int j = 0; j < reclen; j++)
			{
				// cout<<std::hex<<(rec[j].ID & 0x0181)<<endl;
				if(rec[j].ID  == 0x0181) // sdo ID = 1
				{
					if(rec[j].Data[0] == 0x80)
					{
						ROS_ERROR_STREAM("Driver(ID=%d) is abnormal!"<<(rec[j].ID & 0x001));
					}
					else if(rec[j].Data[0] == 0x4b)// 查询status word位状态
					{
						//status_word = (rec[j].Data[] [j].Data[4]);
					}
					// cout<<"id = 1（左腿)） ";
					for(int i=0;i<6;i++)
					{
					// cout<<"rec["<<j<<"].Data["<<i<<"]:  "<<std::hex<< (rec[j].Data[i] & 0xFF) << "  ";
					}
					// cout << endl;
					RTPos_left = (((uint)rec[j].Data[5] << 24) | ((uint)rec[j].Data[4] << 16) | ((uint)rec[j].Data[3] << 8) | (uint)rec[j].Data[2]);
					// cout<<"线程中的RTPos_left:= "<<std::dec<<RTPos_left<<endl;
					// cout<<"线程中的函数增量"<<(float)((1000-RTPos_left)/50.0)<<endl;
				}
				else if(rec[j].ID  == 0x0182)
				{
					RTPos_right = (((uint)rec[j].Data[5] << 24) | ((uint)rec[j].Data[4] << 16) | ((uint)rec[j].Data[3] << 8) | (uint)rec[j].Data[2]);
				}
				// if((rec[j].ID & 0x0582) == 0x0582) // sdo ID = 2
				// {
				// 	if(rec[j].Data[0] == 0x80)
				// 	{
				// 		ROS_ERROR_STREAM("Driver(ID=%d) is abnormal!"<<(rec[j].ID & 0x001));
				// 	}
				// 	else if(rec[j].Data[0] == 0x4b)// 查询status word位状态
				// 	{
				// 		//status_word = (rec[j].Data[] [j].Data[4]);
				// 	}
				// 	cout<<"id = 2（右腿）  ";
				// 	for(int i=0;i<8;i++)
				// 	{
				// 	cout<<"rec["<<j<<"].Data["<<i<<"]:  "<<std::hex<< (rec[j].Data[i] & 0xFF) << "  ";
				// 	}
				// cout << endl;
				// }
				
				
				//读取位置，查CANopen手册


				//读取数据




			}
		}
	}
	pthread_exit(0);
}

int8_t CanPort_Open(void)
{
	// CAN通信板载信息获取，后续待优化
    VCI_BOARD_INFO pInfo1 [50];
    VCI_BOARD_INFO pInfo;
    int MotorNum = VCI_FindUsbDevice2(pInfo1);	// 获取计算机中已插入的USB-CAN适配器的数量
    if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)	// 设备类型：USBCAN-2A或USBCAN-2C或CANalyst-II  设备索引：第一个设备  保留参数（默认0）
    {
        ROS_INFO_STREAM(">>Open device success!");
    }
    else
    {
        ROS_INFO_STREAM(">>Open device failed!");
        return -1;
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1)
    {
        printf(">>Getting board information success!");
        printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);		// 此板卡的序列号，在CHAR str_Serial_Num[20];中
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);			// 硬件类型，比如“USBCAN V1.00”（注意：包括字符串结束符’\0’），在CHAR str_hw_Type[40];中
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (pInfo.fw_Version&0xF00)>>8);				// 固件版本号，用16进制表示。比如0x0100表示V1.00。
		printf(".");
		printf("%x", (pInfo.fw_Version&0xF0)>>4);
		printf("%x", pInfo.fw_Version&0xF);
		printf("\n");	
		return 0;
    }
    else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
		return -1;
    }
}