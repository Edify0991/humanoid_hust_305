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
#define hip_pitch_target 0.10//弧度制
#define hip_roll_target 0.07//弧度制，最早能成功站立的一次是0.07,发改委来的时候是0.094
#define knee_target 30//单位mm


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

// 绝对起点位置
// 4个A1电机
const float StartPos_motorL_0_a = 0.60348;	// 0.0559651
const float StartPos_motorL_1_a = 0.032239;		// 0.302414
const float StartPos_motorR_0_a = 0.518099;		// 0.192464
const float StartPos_motorR_1_a = 0.476842;		// 0.254962
// 4个B1电机
const float StartPos_motorL_0 = 0.276551;		// 0.288463
const float StartPos_motorL_2 = 0.336378;		// 0.402892
const float StartPos_motorR_0 = 0.341471;		// 0.341737
const float StartPos_motorR_2 = 0.321101;		// 0.257421

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

// float left_hip_period[number_period] = {0};
// float right_hip_period[number_period] = {0};
// float kneel_period[number_period] = {0};
// float kneer_period[number_period] = {0};
// float anklel_period[number_period] = {0};
// float ankler_period[number_period] = {0};

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


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "gait_host");
    ros::NodeHandle nh;
	ros::Rate loop_rate(80);

	string filename = "cur_data.csv";
	string data_total_filename = "/home/humanoid/humanoid_hust_305/强化学习开环数据0831.csv";
	//从文件中读数据
	// bool ret_readdata_total = util::readdata(left_kuan, kneel, anklel, right_kuan, kneer, ankler, data_total_filename);
	// if (ret_readdata_total == false) {
	// 	exit(-1);
	// }

    float* left_hip_roll;
    float* left_pitch;
    float* left_knee;
    float* left_ankle;
    float* left_foot;
    float* right_hip_roll;
    float* right_pitch;
    float* right_knee;
    float* right_ankle;
    float* right_foot;
    int point_cnt = util::readdatafromcsv(&left_hip_roll, &left_pitch, &left_knee, &left_ankle, &left_foot, &right_hip_roll, &right_pitch, &right_knee, &right_ankle, &right_foot, data_total_filename);
	if (point_cnt == 0) {
		exit(-1);
	}

	//以下为发送电机指令的部分

	ros::Publisher linemotor_left_cmd_pub = nh.advertise<line_motor_comm_pkg::linemotorMsgCmd>("linemotor_0_cmd", 1);
	ros::Publisher linemotor_right_cmd_pub = nh.advertise<line_motor_comm_pkg::linemotorMsgCmd>("linemotor_1_cmd", 1);
	int linemotorID = 0;
    ros::Subscriber linemotor_left_state_sub = nh.subscribe<line_motor_comm_pkg::linemotorMsgBack>("left_knee_state", 1, boost::bind(&LineMotorCmdCallback, _1, linemotorID));
	linemotorID = 1;
	ros::Subscriber linemotor_right_state_sub = nh.subscribe<line_motor_comm_pkg::linemotorMsgBack>("right_knee_state", 1, boost::bind(&LineMotorCmdCallback, _1, linemotorID));

	// 创建IMU消息订阅者
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu_data", 10, imu_feedback);
    
    //B1电机
	JointMotor motorL_0(0, MotorType::B1, nh, motor_name_[0]);//, test_motor_2(0, MotorType::A1);
	JointMotor motorL_2(2, MotorType::B1, nh, motor_name_[1]);
	JointMotor motorR_0(0, MotorType::B1, nh, motor_name_[2]);
	JointMotor motorR_2(2, MotorType::B1, nh, motor_name_[3]);

	// A1电机
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
	// 宇树旋转电机复位
	int InitPoint_num = 100;
	for(int i = 0 ; i <= InitPoint_num ; i++)
	{
		ros::spinOnce();
		motorL_0.PosMode(1, 1, motorL_0_ORI + (StartPos_motorL_0 - motorL_0_ORI) * i / (float)InitPoint_num, 1);
		motorL_2.PosMode(1, 1, motorL_2_ORI + (StartPos_motorL_2 - motorL_2_ORI) * i / (float)InitPoint_num, 1);
		motorR_0.PosMode(1, 1, motorR_0_ORI + (StartPos_motorR_0 - motorR_0_ORI) * i / (float)InitPoint_num, 2);
		motorR_2.PosMode(1, 1, motorR_2_ORI + (StartPos_motorR_2 - motorR_2_ORI) * i / (float)InitPoint_num, 2);
		motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a + (StartPos_motorL_0_a - motorL_0_ORI_a) * i / (float)InitPoint_num, 3);
		motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a + (StartPos_motorL_1_a - motorL_1_ORI_a) * i / (float)InitPoint_num, 3);
		motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a + (StartPos_motorR_0_a - motorR_0_ORI_a) * i / (float)InitPoint_num, 4);
		motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a + (StartPos_motorR_1_a - motorR_1_ORI_a) * i / (float)InitPoint_num, 4);
		loop_rate.sleep();
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

	line_motor_comm_pkg::linemotorMsgCmd linemotor_left_cmd_msg, linemotor_right_cmd_msg;
    // 宇树旋转电机到达初始位置

    // left_hip_roll, left_pitch, left_knee, left_ankle, left_foot, right_hip_roll, right_pitch, right_knee, right_ankle, right_foot
    for(int i = 0 ; i <= InitPoint_num ; i++)
	{
		ros::spinOnce();
		std::cout << "当前到初始位置的进程： " << i << "/" <<  InitPoint_num << std::endl;
		motorL_0.PosMode(1, 1, motorL_0_ORI + left_hip_roll[0] * i / (float)InitPoint_num, 1);
		motorL_2.PosMode(1, 1, motorL_2_ORI + left_pitch[0] * i / (float)InitPoint_num, 1);
		motorR_0.PosMode(1, 1, motorR_0_ORI + right_hip_roll[0] * i / (float)InitPoint_num, 2);
		motorR_2.PosMode(1, 1, motorR_2_ORI + right_pitch[0] * i / (float)InitPoint_num, 2);
		motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a + left_foot[0] * i / (float)InitPoint_num, 3);
		motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a + left_ankle[0] * i / (float)InitPoint_num, 3);
		motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a + right_foot[0] * i / (float)InitPoint_num, 4);
		motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a + right_ankle[0] * i / (float)InitPoint_num, 4);

		linemotor_left_cmd_msg.x = left_knee[0] * i / (float)InitPoint_num;
		linemotor_left_cmd_msg.dx = 2000;
		linemotor_right_cmd_msg.x = right_knee[0] * i / (float)InitPoint_num;
		linemotor_right_cmd_msg.dx = 2000;

		linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
		linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

		loop_rate.sleep();
	}
	
	std::cout << "L-0" << motorL_0.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "L-2" << motorL_2.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "R-0" << motorR_0.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "R-2" << motorR_2.motor_ret.q / queryGearRatio(MotorType::B1) << std::endl;
	std::cout << "L-0-A" << motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;
	std::cout << "L-1-A" << motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;
	std::cout << "R-0-A" << motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;
	std::cout << "R-1-A" << motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1) << std::endl;

	// line_motor_comm_pkg::linemotorMsgCmd linemotor_left_cmd_msg, linemotor_right_cmd_msg;

	// util::begin_gait(motorL_0_a, motorL_1_a, motorR_0_a, motorR_1_a, motorL_0, motorL_2, motorR_0, motorR_2, 
	// 				motorL_0_ORI_a, motorL_1_ORI_a, motorR_0_ORI_a, motorR_1_ORI_a, motorL_0_ORI, motorL_2_ORI, motorR_0_ORI, motorR_2_ORI,
	// 				left_hip_begin, kneel_begin, anklel_begin, right_hip_begin, kneer_begin, ankler_begin, 
	// 				left_leg, right_leg,
	// 				hip_roll, unit_time, k_ankle_hip_roll, number_begin);

	int num = 1;
	float pitch_roll_max = 0.45;
	while(num < point_cnt)
	{
		if(left_pitch[num] > pitch_roll_max) {
			left_pitch[num] = pitch_roll_max;
		}
		if(right_pitch[num] < -pitch_roll_max) {
			right_pitch[num] = -pitch_roll_max;
		}
		motorL_0.PosMode(1, 1, motorL_0_ORI + left_hip_roll[num], 1);
		motorL_2.PosMode(1, 1, motorL_2_ORI + left_pitch[num], 1);
		motorR_0.PosMode(1, 1, motorR_0_ORI + right_hip_roll[num], 2);
		motorR_2.PosMode(1, 1, motorR_2_ORI + right_pitch[num], 2);
		motorL_0_a.PosMode(0.1, 5, motorL_0_ORI_a + left_foot[num], 3);
		motorL_1_a.PosMode(0.1, 5, motorL_1_ORI_a + left_ankle[num], 3);
		motorR_0_a.PosMode(0.1, 5, motorR_0_ORI_a + right_foot[num], 4);
		motorR_1_a.PosMode(0.1, 5, motorR_1_ORI_a + right_ankle[num], 4);

		linemotor_left_cmd_msg.x = left_knee[num];
		linemotor_left_cmd_msg.dx = 4000;
		linemotor_right_cmd_msg.x = right_knee[num];
		linemotor_right_cmd_msg.dx = 4000;

		// linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
		linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

		num++;

		std::cout<<"motorL_0:\t"<<motorL_0.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;
		std::cout<<"motorL_2:\t"<<motorL_2.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;
		std::cout<<"motorR_0:\t"<<motorR_0.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;
		std::cout<<"motorR_2:\t"<<motorR_2.motor_ret.q / queryGearRatio(MotorType::B1)<<std::endl;
		std::cout<<"motorL_0_a:\t"<<motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1)<<std::endl;
		std::cout<<"motorL_1_a:\t"<<motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1)<<std::endl;
		std::cout<<"motorR_0_a:\t"<<motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1)<<std::endl;
		std::cout<<"motorR_1_a:\t"<<motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1)<<std::endl;
		ros::spinOnce();
		loop_rate.sleep();
	}


	while(ros::ok())
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

		// linemotor_left_cmd_msg.x = kneel_begin[num];
		// linemotor_left_cmd_msg.dx = 4000;
		// linemotor_right_cmd_msg.x = kneer_begin[num];
		// linemotor_right_cmd_msg.dx = 4000;

		// linemotor_left_cmd_pub.publish(linemotor_left_cmd_msg);
		// linemotor_right_cmd_pub.publish(linemotor_right_cmd_msg);

		// num++;

		std::cout<<"motorL_0:\t"<<motorL_0.motor_ret.q / queryGearRatio(MotorType::B1) - motorL_0_ORI<<std::endl;
		std::cout<<"motorL_2:\t"<<motorL_2.motor_ret.q / queryGearRatio(MotorType::B1) - motorL_2_ORI<<std::endl;
		std::cout<<"motorR_0:\t"<<motorR_0.motor_ret.q / queryGearRatio(MotorType::B1) - motorR_0_ORI<<std::endl;
		std::cout<<"motorR_2:\t"<<motorR_2.motor_ret.q / queryGearRatio(MotorType::B1) - motorR_2_ORI<<std::endl;
		std::cout<<"motorL_0_a:\t"<<motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1) - motorL_0_ORI_a<<std::endl;
		std::cout<<"motorL_1_a:\t"<<motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1) - motorL_1_ORI_a<<std::endl;
		std::cout<<"motorR_0_a:\t"<<motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1) - motorR_0_ORI_a<<std::endl;
		std::cout<<"motorR_1_a:\t"<<motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1) - motorR_1_ORI_a<<std::endl;

		loop_rate.sleep();
	}

	float V_LeftHip2;			//左髋前后摆速度
	float V_RightHip2;			//右髋前后摆速度
	float V_LeftAnkle;			//左髋前后摆速度
	float V_RightAnkle;			//右髋前后摆速度
	int k_ankle_hip_roll = 1;	//踝关节侧摆差动大小与髋侧摆的关系

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