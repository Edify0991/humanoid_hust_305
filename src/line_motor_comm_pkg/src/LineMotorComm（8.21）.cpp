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
#define hip_pitch_target 0.10//弧度制
#define hip_roll_target 0.07//弧度制，最早能成功站立的一次是0.07,发改委来的时候是0.094
#define knee_target 30//单位mm


using namespace std;

// 该节点的全局变量
volatile uint16_t status_word = 0x00;
volatile uint8_t go_zero_status_1 = 0;
volatile uint8_t go_zero_status_2 = 0;
volatile uint8_t drv_ready = 0;
volatile double angular_vel[3];
volatile double acceleration[3];
volatile uint RTPos_left;//左腿绝对位置,1mm对应50编码器值
volatile uint RTPos_right;//右腿绝对位置
const int usleep_time = 100;

// 绝对起点位置
// 4个A1电机
const float StartPos_motorL_0_a = 0.4532;	// 0.0559651
const float StartPos_motorL_1_a = 0.183109;		// 0.302414
const float StartPos_motorR_0_a = 0.660077;		// 0.192464
const float StartPos_motorR_1_a = 0.333642;		// 0.254962
// 4个B1电机
const float StartPos_motorL_0 = 0.287268;		// 0.288463
const float StartPos_motorL_2 = 0.425211;		// 0.402892
const float StartPos_motorR_0 = 0.33815;		// 0.341737
const float StartPos_motorR_2 = 0.200029;		// 0.257421

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
float left_kuan[number] = {0};
float right_kuan[number] = {0};
float kneel[number] = {0};
float kneer[number] = {0};
float anklel[number] = {0};
float ankler[number] = {0};
float kneelrel[number] = {0}, kneerrel[number] = {0};

bool readdata() {
	std::ifstream file("/home/humanoid/Code/unitree_actuator_sdk/python/data.csv");
    std::string line;
    std::vector<std::string> column;

	if (file.is_open()) {
		int num = 0;
		while (std::getline(file, line)) {
			std::stringstream ss(line);
			std::string cell;
			for (int i = 0; std::getline(ss, cell, ','); ++i) {
				switch(i) {
					case 0:	// 第一列——left_angle_rad1
					{	
						left_kuan[num] = std::stof(cell);
						break; // 仅获取第二列后退出循环
					}
					case 1:	// left_line
					{	
						kneel[num] = std::stof(cell);
						break; // 仅获取第二列后退出循环
					}
					case 2:	// left_angle_rad3
					{	
						anklel[num] = std::stof(cell);
						break; // 仅获取第二列后退出循环
					}
					case 3:	// right_angle_rad1
					{	
						right_kuan[num] = std::stof(cell);
						break; // 仅获取第二列后退出循环
					}
					case 4:	// right_line
					{	
						kneer[num] = std::stof(cell);
						break; // 仅获取第二列后退出循环
					}
					case 5:	// right_angle_rad3
					{	
						ankler[num] = std::stof(cell);
						break; // 仅获取第二列后退出循环
					}
					default:
						break;
				}
				
			}
			num++;
		}
		file.close();
    } else {
        std::cerr << "无法打开文件" << std::endl;
        return false;
    }
	return true;
    // 输出第一列的内容
    // for (const auto& value : column) {
    //     std::cout << value << std::endl;
    // }
}

// 函数来写入 q/dq 数据到 CSV 文件
void writeToCSV(const std::string& filename, float data, string mark) {
    std::ofstream file;

    // 打开文件以追加模式
    file.open(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "无法打开文件 " << filename << std::endl;
        return;
    }

    // 将数据写入文件
	file << data << mark;

    file.close();
}

double angular[3];
ros::Publisher linemotor_state_pub;

void *Receive_Linemotor_Fcn(void* param);
void *Receive_Zero_Fcn(void* param);
void imu_feedback(sensor_msgs::Imu imu_data);
int8_t CanPort_Open(void);


int main(int argc, char** argv)
{	
	string filename = "cur_data.csv";
	//从文件中读数据
	bool ret_readdata = readdata();
	if (ret_readdata == false) {
		exit(-1);
	}
	//以下为发送电机指令的部分
    ros::init(argc, argv, "line_motor_comm_node");
    ros::NodeHandle nh;

	// 打开can通信端口(can分析仪)
	uint8_t can_port_state = CanPort_Open();
	if(can_port_state == -1)
	{
		exit(-1);
	}

	int unit_time=480;	// 循环步态中的半个周期
	// 定义三个关节在单个运动过程中前后摆位置数组
	// 髋关节侧摆（左右同步）
	float hip_roll_half[unit_time];
	float hip_roll[2 * unit_time+1];//这个+1用来在循环取余时对齐位置

	// 髋关节前后摆
	float hip_pitch_half[unit_time];
	float hip_pitch_L[2 * unit_time];
	float hip_pitch_R[2 * unit_time];

	// 膝关节弯曲
	float knee[unit_time + 1];//这个+1用来在循环取余时对齐位置
	float knee_rel[unit_time];
	float knee_L[2 * unit_time];
	float knee_R[2 * unit_time];

	// 计算膝关节弯曲
	float t = 0;	// 插值参数，范围在[0，1]之间
	float delta = M_PI/unit_time;
	for (int i = 0 ; i < unit_time ; i++)
	{
		knee[i] = knee_target * std::sin(t);
		t = t + delta;
		// std::cout <<"膝第"<<i<<"个元素是"<<knee[i]<< std::endl;
	}

	// 计算侧摆
	t = 0;
	delta = M_PI / unit_time;
	for (int i = 0 ; i < 2 * unit_time ; i++)
	{
		hip_roll[i] = hip_roll_target * std::sin(t);
		t = t + delta;
		std::cout << hip_roll[i] << "hip_roll" << std::endl;
	}

	// 计算前后摆
	t = 0;	// 插值参数，范围在[0，1]之间
	delta = M_PI / unit_time;
	for(int i = 0 ; i < unit_time ; i++)
	{
		hip_pitch_half[i] = hip_pitch_target * std::sin(t);

		// 构建左腿前后摆的完整数组
		hip_pitch_L[i] = hip_pitch_half[i];
		hip_pitch_L[i + unit_time] = 0;

		//构建右腿前后摆完整数组
		hip_pitch_R[i] = 0;
		hip_pitch_R[i+unit_time]=hip_pitch_half[i];

		t = t + delta;
	}

	knee_rel[0] = 0;
	for(int i = 0 ; i < unit_time ; i++) {
		// 膝盖增量数组
		if (i > 0) {
			knee_rel[i] = knee[i] - knee[i - 1];	
		}

		// 构建左腿膝盖完整数组
		knee_L[i] = knee_rel[i];
		knee_L[i + unit_time] = 0;

		// 构建右腿膝盖完整数组
		knee_R[i] = 0;
		knee_R[i + unit_time] = knee_rel[i];
	}

	float sum3 = 0, sum4 = 0;
	// 膝盖增量数组（含步态）	
	kneelrel[0] = 0;
	kneerrel[0] = 0;
	for(int i = 1 ; i < number ; i++) {
	// 构建左腿膝盖完整数组
	kneelrel[i] = kneel[i] - kneel[i - 1];	
	// 构建右腿膝盖完整数组
	kneerrel[i]=kneer[i]-kneer[i-1];

	}

	// 创建IMU消息订阅者
	ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu_data", 10, imu_feedback);

	// 创建接收汇川电机驱动器PDO数据的线程
	int PDO_run0 = 0;
	pthread_t Rcv_PDO_threadid;
	int ret;
	ret = pthread_create(&Rcv_PDO_threadid, NULL, Receive_Linemotor_Fcn, &PDO_run0);
	if(!ret)
	{
		cout<<"直线电机PDO接收进程创建成功"<<endl;
	}

	// 创建接收汇川电机驱动器回零位状态的线程
	int GoZero_run1 = 1;
	pthread_t Rcv_zero_threadid;
	int ret2;
	ret2 = pthread_create(&Rcv_zero_threadid, NULL, Receive_Zero_Fcn, &GoZero_run1);
	if(!ret2)
		{
			cout<<"直线电机回零状态接收进程创建成功"<<endl;
		}

	//B1电机测试
	JointMotor motorL_0(0, MotorType::B1);//, test_motor_2(0, MotorType::A1);
	JointMotor motorL_2(2, MotorType::B1);
	JointMotor motorR_0(0, MotorType::B1);
	JointMotor motorR_2(2, MotorType::B1);

	// A1电机测试
	JointMotor motorL_0_a(0, MotorType::A1);//, test_motor_2(0, MotorType::A1);
	JointMotor motorL_1_a(1, MotorType::A1);
	JointMotor motorR_0_a(0, MotorType::A1);
	JointMotor motorR_1_a(1, MotorType::A1);

	// 初始化左右腿直线电机
	LineMotor left_leg(1), right_leg(2);
	usleep(5000);
	left_leg.DrvEnable();
	usleep(5000);
	right_leg.DrvEnable();
	usleep(5000);
	// left_leg.RelPos_Set(15, 500);

	left_leg.Clear_PosCmd();
	usleep(5000);
	right_leg.Clear_PosCmd();
	usleep(5000);
	left_leg.DrvReset(); //复位
	usleep(3000000);
	right_leg.DrvReset();
	usleep(3000000);
	sleep(5);



	while((!go_zero_status_1) | (!go_zero_status_2))
	{
		left_leg.Gozero_Fdb();
		usleep(1000);
		right_leg.Gozero_Fdb();
	}
	cout<<"原点已回归，可以开始运动!"<<endl;

	for(int i = 0; i < 20; i++)
	// while(1)
	{
		motorL_0.BrakMode(1);
		usleep(5000);//之前是50000
		motorL_2.BrakMode(1);
		usleep(5000);
		motorR_0.BrakMode(2);
		usleep(5000);
		motorR_2.BrakMode(2);
		usleep(5000);	
		motorL_0_a.BrakMode(3);
		usleep(5000);
		motorL_1_a.BrakMode(3);
		usleep(5000);
		motorR_0_a.BrakMode(4);
		usleep(5000);
		motorR_1_a.BrakMode(4);
		usleep(5000);
	}
	motorL_0_ORI=motorL_0.motor_ret.q/ queryGearRatio(MotorType::B1);//记录上电初始位置，之后在这个位置的基础上做增量
	motorL_2_ORI=motorL_2.motor_ret.q/ queryGearRatio(MotorType::B1);
	motorR_0_ORI=motorR_0.motor_ret.q/ queryGearRatio(MotorType::B1);
	motorR_2_ORI=motorR_2.motor_ret.q/ queryGearRatio(MotorType::B1);
	
	motorL_0_ORI_a=motorL_0_a.motor_ret.q/ queryGearRatio(MotorType::A1);//记录上电初始位置，之后在这个位置的基础上做增量
	motorL_1_ORI_a=motorL_1_a.motor_ret.q/ queryGearRatio(MotorType::A1);
	motorR_0_ORI_a=motorR_0_a.motor_ret.q/ queryGearRatio(MotorType::A1);
	motorR_1_ORI_a=motorR_1_a.motor_ret.q/ queryGearRatio(MotorType::A1);
	
	std::cout <<  "motorL_2_ORI   "    << motorL_2_ORI<<  std::endl;
	std::cout <<  "motorR_2_ORI   "    << motorR_2_ORI<<  std::endl;
	std::cout <<  "motorL_0_ORI   "    << motorL_0_ORI<<  std::endl;
	std::cout <<  "motorR_0_ORI   "    << motorR_0_ORI<<  std::endl;
	std::cout <<  "motorL_0_ORI_a   "    << motorL_0_ORI_a<<  std::endl;
	std::cout <<  "motorL_1_ORI_a   "    << motorL_1_ORI_a<<  std::endl;
	std::cout <<  "motorR_0_ORI_a   "    << motorR_0_ORI_a<<  std::endl;
	std::cout <<  "motorR_1_ORI_a   "    << motorR_1_ORI_a<<  std::endl;
	// }	

	// 到绝对位置起点
	// A1电机复位
	int InitPoint_num = 100;
	for(int i=0;i<=InitPoint_num;i++)
	{
		motorL_0.PosMode(3, 1, motorL_0_ORI + (StartPos_motorL_0 - motorL_0_ORI) * i / (float)InitPoint_num, 1);
		usleep(usleep_time);
		motorL_2.PosMode(3, 1, motorL_2_ORI + (StartPos_motorL_2 - motorL_2_ORI) * i / (float)InitPoint_num, 1);
		usleep(usleep_time);
		motorR_0.PosMode(3, 1, motorR_0_ORI + (StartPos_motorR_0 - motorR_0_ORI) * i / (float)InitPoint_num, 2);
		usleep(usleep_time);
		motorR_2.PosMode(3, 1, motorR_2_ORI + (StartPos_motorR_2 - motorR_2_ORI) * i / (float)InitPoint_num, 2);
		usleep(usleep_time);
		motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a + (StartPos_motorL_0_a - motorL_0_ORI_a) * i / (float)InitPoint_num, 3);
		usleep(usleep_time);
		motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a + (StartPos_motorL_1_a - motorL_1_ORI_a) * i / (float)InitPoint_num, 3);
		usleep(usleep_time);
		motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a + (StartPos_motorR_0_a - motorR_0_ORI_a) * i / (float)InitPoint_num, 4);
		usleep(usleep_time);
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

	std::cout<<"L-0"<< motorL_0.motor_ret.q/ queryGearRatio(MotorType::B1)<<endl;
	std::cout<<"L-2"<< motorL_2.motor_ret.q/ queryGearRatio(MotorType::B1)<<endl;
	std::cout<<"R-0"<< motorR_0.motor_ret.q/ queryGearRatio(MotorType::B1)<<endl;
	std::cout<<"R-2"<< motorR_2.motor_ret.q/ queryGearRatio(MotorType::B1)<<endl;
	std::cout<<"L-0-A"<<motorL_0_a.motor_ret.q/ queryGearRatio(MotorType::A1) <<endl;
	std::cout<<"L-1-A"<< motorL_1_a.motor_ret.q/ queryGearRatio(MotorType::A1)<<endl;
	std::cout<<"R-0-A"<< motorR_0_a.motor_ret.q/ queryGearRatio(MotorType::A1)<<endl;
	std::cout<<"R-1-A"<< motorR_1_a.motor_ret.q/ queryGearRatio(MotorType::A1)<<endl;

	int cnt=0;//开始正式走步态了
	// while(cnt<2*unit_time)//走启动步态
	
	float sum1 = 0, sum2 = 0;
	int knee_cnt=0;//记录膝关节第几次运动
	float knee_bigrel=0, knee_bigabs;//膝关节每次大p2p运动的位移
	int KneeScale=4;//把膝关节一个周期的运动分成几份
	int TempCnt1=0;//下蹲过程中直线电机的绝对位置数组位置
	int TempCnt2=600;//迈步过程中直线电机的绝对位置数组位置

	int hip_flag=0;//侧摆开始
	int roll_flag=0;//判断是否应该触发侧摆运动
	int turn_flag = 0;	// 判断哪条腿为迈步腿，而其侧脚踝不需要侧摆
	int leg_flag = 0;//由turn_flag的奇偶来判断是迈哪条腿
	int leg_flag_temp=0;//记录上一个循环中的leg_flag,当上一个循环与本循环中的leg_flag不同时，触发侧摆运动
	int first_step=0;//走第一步时，侧摆0.25个周期，第二步及以后，侧摆走0.5个周期
	int total_steps=2;//总共走x步
	int current_step=0;//记录当前在走第几步
	float V_LeftHip2;//左髋前后摆速度
	float V_RightHip2;//右髋前后摆速度
	float V_LeftAnkle;//左髋前后摆速度
	float V_RightAnkle;//右髋前后摆速度
	int k_ankle_hip_roll=1;//踝关节侧摆差动大小与髋侧摆的关系

	knee[unit_time]=knee[unit_time-1];
	knee_bigabs = knee[0];

	GoZero_run1 = 0;
	drv_ready = 1;
	left_leg.Clear_PosCmd();
	right_leg.Clear_PosCmd();
	left_leg.DrvPosMode();
	right_leg.DrvPosMode();
	left_leg.NMT_Enable(1);
	right_leg.NMT_Enable(2);

	PDO_run0 = 1;
	uint8_t count = 0;

//以下为原版的侧摆与前后摆同步运行的步态
/*
	while(cnt < number)
	{	
		//以下为下蹲过程中的髋部运动		
		left_leg.Clear_PosCmd();
		right_leg.Clear_PosCmd();
		motorR_2.PosMode(0.5, 1, motorR_2_ORI-right_kuan[cnt], 2);
		usleep(100);
		motorL_2.PosMode(0.5, 1, motorL_2_ORI+left_kuan[cnt], 1);
		usleep(100);
		//以下为迈步过程
		if (cnt==600)//运动的前600个点为下蹲，不需要加侧摆
		{
			hip_flag=1;
		}
		if (cnt >= 600 && (cnt - 600) % 480 == 0) 
		{
			turn_flag++;
		}
		//以下为下蹲过程中的脚踝与膝关节运动
		if (hip_flag==0)
		{
			motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a-anklel[cnt], 3);
			usleep(100);
			motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a+anklel[cnt], 3);
			usleep(100);
			motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a+ankler[cnt], 4);
			usleep(100);
			motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a-ankler[cnt], 4);
			usleep(100);

			if (cnt % 6 == 0) 
			{	
				TempCnt1 += 6;
			}
			left_leg.RelPos_Set(kneel[TempCnt1], 4000);//此处的RelPos已变成绝对位置模式
			usleep(100);
			right_leg.RelPos_Set(kneer[TempCnt1], 4000);
			usleep(100);
			cnt=cnt+1;
		}
		if (hip_flag==1)
		{
			if (turn_flag != 0 && turn_flag % 2 == 1) 
			{	// 右腿迈步，右腿踝关节不侧摆
				motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a-anklel[cnt]+k_ankle_hip_roll*hip_roll[(cnt-600)%(2*unit_time)], 3);
				usleep(100);
				motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a+anklel[cnt]+k_ankle_hip_roll*hip_roll[(cnt-600)%(2*unit_time)], 3);
				usleep(100);
				motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a+ankler[cnt], 4);
				usleep(100);
				motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a-ankler[cnt], 4);


			}
			else if(turn_flag != 0 && turn_flag % 2 == 0) 
			{	// 左腿迈步，左腿踝关节不侧摆
				motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a-anklel[cnt], 3);
				usleep(100);
				motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a+anklel[cnt], 3);
				usleep(100);
				motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a+ankler[cnt]+k_ankle_hip_roll*hip_roll[(cnt-600)%(2*unit_time)], 4);
				usleep(100);
				motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a-ankler[cnt]+k_ankle_hip_roll*hip_roll[(cnt-600)%(2*unit_time)], 4);
				usleep(100);
			}

			motorL_0.PosMode(1, 1, motorL_0_ORI-hip_roll[(cnt-600)%(2*unit_time)], 1);//“加”为王梦迪的
			usleep(100);
			motorR_0.PosMode(1, 1, motorR_0_ORI-hip_roll[(cnt-600)%(2*unit_time)], 2);
			usleep(100);

			for(int i = cnt ; i<=cnt+5 ; i++ )
			{
				sum1 += kneelrel[i];
				sum2 += kneerrel[i];
			}

			
			if (cnt % 40 == 0 && cnt<number-40) {	
				TempCnt2 += 40;
			}
			left_leg.RelPos_Set(kneel[TempCnt2], 2000);
			usleep(200);
			// std::cout << "右腿目标位置：  "<<kneer[cnt+30] <<  std::endl;
			right_leg.RelPos_Set(kneer[TempCnt2], 2000);
			usleep(200);
			cnt=cnt+8;
		}
		usleep(1000);
	}
}
*/


	//以下为仲的侧摆分离步态，先侧摆到极值点再迈腿

	std::vector<float> vel_hip_r_2, vel_hip_l_2, vel_ankle_l_0, vel_ankle_l_1, vel_ankle_r_0, vel_ankle_r_1;  
	float Kp = 1;
	float Kd = 10;

	bool lastflag = false;


	while(CntPitch < number)
	{		
		// left_leg.Clear_PosCmd();
		// right_leg.Clear_PosCmd();

		//以下为迈步过程
		if (CntPitch == 600)	// 运动的前600个点为下蹲，不需要加侧摆,不需要加停顿
		{
			hip_flag=1;
		}

		if (CntPitch >= 600 && (CntPitch - 600) % 480 == 0)	// 600个点之后需要加侧摆和停顿
		{
			turn_flag++;
		}

		// 以下为下蹲过程中
		if (hip_flag == 0)
		{
			motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a - anklel[CntPitch], 3);
			usleep(usleep_time);
			motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a + anklel[CntPitch], 3);
			// usleep(usleep_time);
			motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a + ankler[CntPitch], 4);
			usleep(usleep_time);
			motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a - ankler[CntPitch], 4);
			// usleep(usleep_time);
			motorR_2.PosMode(3, 1, motorR_2_ORI-right_kuan[CntPitch], 2);
			usleep(usleep_time);
			motorL_2.PosMode(3, 1, motorL_2_ORI+left_kuan[CntPitch], 1);
			// usleep(usleep_time);
			if (CntPitch % 6 == 0) 
			{
				TempCnt1 += 6;
			}
			left_leg.RelPos_Set(kneel[TempCnt1], 2000);//此处的RelPos已变成绝对位置模式
			// usleep(usleep_time);
			usleep(1000);
			right_leg.RelPos_Set(kneer[TempCnt1], 2000);
			// usleep(usleep_time);
			usleep(1000);
			motorL_0.PosMode(0.5, 1, motorL_0_ORI, 1);//锁住侧摆关节
			usleep(usleep_time);
			motorR_0.PosMode(0.5, 1, motorR_0_ORI, 2);
			// usleep(usleep_time);
			CntPitch=CntPitch+1;
		}

		//以下为迈步过程
		if (hip_flag==1)
		{
			if (CntPitch<number-1)
			{
				V_LeftHip2= (left_kuan[CntPitch+1]-left_kuan[CntPitch])/0.03;
				V_RightHip2= (right_kuan[CntPitch+1]-right_kuan[CntPitch])/0.03;
				V_LeftAnkle= (anklel[CntPitch+1]-anklel[CntPitch])/0.03;
				V_RightAnkle= (ankler[CntPitch+1]-ankler[CntPitch])/0.03;
			}
			else 
			{
				V_LeftHip2= (0-left_kuan[CntPitch])/0.03;
				V_RightHip2= (0-right_kuan[CntPitch])/0.03;	
				V_LeftAnkle= (0-anklel[CntPitch])/0.03;	
				V_RightAnkle= (0-ankler[CntPitch])/0.03;
			}

			if (turn_flag != 0 && turn_flag % 2 == 1) //迈右腿
			{
				leg_flag=1;
				// 右腿迈步，右腿踝关节不侧摆
				if (first_step==0)//这一步是第一步
				{
					for(int i=0;i<0.5*unit_time;i++)//每次走0.5个unit_time
					{
						motorL_0.PosMode(6, 10, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], 1);//“加”为王梦迪的
						usleep(usleep_time);
						motorR_0.PosMode(6, 10, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], 2);
						usleep(usleep_time);
						motorL_0_a.PosMode(1, 20, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
						usleep(usleep_time);
						motorL_1_a.PosMode(1, 20, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
						usleep(usleep_time);
						motorR_0_a.PosMode(1, 20, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
						usleep(usleep_time);
						motorR_1_a.PosMode(1, 20, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
						CntRoll += 1;//这个增量不需要与CntPitch增量一致

						// writeToCSV(filename, motorL_2_ORI+left_kuan[CntPitch], ",");
						// writeToCSV(filename, motorR_2_ORI-right_kuan[CntPitch], ",");
						// writeToCSV(filename, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");

						// writeToCSV(filename, motorL_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorR_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorR_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorL_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), "\n");
					}
					first_step=1;
					leg_flag_temp=1;
				}
				if(leg_flag_temp != leg_flag)//当前循环中hil_flag发生变化
				{
					for(int i=0;i<unit_time;i++)//每次走1个unit_time
					{
						motorL_0.PosMode(6, 10, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], 1);//“加”为王梦迪的
						usleep(usleep_time);
						motorR_0.PosMode(6, 10, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], 2);
						usleep(usleep_time);
						motorL_0_a.PosMode(1, 20, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
						usleep(usleep_time);
						motorL_1_a.PosMode(1, 20, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
						usleep(usleep_time);
						motorR_0_a.PosMode(1, 20, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
						usleep(usleep_time);
						motorR_1_a.PosMode(1, 20, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
						CntRoll += 1;//这个增量不需要与CntPitch增量一致

						// writeToCSV(filename, motorL_2_ORI+left_kuan[CntPitch], ",");
						// writeToCSV(filename, motorR_2_ORI-right_kuan[CntPitch], ",");
						// writeToCSV(filename, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");

						// writeToCSV(filename, motorL_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorR_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorR_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorL_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), "\n");
					}
				}
				auto CurrentTime_before = std::chrono::system_clock::now();
				auto currentTime_ms_before = std::chrono::time_point_cast<std::chrono::milliseconds>(CurrentTime_before);
				auto value_ms_before = currentTime_ms_before.time_since_epoch().count();
				// std::cout << "time_before:  " << value_ms_before << std::endl;

//以下为迈腿，要调
				// motorL_0_a.MixedMode(0,-V_LeftAnkle,motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],8, Kp, Kd);//之前刚度是1,手扶稳定的刚度是0.5
				motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
				usleep(usleep_time);
				// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],8 ,Kp,  Kd);			// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],8 ,30,  3);
				motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
				usleep(usleep_time); 
				std::cout<<"L-0-A"<<motorL_0_a.motor_ret.q/ queryGearRatio(MotorType::A1) <<endl;
				std::cout<<"L-1-A"<< motorL_1_a.motor_ret.q/ queryGearRatio(MotorType::A1)<<endl;
				motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],Kp, Kd, 4);		// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],3, 30,  4);
				// motorR_0_a.PosMode(0.6, 10, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
				usleep(usleep_time);
				motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],Kp, Kd, 4);		// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],3, 30,  4);
				// motorR_1_a.PosMode(0.6, 10, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
				usleep(usleep_time);

				// motorR_2.PosMode(1, 10, motorR_2_ORI-right_kuan[CntPitch], 2);
				motorR_2.MixedMode(0,-V_RightHip2,motorR_2_ORI-right_kuan[CntPitch],3, 30,  2);//之前刚度是3，阻尼是30
				usleep(usleep_time);
				// motorL_2.PosMode(1, 10, motorL_2_ORI+left_kuan[CntPitch], 1);
				motorL_2.MixedMode(0,V_LeftHip2,motorL_2_ORI+left_kuan[CntPitch],3, 30,  1);
				leg_flag_temp=1;

				// writeToCSV(filename, motorL_2_ORI+left_kuan[CntPitch], ",");
				// writeToCSV(filename, motorR_2_ORI-right_kuan[CntPitch], ",");
				// writeToCSV(filename, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, V_LeftHip2, ",");
				// writeToCSV(filename, V_RightHip2, ",");
				// writeToCSV(filename, V_LeftAnkle, ",");
				// writeToCSV(filename, V_RightAnkle, ",");

				// writeToCSV(filename, motorL_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
				// writeToCSV(filename, motorR_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");

				// writeToCSV(filename, motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");

				// writeToCSV(filename, motorL_0_a.motor_ret.tau / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorL_1_a.motor_ret.tau / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_0_a.motor_ret.tau / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_1_a.motor_ret.tau / queryGearRatio(MotorType::A1), ",");

				// writeToCSV(filename, motorL_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
				// writeToCSV(filename, motorR_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");

				// writeToCSV(filename, motorL_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorL_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), "\n");
			}

			else if(turn_flag != 0 && turn_flag % 2 == 0) //迈左腿
			{
				leg_flag=0;
				// 左腿迈步，左腿踝关节不侧摆
				if(leg_flag_temp != leg_flag)//当前循环中hil_flag发生变化
				{
					for(int i=0;i<unit_time;i++)//每次走0.5个unit_time
						{
							motorL_0.PosMode(6, 10, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], 1);//“加”为王梦迪的
							usleep(usleep_time);
							motorR_0.PosMode(6, 10, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], 2);
							usleep(usleep_time);
							motorL_0_a.PosMode(1, 20, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
							usleep(usleep_time);
							motorL_1_a.PosMode(1, 20, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
							usleep(usleep_time);
							motorR_0_a.PosMode(1, 20, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
							usleep(usleep_time);
							motorR_1_a.PosMode(1, 20, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
							CntRoll += 1;//这个增量不需要与CntPitch增量一致

							// writeToCSV(filename, motorL_2_ORI+left_kuan[CntPitch], ",");
							// writeToCSV(filename, motorR_2_ORI-right_kuan[CntPitch], ",");
							// writeToCSV(filename, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
							// writeToCSV(filename, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
							// writeToCSV(filename, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
							// writeToCSV(filename, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
							// writeToCSV(filename, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
							// writeToCSV(filename, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
							// writeToCSV(filename, 0, ",");
							// writeToCSV(filename, 0, ",");
							// writeToCSV(filename, 0, ",");
							// writeToCSV(filename, 0, ",");

							// writeToCSV(filename, motorL_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
							// writeToCSV(filename, motorR_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
							// writeToCSV(filename, motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
							// writeToCSV(filename, motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
							// writeToCSV(filename, motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
							// writeToCSV(filename, motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
							// writeToCSV(filename, motorL_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
							// writeToCSV(filename, motorR_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
							// writeToCSV(filename, motorL_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
							// writeToCSV(filename, motorL_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
							// writeToCSV(filename, motorR_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
							// writeToCSV(filename, motorR_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), "\n");
						}
				}
				
//以下为迈腿，要调
				// motorL_0_a.MixedMode(0,-V_LeftAnkle,motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
				motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);//手扶用0.6
				std::cout<<"L-0-A位置"<<motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)]<<endl;
				usleep(usleep_time);
				// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
				motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);

				usleep(usleep_time);
				// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_0_a.PosMode(Kp, Kd, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
				usleep(usleep_time);
				// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_1_a.PosMode(Kp, Kd, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
				usleep(usleep_time);
				motorR_2.PosMode(3, 30, motorR_2_ORI-right_kuan[CntPitch], 2);//手扶用0.5
				// motorR_2.MixedMode(0,-V_RightHip2,motorR_2_ORI-right_kuan[CntPitch],0.5, 30,  2);
				usleep(usleep_time);
				motorL_2.PosMode(3, 30, motorL_2_ORI+left_kuan[CntPitch], 1);
				// motorL_2.MixedMode(0,V_LeftHip2,motorL_2_ORI+left_kuan[CntPitch],0.5, 30,  1);
				leg_flag_temp=0;

				// writeToCSV(filename, motorL_2_ORI+left_kuan[CntPitch], ",");
				// writeToCSV(filename, motorR_2_ORI-right_kuan[CntPitch], ",");
				// writeToCSV(filename, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
				// writeToCSV(filename, V_LeftHip2, ",");
				// writeToCSV(filename, V_RightHip2, ",");
				// writeToCSV(filename, V_LeftAnkle, ",");
				// writeToCSV(filename, V_RightAnkle, ",");

				// writeToCSV(filename, motorL_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
				// writeToCSV(filename, motorR_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
				// writeToCSV(filename, motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorL_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
				// writeToCSV(filename, motorR_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
				// writeToCSV(filename, motorL_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorL_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
				// writeToCSV(filename, motorR_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), "\n");
			}
			if (CntPitch % 2 == 0 && lastflag == false) 
			{	
				if (CntPitch >= number-38) {
					lastflag == true;
					TempCnt2 = number;
				}
				else
					TempCnt2 += 2;
			}
			std::cout << "TempCnt2:   " << TempCnt2 << std::endl;
			left_leg.RelPos_Set(kneel[TempCnt2], 500);
			// usleep(usleep_time);
			usleep(1000);
			// std::cout << "右腿目标位置：  "<<kneer[cnt+30] <<  std::endl;
			right_leg.RelPos_Set(kneer[TempCnt2], 500);
			usleep(usleep_time);
			CntPitch=CntPitch+1;

			if (leg_flag==0 && (((CntPitch - 600) % 480 == 0) || CntPitch == number))//刚刚迈的是左脚,且下一步要迈右脚
			{
				current_step++ ;//已经迈完一步，右脚、左脚各迈一步，才算完成一步
			}
			std::cout << "current_step: " << current_step << std::endl;
			if (current_step==total_steps)
			{
				left_leg.RelPos_Set(kneel[number], 200);
				usleep(usleep_time);
				// std::cout << "右腿目标位置：  "<<kneer[cnt+30] <<  std::endl;
				right_leg.RelPos_Set(kneer[number], 200);
				// usleep(usleep_time);
				// motorL_0_a.MixedMode(0,-V_LeftAnkle,motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
				motorL_0_a.PosMode(1, 30, motorL_0_ORI_a-anklel[number]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
				std::cout<<"L-0-A位置"<<motorL_0_ORI_a-anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)]<<endl;
				usleep(usleep_time);
				// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
				motorL_1_a.PosMode(1, 30, motorL_1_ORI_a+anklel[number]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);

				// usleep(usleep_time);
				// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_0_a.PosMode(1, 30, motorR_0_ORI_a+ankler[number]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
				usleep(usleep_time);
				// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_1_a.PosMode(1, 30, motorR_1_ORI_a-ankler[number]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
				// usleep(usleep_time);
				motorR_2.PosMode(3, 30, motorR_2_ORI-right_kuan[number], 2);
				// motorR_2.MixedMode(0,-V_RightHip2,motorR_2_ORI-right_kuan[CntPitch],0.5, 30,  2);
				usleep(usleep_time);
				motorL_2.PosMode(3, 30, motorL_2_ORI+left_kuan[number], 1);
				// motorL_2.MixedMode(0,V_LeftHip2,motorL_2_ORI+left_kuan[CntPitch],0.5, 30,  1);
				leg_flag_temp=0;



				std::cout<<"最后一步之前的CntPitch"<<CntPitch<<endl;
				CntRoll=1.5*unit_time;//只走最后0.25个周期
				std::cout<<"当前的CntRoll"<<CntRoll<<endl;
				for(int i=0;i<0.5*unit_time;i++)//每次走0.5个unit_time
					{
						motorL_0.PosMode(5, 5, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], 1);//“加”为王梦迪的
						usleep(2000);
						motorR_0.PosMode(5, 5, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], 2);
						usleep(2000);
						motorL_0_a.PosMode(1, 20, motorL_0_ORI_a-anklel[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
						usleep(2000);
						motorL_1_a.PosMode(1, 20, motorL_1_ORI_a+anklel[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 3);
						usleep(2000);
						motorR_0_a.PosMode(1, 20, motorR_0_ORI_a+ankler[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
						usleep(2000);
						motorR_1_a.PosMode(1, 20, motorR_1_ORI_a-ankler[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
						CntRoll += 1;//这个增量不需要与CntPitch增量一致

						// writeToCSV(filename, motorL_2_ORI+left_kuan[number], ",");
						// writeToCSV(filename, motorR_2_ORI-right_kuan[number], ",");
						// writeToCSV(filename, motorL_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_0_ORI-hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorL_0_ORI_a-anklel[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorL_1_ORI_a+anklel[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_0_ORI_a+ankler[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, motorR_1_ORI_a-ankler[CntPitch-8]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");
						// writeToCSV(filename, 0, ",");

						// writeToCSV(filename, motorL_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorR_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorR_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
						// writeToCSV(filename, motorL_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorL_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
						// writeToCSV(filename, motorR_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), "\n");
					}
				return 0;
			}
		}
		usleep(usleep_time);
		auto CurrentTime_after = std::chrono::system_clock::now();
		auto currentTime_ms_after = std::chrono::time_point_cast<std::chrono::milliseconds>(CurrentTime_after);
		auto value_ms_after = currentTime_ms_after.time_since_epoch().count();
		// std::cout << "time_after:  " << value_ms_after << std::endl;

		// vel_hip_r_2.push_back(motorL_2.motor_ret.q / queryGearRatio(MotorType::B1));
		// vel_hip_l_2.push_back(motorR_2.motor_ret.q / queryGearRatio(MotorType::B1));
		// vel_ankle_l_0.push_back(motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1));
		// vel_ankle_l_1.push_back(motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1));
		// vel_ankle_r_0.push_back(motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1));
		// vel_ankle_r_1.push_back(motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1));
		// writeToCSV(filename, motorL_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
		// writeToCSV(filename, motorR_2.motor_ret.q / queryGearRatio(MotorType::B1), ",");
		// writeToCSV(filename, motorL_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
		// writeToCSV(filename, motorL_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
		// writeToCSV(filename, motorR_0_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
		// writeToCSV(filename, motorR_1_a.motor_ret.q / queryGearRatio(MotorType::A1), ",");
		// writeToCSV(filename, motorL_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
		// writeToCSV(filename, motorR_2.motor_ret.dq / queryGearRatio(MotorType::B1), ",");
		// writeToCSV(filename, motorL_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
		// writeToCSV(filename, motorL_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
		// writeToCSV(filename, motorR_0_a.motor_ret.dq / queryGearRatio(MotorType::A1), ",");
		// writeToCSV(filename, motorR_1_a.motor_ret.dq / queryGearRatio(MotorType::A1), "\n");
		
		
	}
	// ros::Rate loop_rate(10); // 10Hz，周期为100ms

    // while(ros::ok())
    // {
		// ROS_INFO("roll = %f pitch = %f yaw = %f",angular[0],angular[1],angular[2]);
		// ROS_INFO("angular_vel_x = %f angular_vel_y = %f angular_vel_z = %f", angular_vel[0], angular_vel[1], angular_vel[2]);
		// ROS_INFO("acceleration_x = %f acceleration_y = %f acceleration_z = %f", acceleration[0], acceleration[1],acceleration[2]);
		// if(count < 20)
		// {
		// 	left_leg.RelPos_Set(1,300);
		// 	count++;
		// }		
		// else
		// {

		// }

	// 	ros::spinOnce();
    //     loop_rate.sleep();
    // }
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

void* Receive_Zero_Fcn(void* param)
{
	VCI_CAN_OBJ rec[300];//帧内容结构体，这里预留了300个帧的内容
	unsigned int reclen = 0;//接收到的帧的数量
	int *run = (int*)param;
	while((*run) & 0x0f)
	{
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, 0, rec, 300, 1000)>0)//接收到超过一帧，接着往下走
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
	}
	pthread_exit(0);
}


void* Receive_Linemotor_Fcn(void* param)
{
	VCI_CAN_OBJ rec[3000];
	unsigned int reclen = 0;
	int *run = (int*)param;
	//while((*run) & 0x0f)
	while(1)
	{
		
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100)>0)
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