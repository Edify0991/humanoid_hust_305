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
#define hip_roll_target 0.094//弧度制，最早能成功站立的一次是0.07
#define knee_target 30//单位mm

using namespace std;

const double PI = 3.1415926;

// 该节点的全局变量
volatile uint16_t status_word = 0x00;
volatile uint8_t go_zero_status_1 = 0;
volatile uint8_t go_zero_status_2 = 0;
volatile uint8_t drv_ready = 0;
volatile double angular_vel[3];
volatile double acceleration[3];
volatile uint RTPos_left;//左腿绝对位置,1mm对应50编码器值
volatile uint RTPos_right;//右腿绝对位置
const int usleep_time = 1000;

// 绝对起点位置
// 4个A1电机
const float StartPos_motorL_0_a = 0.0559651;
const float StartPos_motorL_1_a = 0.302414;
const float StartPos_motorR_0_a = 0.192464;
const float StartPos_motorR_1_a = 0.254962;
// 4个B1电机
const float StartPos_motorL_0 = 0.261716;
const float StartPos_motorL_2 = 0.489245;
const float StartPos_motorR_0 = 0.30755;
const float StartPos_motorR_2 = 0.183157;

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

double angular[3];
ros::Publisher linemotor_state_pub;

void *Receive_Linemotor_Fcn(void* param);
void *Receive_Zero_Fcn(void* param);
void imu_feedback(sensor_msgs::Imu imu_data);
int8_t CanPort_Open(void);
float control_single_foot_standing(double* IMU_data, double* IMU_data_ref);


int main(int argc, char** argv) {
    ros::init(argc, argv, "Single_Leg_Standing_node");
    ros::NodeHandle nh;

    float motorL_0_ORI;//记录B1电机上电初始值,0是侧摆电机，2是前后摆电机
    float motorL_2_ORI;
    float motorR_0_ORI;
    float motorR_2_ORI;
    float motorL_0_ORI_a;
    float motorL_1_ORI_a;
    float motorR_0_ORI_a;
    float motorR_1_ORI_a;

    // 打开can通信端口(can分析仪)
	uint8_t can_port_state = CanPort_Open();
	if(can_port_state == -1)
	{
		exit(-1);
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
	LineMotor left_leg(1,1), right_leg(2,0);
	left_leg.DrvEnable();
	right_leg.DrvEnable();

	left_leg.DrvReset(); //复位
	usleep(3000000);
	right_leg.DrvReset();
	usleep(3000000);


	while((!go_zero_status_1) | (!go_zero_status_2))
	{
		left_leg.Gozero_Fdb();
		usleep(1000);
		right_leg.Gozero_Fdb();
		usleep(1000);
	}
	sleep(4);
	cout<<"原点已回归，可以开始运动!"<<endl;

	for(int i = 0; i < 20; i++)
	{
		motorL_0.BrakMode(1);
		usleep(500);//之前是50000
		motorL_2.BrakMode(1);
		usleep(500);
		motorR_0.BrakMode(2);
		usleep(500);
		motorR_2.BrakMode(2);
		usleep(500);	
		motorL_0_a.BrakMode(3);
		usleep(50000);
		motorL_1_a.BrakMode(3);
		usleep(50000);
		motorR_0_a.BrakMode(4);
		usleep(50000);
		motorR_1_a.BrakMode(4);
		usleep(50000);
	}

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

    motorL_0_ORI=motorL_0.motor_ret.q/ queryGearRatio(MotorType::B1);//记录上电初始位置，之后在这个位置的基础上做增量
	motorL_2_ORI=motorL_2.motor_ret.q/ queryGearRatio(MotorType::B1);
	motorR_0_ORI=motorR_0.motor_ret.q/ queryGearRatio(MotorType::B1);
	motorR_2_ORI=motorR_2.motor_ret.q/ queryGearRatio(MotorType::B1);
	
	motorL_0_ORI_a=motorL_0_a.motor_ret.q/ queryGearRatio(MotorType::A1);//记录上电初始位置，之后在这个位置的基础上做增量
	motorL_1_ORI_a=motorL_1_a.motor_ret.q/ queryGearRatio(MotorType::A1);
	motorR_0_ORI_a=motorR_0_a.motor_ret.q/ queryGearRatio(MotorType::A1);
	motorR_1_ORI_a=motorR_1_a.motor_ret.q/ queryGearRatio(MotorType::A1);

    ros::Rate loop_rate(10); // 10Hz，周期为100ms

    double angular_ref[3] = {0};

    float hip_c;  // 髋关节控制量

    int usleep_time = 100;

    float motorL_2_Cur = motorL_2_ORI;  // motorL_2_Cur为当前电机角度
    while(ros::ok())
    {
		ROS_INFO("roll = %f pitch = %f yaw = %f",angular[0],angular[1],angular[2]);
		ROS_INFO("angular_vel_x = %f angular_vel_y = %f angular_vel_z = %f", angular_vel[0], angular_vel[1], angular_vel[2]);
		ROS_INFO("acceleration_x = %f acceleration_y = %f acceleration_z = %f", acceleration[0], acceleration[1],acceleration[2]);
		if(count < 20)
		{
			// left_leg.RelPos_Set(1,300);
			// count++;
		}		
		else
		{

		}
        
        // 待imu读取数据后
        hip_c = control_single_foot_standing(angular, angular_ref); // 利用pid控制求解出髋关节控制量

        if (abs(hip_c) > 0.02) {
            hip_c = hip_c > 0 ? 0.02 : -0.02;
        }

        motorL_2.PosMode(3, 1, motorL_2_Cur + hip_c, 1);
        motorL_2_Cur = motorL_2.motor_ret.q / queryGearRatio(MotorType::B1); //记录当前位置

        // 其余电机锁死
        motorL_0.PosMode(3, 1, motorL_0_ORI, 1);
		usleep(usleep_time);
		motorR_0.PosMode(3, 1, motorR_0_ORI, 2);
		usleep(usleep_time);
		motorR_2.PosMode(3, 1, motorR_2_ORI, 2);
		usleep(usleep_time);
		motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a, 3);
		usleep(usleep_time);
		motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a, 3);
		usleep(usleep_time);
		motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a, 4);
		usleep(usleep_time);
		motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a, 4);
		usleep(usleep_time);

		ros::spinOnce();
        loop_rate.sleep();
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

/*
 * 函数名称：control_single_foot_standing
 * 函数描述：控制单腿站立
 * 函数输入：IMU数据
 * 函数返回：关节增量角度。从正面看，转动的增量，逆时针为正
 */
float control_single_foot_standing(double* IMU_data, double* IMU_data_ref) {
    // pid控制器参数
    static float kp = 1;
    static float ki = 0.0;
    static float kd = 0.0;
    static float error_last = 0;
    static float error_sum = 0;
    static float error_sum_max = 100 / 180.0 * PI;
    static float output_max = 10 / 180.0 * PI;
    // 读取IMU当前帧数据
    float roll = IMU_data[0];
    float pitch = IMU_data[1];
    float yaw = IMU_data[2];
    // 读取IMU参考数据
    float roll_ref = IMU_data_ref[0];
    float pitch_ref = IMU_data_ref[1];
    float yaw_ref = IMU_data_ref[2];
    // 当前只用横滚角度进行控制
    float error = roll - roll_ref;
    float output = kp * error + ki * error_sum + kd * (error - error_last);
    // 更新误差
    error_last = error;
    error_sum += error;
    if (error_sum > error_sum_max) {
        error_sum = error_sum_max;
    } else if (error_sum < -error_sum_max) {
        error_sum = -error_sum_max;
    }
    // 返回脚踝关节角度
    if (output > output_max) {
        output = output_max;
    } else if (output < -output_max) {
        output = -output_max;
    }
    return output;
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