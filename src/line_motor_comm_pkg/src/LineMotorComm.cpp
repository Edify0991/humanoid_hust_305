#include "Modbus_Comm.h"
#include <cmath>
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "B1MotorControl.h"
#include "conio.h"
#include <sensor_msgs/Imu.h>
#include "tf/tf.h"

using namespace std;

// 该节点的全局变量
volatile uint16_t status_word = 0x00;
volatile uint8_t go_zero_status_1 = 0;
volatile uint8_t go_zero_status_2 = 0;
volatile uint8_t drv_ready = 0;
volatile double angular_vel[3];
volatile double acceleration[3];
double angular[3];
ros::Publisher linemotor_state_pub;

void *Receive_Linemotor_Fcn(void* param);
void *Receive_Zero_Fcn(void* param);
void imu_feedback(sensor_msgs::Imu imu_data);
int8_t CanPort_Open(void);

int main(int argc, char** argv)
{
	//以下为发送电机指令的部分
    ros::init(argc, argv, "line_motor_comm_node");
    ros::NodeHandle nh;

	// 打开can通信端口
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
	// 初始化左右腿直线电机
	// LineMotor left_leg(1), right_leg(2);

	left_leg.DrvEnable();
	// right_leg.DrvEnable();
	// left_leg.DrvReset(); //复位
	// right_leg.DrvReset();

	// while((!go_zero_status_1) | (!go_zero_status_2))
	// {
	// 	left_leg.Gozero_Fdb();
	// 	usleep(1000);
	// 	right_leg.Gozero_Fdb();
	// 	usleep(1000);
	// }
	// sleep(4);
	// cout<<"原点已回归，可以开始运动!"<<endl;

	// GoZero_run1 = 0;
	// drv_ready = 1;
	// left_leg.DrvPosMode();
	// right_leg.DrvPosMode();
	// left_leg.NMT_Enable(1);
	// right_leg.NMT_Enable(2);

	PDO_run0 = 1;
	uint8_t count = 0;
    ros::Rate loop_rate(10); // 10Hz，周期为100ms
    while(ros::ok())
    {
		ROS_INFO("roll = %f pitch = %f yaw = %f",angular[0],angular[1],angular[2]);
		ROS_INFO("angular_vel_x = %f angular_vel_y = %f angular_vel_z = %f", angular_vel[0], angular_vel[1], angular_vel[2]);
		ROS_INFO("acceleration_x = %f acceleration_y = %f acceleration_z = %f", acceleration[0], acceleration[1],acceleration[2]);
		// if(count < 20)
		// {
		// 	left_leg.RelPos_Set(1,300);
		// 	count++;
		// }		
		// else
		// {

		// }

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

void* Receive_Zero_Fcn(void* param)
{
	VCI_CAN_OBJ rec[300];
	unsigned int reclen = 0;
	int *run = (int*)param;
	while((*run) & 0x0f)
	{
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, 0, rec, 300, 1000)>0)
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
	while((*run) & 0x0f)
	{
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100)>0)
		{
			for(int j = 0; j < reclen; j++)
			{
				if((rec[j].ID & 0x0581) == 0x0581) // sdo ID = 1
				{
					if(rec[j].Data[0] == 0x80)
					{
						ROS_ERROR_STREAM("Driver(ID=%d) is abnormal!"<<(rec[j].ID & 0x001));
					}
					else if(rec[j].Data[0] == 0x4b)// 查询status word位状态
					{
						//status_word = (rec[j].Data[] [j].Data[4]);
					}
				}

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