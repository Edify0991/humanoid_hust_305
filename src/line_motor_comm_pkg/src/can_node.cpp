#include "serialPort/SerialPort.h"
#include "ros/callback_queue.h"
// #include "line_motor_comm_pkg/motorMsgCmd.h"
// #include "line_motor_comm_pkg/motorMsgBack.h"
#include "line_motor_comm_pkg/linemotorMsgCmd.h"
#include "line_motor_comm_pkg/linemotorMsgBack.h"
#include "ros/ros.h"
#include <thread>
#include <vector>
#include "Modbus_Comm.h" // 直线电机

// 是否控制电机
const bool control_motor = true;


// 定义所有的电机的名字 第一个l/r表示左右腿，第二个l/r/f/b表示在腿上的位置
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

std::vector<std::thread> threads;
std::vector<ros::Publisher> linemotor_state_pub;
std::vector<LineMotor*> Linemotor_ports;

void Receive_Linemotor_Fcn(int CANIndex);
void Receive_Zero_Fcn(int CANIndex);
int8_t CanPort_Open(void);

volatile uint16_t status_word = 0x00;
volatile uint8_t go_zero_status_1 = 0;
volatile uint8_t go_zero_status_2 = 0;
volatile uint8_t drv_ready = 0;
volatile uint RTPos_left;//左腿绝对位置,1mm对应50编码器值
volatile uint RTPos_right;//右腿绝对位置

auto start_time = std::chrono::high_resolution_clock::now();
void LineMotorCmdCallback(const line_motor_comm_pkg::linemotorMsgCmd::ConstPtr& msg, int lineMotorID)
{
    float relPos, refSpeed;
    relPos = msg->x;
    refSpeed = msg->dx;
    
	std::cout << "lineMOtorID :   " << lineMotorID << std::endl;
	Linemotor_ports[lineMotorID]->Clear_PosCmd();
    if(control_motor) Linemotor_ports[lineMotorID]->RelPos_Set(relPos, refSpeed);

    if(lineMotorID==0)
    {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time);
        int duration_count = duration_us.count();
        // std::cout << "motor_: "<< duration_count << "\t pos: " << relPos <<std::endl;
        start_time = end_time;
    }
}

// 直线电机线程，用于接收电机的命令
void LineMotor_CmdThread(int lineMotorID, ros::NodeHandle nh)
{
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    ros::Subscriber sub = nh.subscribe<line_motor_comm_pkg::linemotorMsgCmd>(
            "linemotor_" + std::to_string(lineMotorID) + "_cmd", 
            1, 
            boost::bind(LineMotorCmdCallback, 
            _1, 
            lineMotorID));
    while (ros::ok())
    {
        queue.callAvailable(ros::WallDuration());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_node");
    ros::NodeHandle nh;

    // threads.resize(can_name_.size());
    threads.resize(2);
    // motor_state_pub.resize(motor_name_.size());
    linemotor_state_pub.resize(2);

    // 初始化直线电机
    // 打开can通信端口(can分析仪)
	uint8_t can_port_state = CanPort_Open();
	if(can_port_state == -1)
	{
		exit(-1);
	}


    int CANID_left = 1;
	int CANID_right = 0;

    linemotor_state_pub[0] = nh.advertise<line_motor_comm_pkg::linemotorMsgBack>("left_knee_state", 1);
    linemotor_state_pub[1] = nh.advertise<line_motor_comm_pkg::linemotorMsgBack>("right_knee_state", 1);

    // 创建接收汇川电机驱动器PDO数据的线程
    std::thread Rcv_PDO_thread_left(std::bind(Receive_Linemotor_Fcn, CANID_left));
    if (Rcv_PDO_thread_left.joinable()) {
        std::cout << "直线电机PDO接收进程创建成功" << std::endl;
        // 等待线程结束
        Rcv_PDO_thread_left.detach();
    } else {
        std::cerr << "直线电机PDO接收进程创建失败" << std::endl;
        // 处理错误
    }

	std::thread Rcv_PDO_thread_right(std::bind(Receive_Linemotor_Fcn, CANID_right));
    if (Rcv_PDO_thread_right.joinable()) {
        std::cout << "直线电机PDO接收进程创建成功" << std::endl;
        // 等待线程结束
        Rcv_PDO_thread_right.detach();
    } else {
        std::cerr << "直线电机PDO接收进程创建失败" << std::endl;
        // 处理错误
    }

	// 创建直线电机回零监测线程
    std::thread Rcv_zero_thread_left(std::bind(Receive_Zero_Fcn, CANID_left));
    if (Rcv_zero_thread_left.joinable()) {
        std::cout << "直线电机回零监测进程创建成功" << std::endl;
        // 等待线程结束
        Rcv_zero_thread_left.detach();
    } else {
        std::cerr << "直线电机回零监测进程创建失败" << std::endl;
        // 处理错误
    }

    std::thread Rcv_zero_thread_right(std::bind(Receive_Zero_Fcn, CANID_right));
    if (Rcv_zero_thread_right.joinable()) {
        std::cout << "直线电机回零监测进程创建成功" << std::endl;
        // 等待线程结束
        Rcv_zero_thread_right.detach();
    } else {
        std::cerr << "直线电机回零监测进程创建失败" << std::endl;
        // 处理错误
    }

    Linemotor_ports.push_back(new LineMotor(1, 1));
    Linemotor_ports.push_back(new LineMotor(2, 0));

	Linemotor_ports[0]->DrvEnable();
	Linemotor_ports[1]->DrvEnable();

	Linemotor_ports[0]->Clear_PosCmd();
	Linemotor_ports[1]->Clear_PosCmd();
	Linemotor_ports[0]->DrvReset(); //复位
	Linemotor_ports[1]->DrvReset();
	usleep(3000000);

	while((!go_zero_status_1) | (!go_zero_status_2))
	{
		Linemotor_ports[0]->Gozero_Fdb();
		Linemotor_ports[1]->Gozero_Fdb();
	}

    

    

    threads[0] = std::thread(LineMotor_CmdThread, 0, std::ref(nh)); 
    threads[1] = std::thread(LineMotor_CmdThread, 1, std::ref(nh)); 
 
    for(auto& thread : threads)
    {
        thread.detach();
    }
    while(ros::ok()){}
    return 0;
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

    line_motor_comm_pkg::linemotorMsgBack linemotor_ret_msg;
    
	while(1)
	{
		
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, CANIndex, rec, 3000, 100)>0)
		{
			// ROS_INFO("RECEIVED！ ");
			for(int j = 0; j < reclen; j++)
			{
				std::cout<<"ID:   " << std::hex<<(rec[j].ID)<<std::endl;
				if(rec[j].ID  == 0x181) // sdo ID = 1
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
					
                    linemotor_ret_msg.id = 0;
                    linemotor_ret_msg.x = RTPos_left / 50.0;
                    linemotor_state_pub[0].publish(linemotor_ret_msg);
                    std::cout<<"线程中的RTPos_left:= "<<std::endl;
					// cout<<"线程中的函数增量"<<(float)((1000-RTPos_left)/50.0)<<endl;
				}
				else if(rec[j].ID  == 0x0182)
				{
					RTPos_right = (((uint)rec[j].Data[5] << 24) | ((uint)rec[j].Data[4] << 16) | ((uint)rec[j].Data[3] << 8) | (uint)rec[j].Data[2]);
                    linemotor_ret_msg.id = 1;
                    linemotor_ret_msg.x = RTPos_right / 50.0;
                    linemotor_state_pub[1].publish(linemotor_ret_msg);
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