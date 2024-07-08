#include "Modbus_Comm.h"

using namespace std;

// 该节点的全局变量
ros::Publisher linemotor_state_pub;

void *Receive_Linemotor_Fcn(void* param);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_motor_comm_node");
    ros::NodeHandle nh("~");

	// CAN通信板载信息获取，后续待优化
    VCI_BOARD_INFO pInfo1 [50];
    VCI_BOARD_INFO pInfo;
    int MotorNum = VCI_FindUsbDevice2(pInfo1);
    if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)
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
        printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
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

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
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
		printf("%x", (pInfo.fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo.fw_Version&0xF0)>>4);
		printf("%x", pInfo.fw_Version&0xF);
		printf("\n");	
    }
    else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
    }

	// 创建接收汇川电机驱动器数据的线程
	//int m_run0 = 1;
	//pthread_t threadid;
	//int ret;
	//ret = pthread_create(&threadid, NULL, Receive_Linemotor_Fcn, &m_run0);

	// 初始化左右腿直线电机
	LineMotor left_leg(1);//, right_leg(2);
	// 测试效果
	left_leg.DrvDisable();
	left_leg.DrvEnable();
	left_leg.RelPos_Set(10, 4000);
	left_leg.Clear_PosCmd();
	left_leg.RelPos_Set(5, 4000);
	left_leg.Clear_PosCmd();
	left_leg.DrvDisable();

    ros::Rate loop_rate(100); // 100Hz，周期为10ms
    while(ros::ok())
    {
		ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void *Receive_Linemotor_Fcn(void* param)
{
	int reclen = 0;
	VCI_CAN_OBJ rec[3000];
	int *run = (int*)param;
	while((*run)&0x0f)
	{
		if(reclen=VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100)>0)
		{
			for(int j=0;j<reclen;j++)
			{
				if(rec[j].ID)
				{
					
				}
			}
		}
	}
}