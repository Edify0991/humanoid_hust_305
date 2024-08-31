#include <list>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include "Modbus_Comm.h"

VCI_CAN_OBJ axis_send[1];
struct data_conversion
{
	int store_16[4];
	double store10;
};

struct motor_control_parameter
{
	int drive_id;
	double displacement;
	double speed;
	double acceleration;
	int mode;//0ͣ停止、1相对运动（运行完轨迹停止后执行）、2绝对运动（运行完轨迹后停止）、3相对运动（完成轨迹但未停止立即执行）、
	//4绝对运动（完成轨迹但未停止立即执行）、5相对运动（立即执行）、6绝对运动（立即执行）
};

//8位数组拷贝
void array_copy(unsigned char send[], int init[8]) {
	for (int i = 0; i < 8; i++) {
		send[i] = init[i];
	}
}

//16进制与10进制转换
data_conversion Base_conversion(data_conversion D, int M)
{
	data_conversion data = {  };
	if (D.store10 < 0 && M == 0)
	{
		int a = 256 * 256 * 256 - 1;
		int b = a + D.store10;
		a = 256;
		data.store_16[0] = b % a;
		b = b / a;
		data.store_16[1] = b % a;
		b = b / a;
		data.store_16[2] = b % a;
		data.store_16[3] = 255;
	}
	if (D.store10 >= 0 && M == 0)
	{
		int a = 256;
		int b = D.store10;
		data.store_16[0] = b % a;
		b = b / a;
		data.store_16[1] = b % a;
		b = b / a;
		data.store_16[2] = b % a;
		data.store_16[3] = 0;
	}
	if (D.store_16[3] == 0 && M == 1)
	{
		data.store10 = D.store_16[0] + 256 * D.store_16[1] + 256 * 256 * D.store_16[2];
	}
	if (D.store_16[3] == 255 && M == 1)
	{
		data.store10 = (255 - D.store_16[0]) + 256 * (255 - D.store_16[1]) + 256 * 256 * (255 - D.store_16[2]);
		data.store10 = -data.store10;
	}
	return data;
};
data_conversion data_input,data_output;
	//电机单独使能
void motor_enable(motor_control_parameter M){
	axis_send[0].ID=M.drive_id;
	axis_send[0].SendType = 1;
	axis_send[0].RemoteFlag = 0;
	axis_send[0].ExternFlag = 0;
	axis_send[0].DataLen = 8;
	int enable[3][8] = { {0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00},{0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00},
		{0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00}};
	for (int i = 0; i < 3; i++) {
	array_copy(axis_send[0].Data, enable[i]);
	VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
	usleep(3000);
	}
    };
	//电机位置控制模式准备
void motor_ready(motor_control_parameter M){
	axis_send[0].ID=M.drive_id;
	axis_send[0].SendType = 1;
	axis_send[0].RemoteFlag = 0;
	axis_send[0].ExternFlag = 0;
	axis_send[0].DataLen = 8;
	//停止回原点，修改为轮廓运动模式
	int motor_mode[2][8] = { {0x2B, 0x40, 0x60, 0x00, 0x1F, 01, 00, 00 }, { 0x2F, 0x60, 0x60, 0x00, 0x01, 00, 00, 00 } };
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			array_copy(axis_send[0].Data, motor_mode[j]);
			VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
			usleep(3000);
		}
	}
	//初始化电机运行速度和加速度
	int par_init[3][8] = { {0x23,0x81,0x60,0x00,0x20,0x4E,00,00},{0x23,0x83,0x60,0x00,0x40,0x9c,00,00},
		{0x23,0x84,0x60,0x00,0x40,0x9c,00,00} };//初始化电机的运行速度为20000 step/s和加减速为40000 step/s2
	for (int j = 0; j < 3; j++) {
		array_copy(axis_send[0].Data, par_init[j]);
		VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
		usleep(3000);
	}
};
	//电机立即停止
void motor_stop(motor_control_parameter M){
	axis_send[0].SendType = 1;
	axis_send[0].RemoteFlag = 0;
	axis_send[0].ExternFlag = 0;
	axis_send[0].DataLen = 8;
	axis_send[0].ID=M.drive_id;
	int stop_instruct[8]={0x2b,0x40,0x60,00,0x0f,0x01,00,00};
	array_copy(axis_send[0].Data, stop_instruct);
	VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
	usleep(3000);
};
	//单个电机驱动
void single_motor_drive(motor_control_parameter M){
	axis_send[0].SendType = 1;
	axis_send[0].RemoteFlag = 0;
	axis_send[0].ExternFlag = 0;
	axis_send[0].DataLen = 8;
	axis_send[0].ID=M.drive_id;
	int motor_para[4][8] = {{0x23,0x81,0x60,0,0,0,0,0},{0x23,0x7A,0x60,0,0,0,0,0},
	{0x23,0x83,0x60,0,0x40,0x9c,0,0},{0x23,0x84,0x60,0,0x40,0x9c,0,0}};
	//速度输入
	data_input.store10=M.speed;
	data_output=Base_conversion(data_input,0);
	for(int i=0;i<4;i++){
		motor_para[0][4+i]=data_output.store_16[i];
	}
	//位置输入
	data_input.store10=M.displacement;
	data_output=Base_conversion(data_input,0);
	for(int i=0;i<4;i++){
		motor_para[1][4+i]=data_output.store_16[i];
	}
	//加速度输入
	data_input.store10=M.acceleration;
	data_output=Base_conversion(data_input,0);
	for (int i = 0; i < 4; i++) {
		motor_para[2][4+i]=data_output.store_16[i];
		motor_para[3][4+i]=data_output.store_16[i];
	}
	for (int i = 0; i < 4; i++) {
		array_copy(axis_send[0].Data, motor_para[i]);
		VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
		usleep(3000);
	}
	switch (M.mode)
	{
	case 0:{
		int motor_lauch0[8] = { 0x2b,0x40,0x60,0,0x0f,0x01,0,0 };
		array_copy(axis_send[0].Data, motor_lauch0);
		VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
		break;
	}
	case 1:{
		int motor_lauch1[2][8] = { { 0x2b,0x40,0x60,0,0x5f,0,0,0 },{ 0x2b,0x40,0x60,0,0x4f,0,0,0 } };
		for (int j = 0; j < 2; j++)
			{
				array_copy(axis_send[0].Data, motor_lauch1[j]);
				VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
				usleep(3000);
			}
			break;
	}
	case 2:{
		int motor_lauch1[2][8] = { { 0x2b,0x40,0x60,0,0x1f,0,0,0 },{ 0x2b,0x40,0x60,0,0x0f,0,0,0 } };
		for (int j = 0; j < 2; j++)
		{
			array_copy(axis_send[0].Data, motor_lauch1[j]);
			VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
			usleep(3000);
		}
		break;
	}
	case 3:{
		int motor_lauch1[2][8] = {{ 0x2b,0x40,0x60,0,0x5f,2,0,0 },{ 0x2b,0x40,0x60,0,0x4f,2,0,0 } };
		for (int j = 0; j < 2; j++)
		{
			array_copy(axis_send[0].Data, motor_lauch1[j]);
			VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
			usleep(3000);
		}
		break;
	}
	case 4:{
		int motor_lauch1[2][8] = { { 0x2b,0x40,0x60,0,0x1f,2,0,0 },{ 0x2b,0x40,0x60,0,0x0f,2,0,0 } };
		for (int j = 0; j < 2; j++)
		{
			array_copy(axis_send[0].Data, motor_lauch1[j]);
			VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
			usleep(3000);
		}
		break;
	}
	case 5:{
		int motor_lauch1[2][8] = { { 0x2b,0x40,0x60,0,0x7f,2,0,0 },{ 0x2b,0x40,0x60,0,0x6f,2,0,0 } };
		for (int j = 0; j < 2; j++)
		{
			array_copy(axis_send[0].Data, motor_lauch1[j]);
			VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
			usleep(3000);
		}
		break;
	}
	case 6:{
		int motor_lauch1[2][8] = { { 0x2b,0x40,0x60,0,0x3f,2,0,0 },{ 0x2b,0x40,0x60,0,0x2f,2,0,0 } };
			for (int j = 0; j < 2; j++)
			{
				array_copy(axis_send[0].Data, motor_lauch1[j]);
				VCI_Transmit(VCI_USBCAN2, 0, 0, axis_send, 1);
				usleep(3000);
			}
		break;
	}
}
};


VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
motor_control_parameter parall_3_motor;
motor_control_parameter parall_4_motor;
motor_control_parameter parall_5_motor;
motor_control_parameter parall_6_motor;
motor_control_parameter parall_7_motor;
motor_control_parameter parall_8_motor;
int num;
VCI_BOARD_INFO pInfo1 [50];

int main(int argc,char **argv) 
{
    ros::init(argc, argv, "motor_state_publisher");
    ros::NodeHandle nh;
	ros::Time::init();
	ros::Rate loop_late(10);
	printf(">>procedure running !\r\n");//指示程序已运行
	num=VCI_FindUsbDevice2(pInfo1);
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		//printf(" %08X", pInfo.hw_Version);printf("\n");
		//printf(" %08X", pInfo.fw_Version);printf("\n");
		//printf(" %08X", pInfo.dr_Version);printf("\n");
		//printf(" %08X", pInfo.in_Version);printf("\n");
		//printf(" %08X", pInfo.irq_Num);printf("\n");
		//printf(" %08X", pInfo.can_Num);printf("\n");
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
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	//初始化参数，严格参考二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率500
	 Kbps  0x00  0x1C*/
	config.Timing1=0x14;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	
	//需要发送的帧，结构体设置
	parall_3_motor.drive_id=0x603;
	parall_4_motor.drive_id=0x604;
	parall_5_motor.drive_id=0x605;
    parall_6_motor.drive_id=0x606;
	parall_7_motor.drive_id=0x607;
	parall_8_motor.drive_id=0x608;

	//开始五项电机回原点
	//直线电机速度定义1mm/s对应5000step/s
	//xy旋转电机速度定义1mm/s对应800step/s
	parall_3_motor.speed=parall_4_motor.speed=parall_5_motor.speed=parall_6_motor.speed=parall_7_motor.speed=parall_8_motor.speed=1*5000;
	motor_enable(parall_3_motor);
	motor_enable(parall_4_motor);
	motor_enable(parall_5_motor);
	motor_enable(parall_6_motor);
	motor_enable(parall_7_motor);
	motor_enable(parall_8_motor);
	
	int m_run0=1;
	pthread_t threadid;
	//设备已完成复位,系统进入待机状态；
	//各电机驱动初始化至位置控制模式并使能
	motor_ready(parall_3_motor);
	motor_ready(parall_4_motor);
    motor_ready(parall_5_motor);
	motor_ready(parall_6_motor);
	motor_ready(parall_7_motor);
	motor_ready(parall_8_motor);
	
	
	parall_3_motor.speed=parall_4_motor.speed=parall_5_motor.speed=parall_6_motor.speed=parall_7_motor.speed=parall_8_motor.speed=15*5000;
	parall_3_motor.acceleration=parall_4_motor.acceleration=parall_5_motor.acceleration=parall_6_motor.acceleration=parall_7_motor.acceleration=parall_8_motor.acceleration=50*5000;
	parall_3_motor.mode=parall_4_motor.mode=parall_5_motor.mode=parall_6_motor.mode=parall_7_motor.mode=parall_8_motor.mode=4;
	int action=1;
	action=1;
	sleep(1);
	double pos_abs1 = 5;
	double pos_abs2 = 10;
	double pos_abs3 = 10;
	while (action)
	{
	std::cout<<"Do you want to continue ?"<<std::endl;
    std::cin>>action;
     /*std::cout<<"Type absolute position information 1 and enter"<<std::endl;
     double pos_abs1;
     std::cin>>pos_abs1;
	 std::cout<<"Type absolute position information 2 and enter"<<std::endl;
     double pos_abs2;
     std::cin>>pos_abs2;
	 std::cout<<"Type absolute position information 3 and enter"<<std::endl;
     double pos_abs3;
     std::cin>>pos_abs3;
     std::cout<<"Do you want to continue ?"<<std::endl;
     std::cin>>action;*/
	parall_6_motor.displacement=pos_abs1*5000;
	single_motor_drive(parall_6_motor);
	parall_7_motor.displacement=pos_abs2*5000;
	single_motor_drive(parall_7_motor);
	parall_8_motor.displacement=pos_abs3*5000;
	single_motor_drive(parall_8_motor);
	}
	bool restart=1;
	while (restart)
	{
	std::cout<<"start ?"<<std::endl;
    std::cin>>restart;
	double a=10,b=20,c=30,a1=1,b1=1,c1=1;
	parall_3_motor.speed=parall_4_motor.speed=parall_5_motor.speed=parall_6_motor.speed=parall_7_motor.speed=parall_8_motor.speed=15*5000;
	parall_6_motor.displacement=a*5000;
	single_motor_drive(parall_6_motor);
	parall_7_motor.displacement=b*5000;
	single_motor_drive(parall_7_motor);
	parall_8_motor.displacement=c*5000;
	single_motor_drive(parall_8_motor);
	sleep(3);
	for(int i=0;i<100;i++){
	parall_3_motor.speed=parall_4_motor.speed=parall_5_motor.speed=parall_6_motor.speed=parall_7_motor.speed=parall_8_motor.speed=5*5000;
	a=a+0.5*a1;
	b=b+0.5*b1;
	c=c+0.5*c1;
	if(a>=31||a<=9){
		a1=-1*a1;
	}
	if(b>=31||b<=9){
		b1=-1*b1;
	}
	if(c>=31||c<=9){
		c1=-1*c1;
	}
	parall_6_motor.displacement=a*5000;
	single_motor_drive(parall_6_motor);
	parall_7_motor.displacement=b*5000;
	single_motor_drive(parall_7_motor);
	parall_8_motor.displacement=c*5000;
	single_motor_drive(parall_8_motor);
	usleep(50000);
	}
	parall_6_motor.displacement=0;
	single_motor_drive(parall_6_motor);
	parall_7_motor.displacement=0;
	single_motor_drive(parall_7_motor);
	parall_8_motor.displacement=0;
	single_motor_drive(parall_8_motor);
	usleep(10000);
	}
	
	/*
	axisx_motor.mode=4;
	axisy_motor.mode=4;
	axisx_motor.acceleration=5000;
	axisy_motor.acceleration=5000;
	axisx_motor.speed=10*4096/75;
	axisy_motor.speed=15*4096/75;
	axisx_motor.displacement=123*4096/75;
	axisy_motor.displacement=295*2*4096/75;
	step_motor_control::single_motor_drive(axisx_motor);
	step_motor_control::single_motor_drive(axisy_motor);
	sleep(40);
	std::cout<<"daowei"<<std::endl;
	parall_1_motor.displacement=38.5*5000;
	parall_2_motor.displacement=37*5000;
	parall_3_motor.displacement=38.5*5000;
	step_motor_control::single_motor_drive(parall_1_motor);
	step_motor_control::single_motor_drive(parall_2_motor);
	step_motor_control::single_motor_drive(parall_3_motor);
	sleep(10);
	axisx_motor.speed=100/60*4096/75;
	axisy_motor.speed=200/60*4096/75;
	double c=0;
	for(int i=0;i<100;i++){
	std::cout<<c<<std::endl;
	c=(123+i*0.05)*4096/75;
	axisx_motor.displacement=c;
	step_motor_control::single_motor_drive(axisx_motor);
	usleep(400000);
	}
	std::cout<<"chufa"<<std::endl;
	double a,b=0;
	for(int i=0;i<1000;i++)
	{
		a=(128+i*0.0385)*4096/75;
		b=(295-i*0.04)*2*4096/75;
	   axisx_motor.displacement=a;
	   axisy_motor.displacement=b;
	   step_motor_control::single_motor_drive(axisx_motor);
	   step_motor_control::single_motor_drive(axisy_motor);
	   std::cout<<axisx_motor.displacement<<"  "<<axisy_motor.displacement<<std::endl;
	   usleep(300000);
	}
	parall_1_motor.displacement=parall_2_motor.displacement=parall_3_motor.displacement=0;
	step_motor_control::single_motor_drive(parall_1_motor);
	step_motor_control::single_motor_drive(parall_2_motor);
	step_motor_control::single_motor_drive(parall_3_motor);
	*/
	return 0;
}