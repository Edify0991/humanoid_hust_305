/*
 * 文件名：ModbusCommDrv.cpp
 * 作者：卢洪磊
 * 日期：2024-6-26
 * 文件描述：该文件产生了一个直线电机类，用于配置直线电机的相关数据，对应驱动器为汇川SV660C
 *          需说明的是PDO的映射没有写在该类里面，需在上位机界面手动实现映射
 * Version：1.0
 * 第一次修改：将汇川驱动器修改为SV660C，2024年6月28日
 *           完成驱动器使能、失能、SDO配置PDO、PDO配置轮廓位置模式
 *           待加入回原点功能和接收线程
 * 第二次修改：2024年7月10日，行程开关硬件调试完成
 *           完成驱动器反向行程复位操作和接收线程，完成B1电机与膝关节电机集成
 *           后续需增加新的功能包实现运动控制
 */
#include <iostream>
#include <string.h>
#include "Modbus_Comm.h"
#include <unitreeMotor/unitreeMotor.h>

using namespace std;
/*
 * 函数名称：LineMotor
 * 函数描述：直线电机类的构造函数,使用SDO方法建立PDO映射
 * 函数输入：addr - 驱动器在总线上的地址
 * 函数返回：None
 */
LineMotor::LineMotor(uint8_t addr, int CANIndex): addr_BUS(addr), CANIndex(CANIndex)
{
    // addr_BUS = addr;

    //开启CAN通信
    uint8_t open_signal = DRIVER_OpenCanComm();
    if(open_signal == 0)
    {
        ROS_INFO_STREAM(">>开启CAN通信失败!");
        exit(1);
    }

    Sdo_COB_ID = 0x600 + addr_BUS;      // SDO，主站发送（0x600）

    TPdo1_COB_ID = 0x180 + addr_BUS;       // 启用TPDO1
    uint8_t Tx_COB_ID_H = (uint8_t)(TPdo1_COB_ID >> 8) & 0x00ff;
    uint8_t Tx_COB_ID_L = (uint8_t)(TPdo1_COB_ID & 0x00ff);

    RPdo1_COB_ID = 0x200 + addr_BUS;       // 启用RPDO1
    uint8_t Rx_COB_ID_H = (uint8_t)(RPdo1_COB_ID >> 8) & 0x00ff;
    uint8_t Rx_COB_ID_L = (uint8_t)(RPdo1_COB_ID & 0x00ff);

    RPdo2_COB_ID = 0x300 + addr_BUS;       // 启用RPDO2
    uint8_t Rx2_COB_ID_H = (uint8_t)(RPdo2_COB_ID >> 8) & 0x00ff;
    uint8_t Rx2_COB_ID_L = (uint8_t)(RPdo2_COB_ID & 0x00ff);

    sendTodriver_msg.SendType = 1;      // 发送类型为单次发送
    sendTodriver_msg.RemoteFlag = 0;    // 远程帧不开启
    sendTodriver_msg.ExternFlag = 0;    // 扩展帧不开启
    sendTodriver_msg.DataLen = 8;       // 数据长度为8个字节
    sendTodriver_msg.ID = Sdo_COB_ID;
    /*
    * 1. PDO1的Communication Parameters(0x1400)中COB-ID(01)，MSB写1，失能PDO1(最高位，即0x80的最高位)
    * 2. PDO1的Communication Parameters(0x1400)中transmission type(02)，写1，同步
    * 3. PDO1的Mapping Object(0x1600)中sub-index(0x00)，先写0，清空原先PDO映射
    * 4. PDO1的Mapping Object(0x1600)中sub-index(0x01~0x0n)，建立n个映射
    * 5. PDO1的Mapping Object(0x1600)中sub-index(0x00)，写n，指示映射变量的个数
    */
    // 配置RPDO1，流程如下
    // sub-index = 0x01:目标位置
    // sub-index = 0x02:轮廓速度
    uint8_t temp_data[7][8] = { {0x23, 0x00, 0x14, 0x01, Rx_COB_ID_L, Rx_COB_ID_H, 0x00, 0x80},        // 0x23:Data为4byte 0x1400-01h:RPDO1的COB-ID 0x80:最高为为1
                                {0x23, 0x00, 0x14, 0x02, 0xFE, 0x00, 0x00, 0x00},                      // 0x1400-02h:RPDO1的传输类型 0xFE:将接收到的数据直接更新到应用
                                {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},                      // 0x2f:Data为1byte 0x1600-00h:RPDO1有效映射对象个数，写入“0”表示清除原有映射内容
                                {0x23, 0x00, 0x16, 0x01, 0x20, 0x00, 0x7A, 0x60},                      // 0x1600-01h:RPDO1映射对象1 0x607Ah:设置轮廓位置模式下目标位置(与0x6040h的设置相关-增量位移 绝对位置)
                                {0x23, 0x00, 0x16, 0x02, 0x20, 0x00, 0x81, 0x60},                      // 0x1600-02h:RPDO1映射对象2 0x6081h:设置轮廓位置模式下轮廓匀速运行速度
                                {0x2F, 0x00, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},                      // 0x1600-00h:RPDO1有效映射对象个数，该PDO映射对象总个数为2
                                {0x23, 0x00, 0x14, 0x01, Rx_COB_ID_L, Rx_COB_ID_H, 0x00, 0x00},        // 0x00:最高为为0
                                };
    for(int i = 0; i < 7; i++)
    {
        array_copy(sendTodriver_msg.Data, temp_data[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }

    // 配置RPDO2，流程如下
    // sub-index = 0x01:伺服模式选择
    // sub-index = 0x02:控制字
    uint8_t temp_data_5[7][8] = {{0x23, 0x01, 0x14, 0x01, Rx2_COB_ID_L, Rx2_COB_ID_H, 0x00, 0x80},     
                                {0x23, 0x01, 0x14, 0x02, 0xFE, 0x00, 0x00, 0x00}, 
                                {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                                {0x23, 0x01, 0x16, 0x01, 0x08, 0x00, 0x60, 0x60},                      // 0x6060h:伺服模式选择
                                {0x23, 0x01, 0x16, 0x02, 0x10, 0x00, 0x40, 0x60},                      // 0x6040h:控制字
                                {0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00}, 
                                {0x23, 0x01, 0x14, 0x01, Rx2_COB_ID_L, Rx2_COB_ID_H, 0x00, 0x00},
                                };
    for(int i = 0; i < 7; i++)
    {
        array_copy(sendTodriver_msg.Data, temp_data_5[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }

    // 配置TPDO1
    // sub-index = 0x01:状态字
    // sub-index = 0x02:反馈位置
    // PDO事件定时上报，周期时间50ms，静止时间为0
    uint8_t temp_data_2[9][8] = {{0x23, 0x00, 0x18, 0x01, Tx_COB_ID_L, Tx_COB_ID_H, 0x00, 0x80},       // 0x1800-01h:TPDO1的COB-ID
                                {0x2F, 0x00, 0x18, 0x02, 0xff, 0x00, 0x00, 0x00},                      // 0xff:映射数据发生改变或者事件计时器到达则发送该TPDO
                                {0x2B, 0x00, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00},                      // 0x2B:Data为2Byte 0x1800-03h:设置TPDO静止时间，单位为100us，同一个TPDO传输时间间隔不得小于该参数对应的禁止时间
                                {0x2B, 0x00, 0x18, 0x05, 0x02, 0x00, 0x00, 0x00},                      // 0x1800-05h:定义事件计时器，单位ms，这里为50ms
                                {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},                      // 0x1A00h:TPDO1映射对象
                                {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60},                      // 0x6041h:状态字
                                {0x23, 0x00, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60},                      // 0x6064h:位置反馈，反映实时用户绝对位置反馈(指令单位)，用户位置反馈(6064h )× 齿轮比(6091h) = 电机位置反馈(6063h)
                                {0x2F, 0x00, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00}, 
                                {0x23, 0x00, 0x18, 0x01, Tx_COB_ID_L, Tx_COB_ID_H, 0x00, 0x00}, 
                                };
    sendTodriver_msg.ID = Sdo_COB_ID;
    for(int i = 0; i < 9; i++)
    {
        array_copy(sendTodriver_msg.Data, temp_data_2[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }

    // 设置电子齿轮比
    // 分子：8388608
    // 分母：100
    /* 直线电机指令单位与伸出轴位移的转换关系 */
    /*
    * 直线电机的重复定位精度M为0.02mm，因此令一个指令单位可移动0.02mm
    * 减速比R=1；编码器分辨率Rg=8388608(23 bits)；导程Pg=2mm
    * 电子齿轮比B/A=M*1/R*Pg=8388608/100；
    */
    uint8_t GearRation[2][8] = {{0x23,0x91,0x60,0x01,0x00,0x00,0x80,0x00},\
                                {0x23,0x91,0x60,0x02,0x64,0x00,0x00,0x00}};
    sendTodriver_msg.ID = Sdo_COB_ID;
    for(int i = 0; i < 2; i++)
    {
        array_copy(sendTodriver_msg.Data, GearRation[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }
    //配置轮廓加速度和减速度
    // 额定速度下是0x57e4,但进行小段位移时加减速度过于大，易振荡
    uint8_t Acc_Dec[2][8] = {{0x23,0x83,0x60,0x00,0xe4,0x30,0x00,0x00},\
                              {0x23,0x84,0x60,0x00,0xe4,0x30,0x00,0x00}};//加速度为30e4，由两位地址决定
    sendTodriver_msg.ID = Sdo_COB_ID;
    for(int i = 0; i < 2; i++)
    {
        array_copy(sendTodriver_msg.Data, Acc_Dec[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }
    // 设置回零快速度和慢速度
    uint8_t Zero_Acc_Dec[2][8] ={{0x23,0x99,0x60,0x01,0xd3,0x01,0x00,0x00},\
                              {0x23,0x99,0x60,0x02,0xd3,0x00,0x00,0x00},\
                              };
    sendTodriver_msg.ID = Sdo_COB_ID;
    for(int i = 0; i < 2; i++)
    {
        array_copy(sendTodriver_msg.Data, Zero_Acc_Dec[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }
    // 开启NMT 操作模式
    NMT_Enable(addr);
}
/*
 * 函数名称：DRIVER_OpenCanComm
 * 函数描述：开启PC端与驱动器端的can通信
 * 函数输入：None
 * 函数返回：can通信状态
 *          0 - 使能失败
 *          1 - 使能成功
 */
int8_t LineMotor::DRIVER_OpenCanComm(void)
{
    motor_can_config.AccCode = 0; 
    motor_can_config.AccMask = 0xffffffff;                          // 接收所有ID的CAN信息帧
    motor_can_config.Filter = 1;                                    // 接受所有类型帧（标准帧+扩展帧）
    motor_can_config.Timing0 = 0x00;                                // Timing0 = 0x00, Timing1 = 0x1C
    motor_can_config.Timing1 = 0x14;                                // 1M bps 波特率
    motor_can_config.Mode = 0;                                      // 正常收发模式
    if(VCI_InitCAN(VCI_USBCAN2, 0, CANIndex, &motor_can_config) != 1)
    {
        ROS_ERROR_STREAM(">>CAN initialization failed!");
        return 0;
    }
    if(VCI_StartCAN(VCI_USBCAN2, 0, CANIndex)!=1)
    {
        ROS_ERROR_STREAM(">>Starting CAN failed!");
        return 0;
    }
    return 1;
}
/*
 * 函数名称：QuickStop
 * 函数描述：驱动器选用自由停机方式停机
 * 函数输入：None
 * 函数返回：None
 */
void LineMotor::QuickStop(void)
{
    uint8_t temp_data[2][8] = {{0x2b,0x5A,0x60,0x00,0x00,0x00,0x00,0x00},\
                                {0x2b,0x40,0x60,0x00,0x02,0x00,0x00,0x00},\
                                };
    sendTodriver_msg.ID = Sdo_COB_ID;
    for(int i = 0; i < 2; i++)
    {
        array_copy(sendTodriver_msg.Data, temp_data[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }
}
/*
 * 函数名称：DrvEnable
 * 函数描述：驱动器由伺服无故障状态切换至伺服运行状态
 * 函数输入：None
 * 函数返回：None
 */
void LineMotor::DrvEnable(void)
{
    uint8_t temp_data[3][8] = { {0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00},\
                                {0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00},\
                                {0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00},\
                                };
    sendTodriver_msg.ID = Sdo_COB_ID;
    std::cout << "sdo id" << std::endl;
    for(int i = 0; i < 3; i++)
    {
        
        array_copy(sendTodriver_msg.Data, temp_data[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }
}
/*
 * 函数名称：DrvDisable
 * 函数描述：驱动器失能
 * 函数输入：None
 * 函数返回：None
 */
void LineMotor::DrvDisable(void)
{
    uint8_t temp_data[2][8] = {{0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00},\
                                {0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00}
                                };
    sendTodriver_msg.ID = Sdo_COB_ID;
    for(int i = 0; i < 2; i++)
    {
        array_copy(sendTodriver_msg.Data, temp_data[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(10000);
    }
}
void LineMotor::DrvReset(void)
{
    // 设置回零方式和原点回归
    // 反向限位开关回零
    // 回零加速度
    uint8_t temp_data[3][8] = {{0x2F,0x60,0x60,0x00,0x06,0x00,0x00,0x00},\
                                {0x2F,0x98,0x60,0x00,0x01,0x00,0x00,0x00},\
                                {0x23,0x9A,0x60,0x00,0xd3,0x06,0x00,0x00}};
    sendTodriver_msg.ID = Sdo_COB_ID;
    for(int i = 0; i < 3; i++)
    {
        array_copy(sendTodriver_msg.Data, temp_data[i], 8);
        VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
        usleep(80000);
    }
    // 设置控制字 
    uint8_t temp_data_3[8] = {0x2b,0x40,0x60,0x00,0x7f,0x00,0x00,0x00};
    array_copy(sendTodriver_msg.Data, temp_data_3, 8);
    VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
    usleep(80000);
}
/*
 * 函数名称：AbsPos_Set
 * 函数描述：以绝对式位置控制方式控制电机运动
 * 函数输入：None
 * 函数返回：None
 */
void LineMotor::AbsPos_Set(float absPos)
{
}
/*
 * 函数名称：RelPos_Set
 * 函数描述：以相对位移控制方式控制电机运动
 * 函数输入：relPos - 直线电机伸缩距离(mm)
 *         refSpeed - 点对点运动时的参考速度(mm/s)
 * 函数返回：None
 */
void LineMotor::RelPos_Set(float relPos, uint32_t refSpeed)
{
    
    Clear_PosCmd();
    // 需要注意的是这里使用类的变量无法发送到驱动器上
    VCI_CAN_OBJ sendtoDrv;
    VCI_CAN_OBJ sendtoDrv2;

    sendtoDrv.SendType = 1;      // 发送类型为单次发送
    sendtoDrv.RemoteFlag = 0;    // 远程帧不开启
    sendtoDrv.ExternFlag = 0;    // 扩展帧不开启
    sendtoDrv.DataLen = 8;       // 数据长度为8个字节

    sendtoDrv2.SendType = 1;      // 发送类型为单次发送
    sendtoDrv2.RemoteFlag = 0;    // 远程帧不开启
    sendtoDrv2.ExternFlag = 0;    // 扩展帧不开启
    sendtoDrv2.DataLen = 3;       // 控制字数据长度为3个字节

    int32_t drvPos = fPos_to_iPos_Fcn(relPos);
    uint32_t drvSpeed = realSpeed_to_drvSpeed(refSpeed);

    uint8_t drvPos_8 = (drvPos & 0x000000ff);
    uint8_t drvPos_16 = (((uint32_t)drvPos>>8) & 0x000000ff);
    uint8_t drvPos_24 = (((uint32_t)drvPos>>16) & 0x000000ff);
    uint8_t drvPos_32 = (((uint32_t)drvPos>>24) & 0x000000ff);

    uint8_t drvSpeed_8 = (drvSpeed & 0x000000ff);
    uint8_t drvSpeed_16 = (((uint32_t)drvSpeed>>8) & 0x000000ff);
    uint8_t drvSpeed_24 = (((uint32_t)drvSpeed>>16) & 0x000000ff);
    uint8_t drvSpeed_32 = (((uint32_t)drvSpeed>>24) & 0x000000ff);

    uint8_t temp_data_1[8] = {drvPos_8,drvPos_16,drvPos_24,drvPos_32,\
                              drvSpeed_8,drvSpeed_16,drvSpeed_24,drvSpeed_32\
                             };
    uint8_t temp_data_2[3] = {0x01,0x3f,0x00};  // 0x01,0x5f,0x00
    // 先设置轮廓速度和目标位置
    sendtoDrv.ID = RPdo1_COB_ID;
    array_copy(sendtoDrv.Data, &temp_data_1[0], 8);
    VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendtoDrv, 1);
    // usleep(100); // 1ms
    // 再设置控制字和控制模式
    sendtoDrv2.ID = RPdo2_COB_ID;
    array_copy(sendtoDrv2.Data, &temp_data_2[0], 3);
    VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendtoDrv2, 1);
    // usleep(2000); // 1ms
    // cout<<"已发送，控制量为： "<<relPos<<endl;
}
void LineMotor::Clear_PosCmd(void)
{
    uint8_t cmd[8] = {0x2b,0x40,0x60,0x00,0x6f,0x00,0x00,0x00};//{0x2b,0x40,0x60,0x00,0x6f,0x00,0x00,0x00}

    sendTodriver_msg.ID = Sdo_COB_ID;

    array_copy(sendTodriver_msg.Data, &cmd[0], 8);
    VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
    // usleep(2000);
}

void LineMotor::DrvPosMode(void)
{
    VCI_CAN_OBJ sendtoDrv2;
    sendtoDrv2.SendType = 1;      // 发送类型为单次发送
    sendtoDrv2.RemoteFlag = 0;    // 远程帧不开启
    sendtoDrv2.ExternFlag = 0;    // 扩展帧不开启
    sendtoDrv2.DataLen = 3;       // 控制字数据长度为3个字节
    // 再设置控制字和控制模式
    sendtoDrv2.ID = RPdo2_COB_ID;
    uint8_t temp_data_2[3] = {0x01,0x2f,0x00};      // 0x01,0x0f,0x00
    array_copy(sendtoDrv2.Data, &temp_data_2[0], 3);
    VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendtoDrv2, 1);
    usleep(2000); // 1ms
}
/*
 * 函数名称：Gozero_Fdb
 * 函数描述：获取回零点状态
 * 函数输入：None
 * 函数返回：None
 */
void LineMotor::Gozero_Fdb(void)
{
    uint8_t cmd[8] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00};
    sendTodriver_msg.ID = Sdo_COB_ID;
    array_copy(sendTodriver_msg.Data, &cmd[0], 8);
    VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendTodriver_msg, 1);
    usleep(1000);
}
/*
 * 函数名称：RelPos_Set
 * 函数描述：以相对位移控制方式控制电机运动
 * 函数输入：relPos - 直线电机伸缩距离(mm)
 *         refSpeed - 点对点运动时的参考速度(mm/s)
 * 函数返回：None
 */
void LineMotor::NMT_Enable(uint8_t addr)
{
    VCI_CAN_OBJ sendtoDrv;

    sendtoDrv.SendType = 1;      // 发送类型为单次发送
    sendtoDrv.RemoteFlag = 0;    // 远程帧不开启
    sendtoDrv.ExternFlag = 0;    // 扩展帧不开启
    sendtoDrv.DataLen = 2;       // 数据长度为8个字节
    sendtoDrv.ID = 0x000;

    uint8_t data[2] = {0x01,addr};
    array_copy(sendtoDrv.Data, &data[0], 2);
    VCI_Transmit(VCI_USBCAN2, 0, CANIndex, &sendtoDrv, 1);
    usleep(10000);
}
/*
 * 函数名称：fPos_to_iPos_Fcn
 * 函数描述：驱动器浮点数位移量转为供驱动器使用的整型位置指令
 * 函数输入：target_position - 目标位置(mm)
 * 函数返回：返回供驱动器使用的32位整型变量
 */
int32_t LineMotor::fPos_to_iPos_Fcn(float target_position)
{
    int32_t position_Q32 = target_position / INSTRUCTION_UNIT;
    if(position_Q32 > MAX_TARGET_POSITION)
    {
        position_Q32 = MAX_TARGET_POSITION;
    }
    else if(position_Q32 < MIN_TARGET_POSITION)
    {
        position_Q32 = MIN_TARGET_POSITION;
    }
    return position_Q32;
}
/*
 * 函数名称：realSpeed_to_drvSpeed
 * 函数描述：将实际的设定速度(mm/s)转为驱动器使用的速度指令(指令单位/s)
 * 函数输入：relSpeed - 实际速度(mm/s)
 * 函数返回：供驱动器使用的32位无符号整型变量(指令单位/s)
 */
uint32_t LineMotor::realSpeed_to_drvSpeed(uint32_t relSpeed)
{
    uint32_t speed_Q31 = (relSpeed / INSTRUCTION_UNIT);
    if(speed_Q31 > MAX_PROFILE_SPEED)
    {
        speed_Q31 = MAX_PROFILE_SPEED;
    }
    else if(speed_Q31 < MIN_PROFILE_SPEED)
    {
        speed_Q31 = MIN_PROFILE_SPEED;
    }
    return speed_Q31;
}
/*
 * 函数名称：array_copy
 * 函数描述：数组复制
 * 函数输入：send_data - 目标数组
 *          given_data - 给定数组
 *          length - 数组长度
 * 函数返回：void
 */
void LineMotor::array_copy(uint8_t send_data[], uint8_t given_data[], uint8_t length)
{
    for(int i = 0; i < length; i++)
    {
        send_data[i] = given_data[i];
    }
}

