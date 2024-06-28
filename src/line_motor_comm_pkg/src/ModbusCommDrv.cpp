/*
 * 文件名：ModbusCommDrv.cpp
 * 作者：卢洪磊
 * 日期：2024-6-26
 * 文件描述：该文件产生了一个直线电机类，用于配置直线电机的相关数据，对应驱动器为汇川SV660P
 * Version：1.0
 * 第一次修改：
 * 第二次修改
 */
#include <iostream>
#include <string.h>
#include "Modbus_Comm.h"

using namespace std;
using namespace serial;
/*
 * 函数名称：LineMotor
 * 函数描述：直线电机类的构造函数
 * 函数输入：DrvAddr - 驱动器的总线地址
 *         DefaultPulse - 直线电机步进量脉冲
 * 函数返回：None
 */
LineMotor::LineMotor(u_char DrvAddr, int16_t DefaultPulse)
{
    MotorID = DrvAddr;
    MotorStepNum = DefaultPulse;

    SendToMotorValue.MasterData.Addr = DrvAddr;
}
/*
 * 函数名称：Enable_VDI
 * 函数描述：驱动器虚拟输入输出端口使能函数，控制H0C.09和H0C.10功能码
 * 函数输入：void
 * 函数返回：VDI使能状态
 *          -1 - 使能失败
 *          0 - 串口未发送成功
 *          1 - 使能成功
 */
uint8_t LineMotor::Enable_VDI(Serial* serialport)
{
    uint16_t Crc_16B = 0;
    SendToMotorValue.MasterData.Addr = MotorID;
    SendToMotorValue.MasterData.Cmd = WRITE_16B;
    SendToMotorValue.MasterData.Group = 0x0C;
    SendToMotorValue.MasterData.Offset = 0x09;
    SendToMotorValue.MasterData.DataH = 0x00;
    SendToMotorValue.MasterData.DataL = 0x01;  // H0C.09置1，启动通信VDI功能

    // 获取CRC校验位
    Crc_16B = CommCrcValueCal(&SendToMotorValue.MasterData.Addr, 6);
    SendToMotorValue.MasterCrc.CrcL = (u_char)(Crc_16B & 0x00ff);
    SendToMotorValue.MasterCrc.CrcH = (u_char)((Crc_16B >> 8) & 0x00ff);

    //printf("占用字节为%d字节\r\n", sizeof(SendToMotorValue)); // 共8字节

    serialport->write(&SendToMotorValue.MasterData.Addr, sizeof(SendToMotorValue));
    if(serialport->available())
    {
        ROS_INFO_STREAM("Motor driver is returning VDI!");
        serialport->read(RcvFromMotor_Buffer, serialport->available());
        if(RcvFromMotor_Buffer[CMD] == WRITE_ERROR)
        {
            ROS_ERROR_STREAM("Communication with sv660p occurred errors, VDI failed!");
            return -1;
        }
        else if(RcvFromMotor_Buffer[CMD] == WRITE_16B)
        {
            // 设置VDI上电后的默认电平值为0
            SendToMotorValue.MasterData.Offset = 0x10;
            SendToMotorValue.MasterData.DataH = 0x00;
            SendToMotorValue.MasterData.DataL = 0X00;
            // 获取CRC校验位
            Crc_16B = CommCrcValueCal(&SendToMotorValue.MasterData.Addr, 6);
            SendToMotorValue.MasterCrc.CrcL = (u_char)(Crc_16B & 0x00ff);
            SendToMotorValue.MasterCrc.CrcH = (u_char)((Crc_16B >> 8) & 0x00ff);
            serialport->write(&SendToMotorValue.MasterData.Addr, sizeof(SendToMotorValue));
            if(serialport->available())
            {
                ROS_INFO_STREAM("Motor driver is returning default VDI!");
                serialport->read(RcvFromMotor_Buffer, serialport->available());
                if(RcvFromMotor_Buffer[CMD] == WRITE_ERROR)
                {
                    ROS_INFO_STREAM("");
                }
            }
        }
        return 1;
    }
    return 0;
}
/*
 * 函数名称：CommCrcValueCal
 * 函数描述：汇川SV660P CRC校验函数
 * 函数输入： data - 发送或者接收数组
 *          length - 发送或者接收数据长度
 * 函数返回：16位CRC校验值
 */
uint16_t LineMotor::CommCrcValueCal(const uint8_t *data, uint16_t length)
{
    uint16_t crcValue = 0xffff;
    int16_t i;
    while(length--)
    {
        crcValue ^= *data++;
        for(i = 0; i < 8; i++)
        {
            if(crcValue & 0x0001)
            {
                crcValue = (crcValue >> 1) ^ 0xA001;
            }
            else
            {
                crcValue = crcValue >> 1;
            }
        }
    }
    return crcValue;
}