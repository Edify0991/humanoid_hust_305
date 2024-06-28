#ifndef __MODBUS_COMM_H
#define __MODBUS_COMM_H

#include <serial/serial.h>
#include <ros/ros.h>

// 通用参数
#define ADDR            0
#define CMD             1
#define GROUP           2
#define OFFSET          3
// 读写命令
#define READ_ALL        0x03
#define WRITE_16B       0x06 
#define WRITE_32B       0x10
#define WRITE_ERROR     0x86
// 读写16位参数命令
#define DATAH_16B       4
#define DATAL_16B       5
#define CRCL_16B        6
#define CRCH_16B        7
// 写32位参数命令
#define DATALH_32B      4
#define DATALL_32B      5
#define DATA_BYTE       6

#pragma pack(1)
typedef struct{
    u_char Addr;
    u_char Cmd;
    u_char Group;
    u_char Offset;
    u_char DataH;
    u_char DataL;
}DATA_FRAME_16B;
#pragma pack();

#pragma pack(1)
typedef struct{
    u_char CrcL;
    u_char CrcH;
}CRC_FRAME_16B;
#pragma pack()

#pragma pack(1)
typedef struct{
    DATA_FRAME_16B MasterData;
    CRC_FRAME_16B MasterCrc;
}MASTER_FRAME_16B;
#pragma pack()

class LineMotor
{
public:
    u_char MotorID; // 1 or 2
    int16_t MotorStepNum; // -9999~9999

    MASTER_FRAME_16B SendToMotorValue;
    u_char RcvFromMotor_Buffer[20];

    LineMotor(u_char DrvAddr, int16_t DefaultPulse);
    uint8_t Enable_VDI(serial::Serial* serialport);       // 虚拟数据输入端口使能
    uint8_t Para_H17_Init(void);    // 分配伺服使能,步进量使能功能码和旋转一圈位置指令数
    uint8_t Enable_StepFcn(void);   // 使能步进量
    uint16_t CommCrcValueCal(const uint8_t *data, uint16_t length); // CRC校验
};

extern serial::Serial LineMotorSerial;

#endif