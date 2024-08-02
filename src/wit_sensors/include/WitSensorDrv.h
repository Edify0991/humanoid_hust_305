#ifndef __WIT_SENSOR_DRV_H
#define __WIT_SENSOR_DRV_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/tf.h>

#define PI              3.14159

// 读写功能码
#define READ_DATA       0x03
#define WRITE_DATA      0X06
// 读数据的首地址
#define AX              0x34
#define READ_LEN        12
// 发送数据帧指令
#define ADDR            0
#define CMD             1
#define REG_H           2
#define REG_L           3
#define DATALH          4
#define DATALL          5
#define CRCH            6
#define CRCL            7
// 方位声明
#define X               0
#define Y               1
#define Z               2
// 姿态传感器类
class WitSensor
{
    public:
        serial::Serial serial_port;
        std::vector<uint8_t> rcv_buffer;
        u_char send_data[8];
        u_char data_ready;

        int16_t acc_XYZ[3];
        int16_t dang_XYZ[3];
        int16_t mag_XYZ[3];
        int16_t ang_XYZ[3];
        int16_t quaternion[4];

        WitSensor(); 

        void ReadAcc(void);
        void ReadAngSpeed(void);
        void ReadAng(void);

        void ReadHoldReg(void);

        void UnlockSensor(void);
        void AccCaliFcn(void);
        void AngCaliFcn(void);
        void GyroCaliFcn(void);
        void WriteEeprom(void);

        void StartCalibrating(void);
        void ExtractData(void);

        //void CrcCheck(uint8_t start, uint8_t end);
};

extern WitSensor hwt901b;
void *ReceiveWitSensor(void* param);

#endif