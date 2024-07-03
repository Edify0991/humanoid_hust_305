#ifndef __MODBUS_COMM_H
#define __MODBUS_COMM_H

#include <serial/serial.h>
#include <ros/ros.h>
#include "controlcan.h"

// 驱动器参数限制值
#define MAX_TARGET_POSITION      2147483648
#define MIN_TARGET_POSITION      -2147483648
#define MAX_PROFILE_SPEED        4294967295
#define MIN_PROFILE_SPEED        0
// 驱动器相关参数
#define MAX_INSTRUCTION_UNIT     1048576     // 电机每转一圈的位置指令总数
#define LEAD_SCREW               2           // 丝杠导程，每转一圈可移动2mm距离
// PDO映射地址
#define RPDO1_MAPPING_OBJECT     0x1600
#define TPDO1_MAPPING_OBJECT     0x1A00

typedef struct{
    float target_position;
    uint32_t profile_speed;
}MASTER_TO_SERVO_PDO;

typedef struct{
    uint16_t status_cmd;
    float position_feedback;
}SERVO_TO_MASTER_PDO;

typedef struct{
    u_char command_code;   // 0x2f, 0x2b, 0x27, 0x23：1~4字节
    u_char index_L;        // 主索引低字节
    u_char index_H;        // 主索引高字节
    u_char sub_index;      // 子索引
    u_char data_L1;        // 数据低8位
    u_char data_L2;        // 数据低9~16位
    u_char data_H1;        // 数据高17~24位
    u_char data_H2;        // 数据高25~32位 
}LINEMOTOR_COMM_FRAME;

class LineMotor
{
public:
    /* 变量声明区 */
    uint8_t addr_BUS;
    /* 
     * TPDO1:0011b(180h)+addr；RPDO1：0100b(200h)+addr；
     * TPDO2:0101b+addr；RPDO2：0110b+addr；
     * TPDO3:0111b+addr；RPDO3：1000b+addr；
     * T_SDO:1011b(580h)+addr；R_SDO：1100b(600h)+addr;
    */
    uint16_t RPdo1_COB_ID; // COB-ID = Function code + Node addr
    uint16_t RPdo2_COB_ID;
    uint16_t TPdo1_COB_ID;
    uint32_t Sdo_COB_ID; 
    MASTER_TO_SERVO_PDO command;  // 设置轮廓位置模式下的目标位置和轮廓速度,提供给开发者
    SERVO_TO_MASTER_PDO feedback; // 接收驱动器返回的状态字和位置反馈，提供给开发者

    LINEMOTOR_COMM_FRAME frame;
    VCI_INIT_CONFIG motor_can_config;
    VCI_CAN_OBJ sendTodriver_msg;
    /* 函数声明区域 */
    LineMotor(uint8_t addr); // 直线电机构造函数
    int8_t DRIVER_OpenCanComm(void); // 开启can通信
    void AbsPos_Set(float absPos); // 绝对位置设置
    void RelPos_Set(float relPos, uint32_t refSpeed); // 相对位置设置
    void AbsPos_Fdb(void);           // 绝对位置反馈值获取

    int32_t fPos_to_iPos_Fcn(float target_position);  // 浮点位置指令转为供驱动器使用的int32位置指令
    uint32_t realSpeed_to_drvSpeed(float relSpeed);   // 实际设定速度转为供驱动器使用的驱动器轮廓速度
    void array_copy(uint8_t send_data[], uint8_t given_data[], uint8_t length);
};

#endif