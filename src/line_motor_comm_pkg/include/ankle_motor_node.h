#ifndef __TEST_MAIN_H
#define __TEST_MAIN_H

/**************************CAN通信设备宏定义**********************************/
#define LEFT_ANKLE_DEVICES_NUM          0       // 左腿踝关节两个电机对应的设备号
#define RIGHT_ANKLE_DEVICES_NUM         1       // 右腿踝关节两个电机对应的设备号
#define LEFT_ANKLE_CHANNEL              0       // 同一个踝关节的左侧电机通道
#define RIGHT_ANKLE_CHANNEL             1       // 同一个踝关节的右侧电机通道
#define CAN_BAUDRATE                    1000000 // 波特率1M
/****************************电机通信参数宏定义*******************************/
/* the following three cmds are used by MotorSetting function */
#define AUTOMATIC_FEEDBACK_CMD          0x01    // 自动反馈模式
#define RESPONSE_CMD                    0x02    // 应答模式，默认
#define SETTING_ZERO_CMD                0x03    // 设置当前位置为零点

#define COMM_RESPONSE_MODE              0x00
#define COMM_FEEDBACK_MODE              0x01
/* the following three acks are used as ack_status */
#define MIX_CONTROL_ACK                 1
#define POSITION_CURRENT_ACK            2
#define SPEED_CURRENT_ACK               3
/* ankle motor id: all ankle motors' ID are 1 */
#define ANKLE_MOTOR_ID                  1

#define START_RECEIVE_MESSAGE           1
#define END_RECEIVE_MESSAGE             0
/* ROS cmd state machine  */
#define ZEROS_SETTING_STEP              0
#define MIXED_CONTROL_STEP              1 //default step
#define POS_CONTROL_STEP                2 
#define SPEED_CONTROL_STEP              3
#define TORQUE_CONTROL_STEP             4   
/*********************************END**************************************/

#endif