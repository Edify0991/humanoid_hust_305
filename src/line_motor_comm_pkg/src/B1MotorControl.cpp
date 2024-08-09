#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "B1MotorControl.h"


// 启用串口前需在B1MotorControl.h文件中取消相应宏定义的注释
#ifdef TTYUSB0
SerialPort  left_leg_b1("/dev/usb_hip_left");
#endif

#ifdef TTYUSB1
SerialPort  right_leg_b1("/dev/usb_hip_right");
#endif

#ifdef TTYUSB2
SerialPort  left_leg_a1("/dev/usb_ankle_left");
#endif

#ifdef TTYUSB3
SerialPort  right_leg_a1("/dev/usb_ankle_right");
#endif

JointMotor::JointMotor(unsigned short motorID, MotorType motorType)
{
    motor_cmd.id = motorID;
    motor_cmd.motorType = motorType;
    motor_ret.motorType = motorType;
}
/*
 * 函数名称：BrakMode
 * 函数描述：关节电机停机
 * 函数输入：serialport - 关节电机对应串口
 * LEFT_LEG_B1：左腿B1电机串口
 * LEFT_LEG_A1：左腿A1电机串口
 * RIGHT_LEG_B1：右腿B1电机串口
 * LEFT_LEG_B1：左腿B1电机串口           
 * 函数返回：None
 */
void JointMotor::BrakMode(uint8_t serialport)
{
    motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::BRAKE);
    switch(serialport)
    {
        #ifdef TTYUSB0
        case LEFT_LEG_B1:
            left_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif 

        #ifdef TTYUSB1
        case RIGHT_LEG_B1:
            right_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB2
        case LEFT_LEG_A1:
            left_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB3
        case RIGHT_LEG_A1:
            right_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;  
        #endif 
    }
}
/*
 * 函数名称：PosMode
 * 函数描述：关节电机位置控制
 * 函数输入：serialport - LEFT_LEG_B1：左腿B1电机串口
 *                      LEFT_LEG_A1：左腿A1电机串口
 *                      RIGHT_LEG_B1：右腿B1电机串口
 *                      LEFT_LEG_B1：左腿B1电机串口
 *          kp - 位置比例系数，无负载下默认0.01   
 *          kd - 速度比例系数，无负载下默认0.01
 *          q - 目标位置，弧度               
 * 函数返回：None
 */
void JointMotor::PosMode(float kp,float kd,float q, uint8_t serialport)
{
    motor_cmd.tau = 0;
    motor_cmd.dq = 0;

    motor_cmd.kp = kp;
    motor_cmd.kd = kd;
    motor_cmd.q = q * queryGearRatio(motor_cmd.motorType); // q代表角度
    motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    switch(serialport)
    {
        #ifdef TTYUSB0
        case LEFT_LEG_B1:
            left_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif 

        #ifdef TTYUSB1
        case RIGHT_LEG_B1:
            right_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB2
        case LEFT_LEG_A1:
            left_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB3
        case RIGHT_LEG_A1:
            right_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;  
        #endif  
    }
}
/*
 * 函数名称：SpeedMode
 * 函数描述：关节电机速度控制
 * 函数输入：serialport - LEFT_LEG_B1：左腿B1电机串口
 *                      LEFT_LEG_A1：左腿A1电机串口
 *                      RIGHT_LEG_B1：右腿B1电机串口
 *                      LEFT_LEG_B1：左腿B1电机串口  
 *          kd - 速度比例系数，无负载下默认0.01
 *          dq - 目标角速度(r/s)               
 * 函数返回：None
 */
void JointMotor::SpeedMode(float kd, float dq, uint8_t serialport)
{
    motor_cmd.tau = 0;
    motor_cmd.q = 0;
    motor_cmd.kp = 0;

    motor_cmd.kd = kd;
    motor_cmd.dq = dq * queryGearRatio(motor_cmd.motorType);
    motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    switch(serialport)
    {
        #ifdef TTYUSB0
        case LEFT_LEG_B1:
            left_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif 

        #ifdef TTYUSB1
        case RIGHT_LEG_B1:
            right_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB2
        case LEFT_LEG_A1:
            left_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB3
        case RIGHT_LEG_A1:
            right_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;  
        #endif
    }
}
/*
 * 函数名称：TorqueMode
 * 函数描述：关节电机转矩控制
 * 函数输入：serialport - LEFT_LEG_B1：左腿B1电机串口
 *                      LEFT_LEG_A1：左腿A1电机串口
 *                      RIGHT_LEG_B1：右腿B1电机串口
 *                      LEFT_LEG_B1：左腿B1电机串口  
 *          tau - 目标转矩              
 * 函数返回：None
 */
void JointMotor::TorqueMode(float tau, uint8_t serialport)
{
    motor_cmd.q = 0;
    motor_cmd.kp = 0;
    motor_cmd.kd = 0;
    motor_cmd.dq = 0;

    motor_cmd.tau = tau;
    motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    switch(serialport)
    {
        #ifdef TTYUSB0
        case LEFT_LEG_B1:
            left_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif 

        #ifdef TTYUSB1
        case RIGHT_LEG_B1:
            right_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB2
        case LEFT_LEG_A1:
            left_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB3
        case RIGHT_LEG_A1:
            right_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;  
        #endif
    }
}
/*
 * 函数名称：MixedMode
 * 函数描述：关节电机混合模式控制
 * 函数输入：serialport - LEFT_LEG_B1：左腿B1电机串口
 *                      LEFT_LEG_A1：左腿A1电机串口
 *                      RIGHT_LEG_B1：右腿B1电机串口
 *                      LEFT_LEG_B1：左腿B1电机串口  
 *          各个参数如以上函数所示，此处不再重复              
 * 函数返回：None
 */
void JointMotor::MixedMode(float tau, float dq, float q, float kp, float kd, uint8_t serialport)
{
    motor_cmd.tau = tau* queryGearRatio(motor_cmd.motorType);
    motor_cmd.dq = dq* queryGearRatio(motor_cmd.motorType);
    motor_cmd.q = q* queryGearRatio(motor_cmd.motorType);
    motor_cmd.kp = kp;
    motor_cmd.kd = kd;
    motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    switch(serialport)
    {
        #ifdef TTYUSB0
        case LEFT_LEG_B1:
            left_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif 

        #ifdef TTYUSB1
        case RIGHT_LEG_B1:
            right_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB2
        case LEFT_LEG_A1:
            left_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB3
        case RIGHT_LEG_A1:
            right_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;  
        #endif
    }
}
/*
 * 函数名称：DampingMode
 * 函数描述：关节电机阻尼控制
 * 函数输入：serialport - LEFT_LEG_B1：左腿B1电机串口
 *                      LEFT_LEG_A1：左腿A1电机串口
 *                      RIGHT_LEG_B1：右腿B1电机串口
 *                      LEFT_LEG_B1：左腿B1电机串口  
 *          kd - 速度比例系数              
 * 函数返回：None
 */
void JointMotor::DampingMode(float kd, uint8_t serialport)
{
    motor_cmd.tau = 0;
    motor_cmd.q = 0;
    motor_cmd.kp = 0;
    motor_cmd.dq = 0;
    
    motor_cmd.kd = kd;
    motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    switch(serialport)
    {
        #ifdef TTYUSB0
        case LEFT_LEG_B1:
            left_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif 

        #ifdef TTYUSB1
        case RIGHT_LEG_B1:
            right_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB2
        case LEFT_LEG_A1:
            left_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB3
        case RIGHT_LEG_A1:
            right_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;  
        #endif 
    }
}
/*
 * 函数名称：ZeroTorqueMode
 * 函数描述：关节电机零力矩模式
 * 函数输入：serialport - LEFT_LEG_B1：左腿B1电机串口
 *                      LEFT_LEG_A1：左腿A1电机串口
 *                      RIGHT_LEG_B1：右腿B1电机串口
 *                      LEFT_LEG_B1：左腿B1电机串口              
 * 函数返回：None
 */
void JointMotor::ZeroTorqueMode(uint8_t serialport)
{
    motor_cmd.tau = 0;
    motor_cmd.q = 0;
    motor_cmd.kp = 0;
    motor_cmd.kd = 0;
    motor_cmd.dq = 0;
    motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    switch(serialport)
    {
        #ifdef TTYUSB0
        case LEFT_LEG_B1:
            left_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif 

        #ifdef TTYUSB1
        case RIGHT_LEG_B1:
            right_leg_b1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB2
        case LEFT_LEG_A1:
            left_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;
        #endif

        #ifdef TTYUSB3
        case RIGHT_LEG_A1:
            right_leg_a1.sendRecv(&motor_cmd, &motor_ret);
            break;  
        #endif   
    }
}