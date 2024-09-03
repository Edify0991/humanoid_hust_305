#include <unistd.h>
// #include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "B1MotorControl.h"

JointMotor::JointMotor(unsigned short motorID, MotorType motorType, ros::NodeHandle nh, std::string motor_name)
{
    this->nh = nh;
    this->motor_name = motor_name;
    motor_cmd.id = motorID;
    motor_cmd.motorType = motorType;
    motor_ret.motorType = motorType;
    motor_ret.q = 100;
    motor_cmd_pub = nh.advertise<line_motor_comm_pkg::hipMotorMsgCmd>(motor_name + "_cmd", 1);
    motor_state_sub = nh.subscribe(motor_name + "_state", 1, &JointMotor::MotorStateCallback, this);
}

float JointMotor::PosBias(void)
{
    return (abs(motor_ret.q - motor_last_ret.q));
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
    // motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::BRAKE);
    // motor_cmd.tau = 0;
    // motor_cmd.q = 0;
    // motor_cmd.kp = 0;
    // motor_cmd.kd = 0;
    // motor_cmd.dq = 0;
    // PublishMotorCmd(motor_cmd);
    line_motor_comm_pkg::hipMotorMsgCmd hip_motor_cmd_msg;
    hip_motor_cmd_msg.tau = 0;
    hip_motor_cmd_msg.dq = 0;
    hip_motor_cmd_msg.q = 0;
    hip_motor_cmd_msg.kp = 0;
    hip_motor_cmd_msg.kd = 0;
    hip_motor_cmd_msg.id = motor_cmd.id;
    hip_motor_cmd_msg.motorType = (char)motor_cmd.motorType;
    hip_motor_cmd_msg.mode = queryMotorMode(motor_cmd.motorType, MotorMode::BRAKE);
    motor_cmd_pub.publish(hip_motor_cmd_msg);
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
    PublishMotorCmd(motor_cmd);
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
    PublishMotorCmd(motor_cmd);
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
    PublishMotorCmd(motor_cmd);
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
    line_motor_comm_pkg::hipMotorMsgCmd hip_motor_cmd_msg;
    hip_motor_cmd_msg.tau = tau;
    hip_motor_cmd_msg.dq = dq;
    hip_motor_cmd_msg.q = q;
    hip_motor_cmd_msg.kp = kp;
    hip_motor_cmd_msg.kd = kd;
    hip_motor_cmd_msg.id = motor_cmd.id;
    hip_motor_cmd_msg.motorType = (char)motor_cmd.motorType;
    hip_motor_cmd_msg.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    motor_cmd_pub.publish(hip_motor_cmd_msg);
    // motor_cmd.tau = tau* queryGearRatio(motor_cmd.motorType);
    // motor_cmd.dq = dq* queryGearRatio(motor_cmd.motorType);
    // motor_cmd.q = q* queryGearRatio(motor_cmd.motorType);
    // motor_cmd.kp = kp;
    // motor_cmd.kd = kd;
    // motor_cmd.mode = queryMotorMode(motor_cmd.motorType, MotorMode::FOC);
    // PublishMotorCmd(motor_cmd);
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
    PublishMotorCmd(motor_cmd);
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
    PublishMotorCmd(motor_cmd);
}

void JointMotor::MotorStateCallback(const line_motor_comm_pkg::hipMotorMsgBack::ConstPtr& msg)
{
    motor_last_ret.q = motor_ret.q;
    motor_last_ret.dq = motor_ret.dq;
    motor_last_ret.tau = motor_ret.tau;
    motor_last_ret.motor_id = motor_ret.motor_id;

    motor_ret.q = msg->q;
    motor_ret.dq = msg->dq;
    motor_ret.tau = msg->tau;
    motor_ret.motor_id = msg->id;
}

void JointMotor::PublishMotorCmd(MotorCmd motor_cmd)
{
    line_motor_comm_pkg::hipMotorMsgCmd motor_cmd_msg;
    motor_cmd_msg.id = motor_cmd.id;
    motor_cmd_msg.mode = motor_cmd.mode;
    motor_cmd_msg.motorType = (char)motor_cmd.motorType;
    motor_cmd_msg.tau = motor_cmd.tau;
    motor_cmd_msg.dq = motor_cmd.dq;
    motor_cmd_msg.q = motor_cmd.q;
    motor_cmd_msg.kp = motor_cmd.kp;
    motor_cmd_msg.kd = motor_cmd.kd;
    motor_cmd_pub.publish(motor_cmd_msg);
}
