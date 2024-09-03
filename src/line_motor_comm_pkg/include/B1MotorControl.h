#ifndef __B1MOTORCOBTROL_H
#define __B1MOTORCOBTROL_H

#include <unistd.h>
// #include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "line_motor_comm_pkg/hipMotorMsgCmd.h"
#include "line_motor_comm_pkg/hipMotorMsgBack.h"


class JointMotor{
    public:
        JointMotor(unsigned short motorID, MotorType motorType, ros::NodeHandle nh, std::string motor_name);

        void BrakMode(uint8_t serialport);
        void MixedMode(float tau, float dq, float q, float kp, float kd, uint8_t serialport);
        void PosMode(float kp,float kd,float q, uint8_t serialport);
        void SpeedMode(float kd, float dq, uint8_t serialport);
        void DampingMode(float kd, uint8_t serialport);
        void TorqueMode(float tau, uint8_t serialport);
        void ZeroTorqueMode(uint8_t serialport);

        float PosBias(void);
        
        MotorCmd    motor_cmd; // dq为角速度,q为位置
        MotorData   motor_ret;
        MotorData   motor_last_ret; 
        MotorData   motor_init_ret;

    private:
        ros::NodeHandle nh; // ros节点句柄
        std::string motor_name;
        ros::Publisher motor_cmd_pub;
        ros::Subscriber motor_state_sub;
        void MotorStateCallback(const line_motor_comm_pkg::hipMotorMsgBack::ConstPtr& msg);
        void PublishMotorCmd(MotorCmd motor_cmd);
        double origangle;
};

    
#endif