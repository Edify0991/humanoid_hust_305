#ifndef __B1MOTORCOBTROL_H
#define __B1MOTORCOBTROL_H

#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "std_msgs/String.h"

#define TTYUSB0
#define TTYUSB1
#define TTYUSB2
#define TTYUSB3

#define LEFT_LEG_B1     1
#define LEFT_LEG_A1     3
#define RIGHT_LEG_B1    2
#define RIGHT_LEG_A1    4

class JointMotor{
    public:
        JointMotor(unsigned short motorID, MotorType motorType);

        void BrakMode(uint8_t serialport);
        void MixedMode(float tau, float dq, float q, float kp, float kd, uint8_t serialport);
        void PosMode(float kp,float kd,float q, uint8_t serialport);
        void SpeedMode(float kd, float dq, uint8_t serialport);
        void DampingMode(float kd, uint8_t serialport);
        void TorqueMode(float tau, uint8_t serialport);
        void ZeroTorqueMode(uint8_t serialport);

        MotorCmd    motor_cmd; // dq为角速度,q为位置
        MotorData   motor_ret;
};

#endif