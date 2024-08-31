#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include "unitreeMotor/unitreeMotor.h"
#include "B1MotorControl.h"
#include "Modbus_Comm.h"

namespace util
{
    void writeToCSV(const std::string& filename, float data, std::string mark);
    bool readdata(float* left_kuan, float* kneel, float* anklel, float* right_kuan, float* kneer, float* ankler, const std::string& filename);
    void begin_gait(JointMotor& motorL_0_a, JointMotor& motorL_1_a, 
                    JointMotor& motorR_0_a, JointMotor& motorR_1_a, 
                    JointMotor& motorL_0, JointMotor& motorL_2, 
                    JointMotor& motorR_0, JointMotor& motorR_2, 
                    float motorL_0_ORI_a, float motorL_1_ORI_a, 
                    float motorR_0_ORI_a, float motorR_1_ORI_a, 
                    float motorL_0_ORI, float motorL_2_ORI, 
                    float motorR_0_ORI, float motorR_2_ORI, 
                    float* left_hip_begin, float* kneel_begin, 
                    float* anklel_begin, float* right_hip_begin, 
                    float* kneer_begin, float* ankler_begin, 
                    LineMotor& left_leg, LineMotor& right_leg, 
                    float* hip_roll, int unit_time, float k_ankle_hip_roll, int number_begin);
    void period_gait(JointMotor& motorL_0_a, JointMotor& motorL_1_a, 
                    JointMotor& motorR_0_a, JointMotor& motorR_1_a, 
                    JointMotor& motorL_0, JointMotor& motorL_2, 
                    JointMotor& motorR_0, JointMotor& motorR_2, 
                    float motorL_0_ORI_a, float motorL_1_ORI_a, 
                    float motorR_0_ORI_a, float motorR_1_ORI_a, 
                    float motorL_0_ORI, float motorL_2_ORI, 
                    float motorR_0_ORI, float motorR_2_ORI, 
                    float* left_hip_period, float* kneel_period, 
                    float* anklel_period, float* right_hip_period, 
                    float* kneer_period, float* ankler_period, 
                    float* ankler_begin, float* anklel_begin,
                    LineMotor& left_leg, LineMotor& right_leg, 
                    float* hip_roll, int unit_time, float k_ankle_hip_roll, int number_begin, int number_period, int loop_num
                    );

    int readdatafromcsv(float** left_hip_roll, float** left_pitch, float** left_knee, float** left_ankle, float** left_foot, float** right_hip_roll, float** right_pitch, float** right_knee, float** right_ankle, float** right_foot, const std::string& filename);
}


#endif

