#ifndef __ROBOT_MAIN_NODE_H
#define __ROBOT_MAIN_NODE_H

enum ROBOT_STATE
{
    ROBOT_START_MODE,
    ROBOT_CALIBRATION_MODE,
    ROBOT_INIT_MODE,
    ROBOT_TEST_MODE,
    ROBOT_RL_MODE
};

enum MOTOR_STATE
{
    MOTOR_INIT_STATE,
    MOTOR_RESET_STATE,
    MOTOR_ARRIVE
};

typedef struct{
    float ankle_motor[4];
    float hip_motor[4];
    float knee_motor[2];
} RL_MotorsCmd;

#define ANKLE_LL_MOTOR_NUM         1
#define ANKLE_LR_MOTOR_NUM         2
#define ANKLE_RL_MOTOR_NUM         3
#define ANKLE_RR_MOTOR_NUM         4

#endif