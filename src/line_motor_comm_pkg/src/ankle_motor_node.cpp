#include "ros/ros.h"
#include "ankle_comm.h"
#include "ankle_motor_node.h"
#include "Modbus_Comm.h"

#include "line_motor_comm_pkg/ankleMotorMsgCmd.h"
#include "line_motor_comm_pkg/ankleMotorMsgBack.h"
#include "ros/callback_queue.h"
#include <thread>
#include <vector>

// 函数接口
int Can_Channel_Init(int devs, int channel, std::string str);
void AnkleMotor_Rcv_Fcn(int devs, int channel);
void AnkleMotor_Send_Fcn(int devs, int channel);
// 是否控制踝关节电机
const bool is_ankle_motor_control = true;
// 踝关节CAN设备命名
std::vector<std::string> ankle_canDevs_name = {
                                "左腿左侧",
                                "左腿右侧",
                                "右腿左侧",
                                "右腿右侧"
                                };

// 踝关节电机命名
std::vector<std::string> ankle_motor_name = {
                                "ankle_motor_ld",
                                "ankle_motor_lu",
                                "ankle_motor_rd",
                                "ankle_motor_ru"
                                };
// 踝关节电机与USB_CAN设备对应关系
std::vector<int> ankle_motor_usb = {
                                LEFT_ANKLE_DEVICES_NUM,
                                LEFT_ANKLE_DEVICES_NUM,
                                RIGHT_ANKLE_DEVICES_NUM,
                                RIGHT_ANKLE_DEVICES_NUM
                                };
// 踝关节电机与CAN_CHANNEL对应关系
std::vector<int> ankle_motor_channel = {
                                LEFT_ANKLE_CHANNEL,
                                RIGHT_ANKLE_CHANNEL,
                                LEFT_ANKLE_CHANNEL,
                                RIGHT_ANKLE_CHANNEL
                                };

std::vector<std::thread> ankle_motor_threads;
std::vector<ros::Publisher> ankle_motor_state_pub;
std::vector<ankle_motor*> ankle_motor_ports;

// 电机控制变量
volatile int state[4] = {MIXED_CONTROL_STEP, MIXED_CONTROL_STEP, MIXED_CONTROL_STEP, MIXED_CONTROL_STEP};
// volatile int state[4] = {POS_CONTROL_STEP, POS_CONTROL_STEP, POS_CONTROL_STEP, POS_CONTROL_STEP};
volatile float kp[4] = {10, 10, 10, 10};
volatile float kd[4] = {0.1, 0.1, 0.1, 0.1};
volatile float pos[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 弧度
volatile float spd[4] = {0.8, 0.8, 0.8, 0.8}; // 弧度/秒
volatile float tau[4] = {0, 0, 0, 0};
volatile float limit_current[4] = {150,150,150,150};

// 踝关节电机接收到控制命令回调
void AnkleMotor_CmdCallback(const line_motor_comm_pkg::ankleMotorMsgCmd::ConstPtr& msg, int ankle_motor_thread_id)
{
    state[ankle_motor_thread_id] = msg->motor_state;
    kp[ankle_motor_thread_id] = msg->kp;
    kd[ankle_motor_thread_id] = msg->kd;
    pos[ankle_motor_thread_id] = msg->pos;
    spd[ankle_motor_thread_id] = msg->spd;
    tau[ankle_motor_thread_id] = msg->tau;
    limit_current[ankle_motor_thread_id] = msg->limit_current * 10.0f;
}

// 踝关节电机线程，用于接收踝关节控制命令
void AnkleMotor_CmdThread(int ankle_motor_thread_id, ros::NodeHandle nh)
{
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    ros::Subscriber sub = nh.subscribe<line_motor_comm_pkg::ankleMotorMsgCmd>(
                                    ankle_motor_name[ankle_motor_thread_id] + "_cmd", \
                                    1, \
                                    boost::bind(AnkleMotor_CmdCallback, \
                                    _1, \
                                    ankle_motor_thread_id) \
                                    );
    while(ros::ok())
    {
        queue.callAvailable(ros::WallDuration());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ankle_motor_node");
    ros::NodeHandle nh;

    ankle_motor_threads.resize(ankle_motor_name.size());
    ankle_motor_state_pub.resize(ankle_motor_name.size());
    
    // 打开CAN通信硬件接口
    int devs_num; 
    devs_num = CAN_ScanDevice(); // 扫描设备
    if(devs_num <= 0)
    {
        std::cout<<"未扫描到踝关节设备！"<<std::endl;
        return -1;
    }
    std::cout<<"在线设备数量："<<devs_num<<std::endl;
    Can_Channel_Init(ankle_motor_usb[0], ankle_motor_channel[0], ankle_canDevs_name[0]);
    Can_Channel_Init(ankle_motor_usb[1], ankle_motor_channel[1], ankle_canDevs_name[1]);
    Can_Channel_Init(ankle_motor_usb[2], ankle_motor_channel[2], ankle_canDevs_name[2]);
    Can_Channel_Init(ankle_motor_usb[3], ankle_motor_channel[3], ankle_canDevs_name[3]);
    
    // 初始化踝关节电机类
    for(uint8_t i = 0; i < ankle_motor_name.size(); i++)
    {
        ankle_motor* ptr = new ankle_motor(ankle_motor_usb[i], ankle_motor_channel[i], ANKLE_MOTOR_ID);
        ankle_motor_ports.push_back(ptr);
        ankle_motor_state_pub[i] = nh.advertise<line_motor_comm_pkg::ankleMotorMsgBack>(ankle_motor_name[i] + "_state", 1);
        ankle_motor_threads[i] = std::thread(AnkleMotor_CmdThread, i, std::ref(nh));
    }

    // 创建踝关节电机的接收线程
    std::thread AnkleMotor_ll_RcvThread(std::bind(AnkleMotor_Rcv_Fcn, ankle_motor_usb[0], ankle_motor_channel[0]));
    if(AnkleMotor_ll_RcvThread.joinable())
    {
        std::cout << "左侧左边脚踝电机接收线程创建成功" << std::endl;
        AnkleMotor_ll_RcvThread.detach();
    } else {
        std::cerr << "左侧左边脚踝电机接收线程创建失败" << std::endl;
    }
    std::thread AnkleMotor_lr_RcvThread(std::bind(AnkleMotor_Rcv_Fcn, ankle_motor_usb[1], ankle_motor_channel[1]));
    if(AnkleMotor_lr_RcvThread.joinable())
    {
        std::cout << "左侧右边脚踝电机接收线程创建成功" << std::endl;
        AnkleMotor_lr_RcvThread.detach();
    } else {
        std::cerr << "左侧右边脚踝电机接收线程创建失败" << std::endl;
    }
    std::thread AnkleMotor_rl_RcvThread(std::bind(AnkleMotor_Rcv_Fcn, ankle_motor_usb[2], ankle_motor_channel[2]));
    if(AnkleMotor_rl_RcvThread.joinable())
    {
        std::cout << "右侧左边脚踝电机接收线程创建成功" << std::endl;
        AnkleMotor_rl_RcvThread.detach();
    } else {
        std::cerr << "右侧左边脚踝电机接收线程创建失败" << std::endl;
    }
    std::thread AnkleMotor_rr_RcvThread(std::bind(AnkleMotor_Rcv_Fcn, ankle_motor_usb[3], ankle_motor_channel[3]));
    if(AnkleMotor_rr_RcvThread.joinable())
    {
        std::cout << "右侧右边脚踝电机接收线程创建成功" << std::endl;
        AnkleMotor_rr_RcvThread.detach();
    } else {
        std::cerr << "右侧右边脚踝电机接收线程创建失败" << std::endl;
    }

    std::thread AnkleMotor_ll_SendThread(std::bind(AnkleMotor_Send_Fcn, ankle_motor_usb[0], ankle_motor_channel[0]));
    if(AnkleMotor_ll_SendThread.joinable())
    {
        std::cout << "左侧左边脚踝电机发送线程创建成功" << std::endl;
        AnkleMotor_ll_SendThread.detach();
    } else {
        std::cerr << "左侧左边脚踝电机发送线程创建失败" << std::endl;
    }
    std::thread AnkleMotor_lr_SendThread(std::bind(AnkleMotor_Send_Fcn, ankle_motor_usb[1], ankle_motor_channel[1]));
    if(AnkleMotor_lr_SendThread.joinable())
    {
        std::cout << "左侧右边脚踝电机发送线程创建成功" << std::endl;
        AnkleMotor_lr_SendThread.detach();
    } else {
        std::cerr << "左侧右边脚踝电机发送线程创建失败" << std::endl;
    }
    std::thread AnkleMotor_rl_SendThread(std::bind(AnkleMotor_Send_Fcn, ankle_motor_usb[2], ankle_motor_channel[2]));
    if(AnkleMotor_rl_SendThread.joinable())
    {
        std::cout << "右侧左边脚踝电机发送线程创建成功" << std::endl;
        AnkleMotor_rl_SendThread.detach();
    } else {
        std::cerr << "右侧左边脚踝电机发送线程创建失败" << std::endl;
    }
    std::thread AnkleMotor_rr_SendThread(std::bind(AnkleMotor_Send_Fcn, ankle_motor_usb[3], ankle_motor_channel[3]));
    if(AnkleMotor_rr_SendThread.joinable())
    {
        std::cout << "右侧右边脚踝电机发送线程创建成功" << std::endl;
        AnkleMotor_rr_SendThread.detach();
    } else {
        std::cerr << "右侧右边脚踝电机发送线程创建失败" << std::endl;
    }

    for(auto& thread : ankle_motor_threads)
    {
        thread.detach();
    }
    while(ros::ok()){}
}

void AnkleMotor_Send_Fcn(int devs, int channel)
{
    int i = devs * 2 + channel;
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        // msg传入消息为度和度每秒, 使用send_motor_ctrl_cmd，传入参数为弧度制
        float pos_rad = pos[i];
        float spd_rad = spd[i];
        float spd_rpm = ((spd_rad * 30.0f) / PI) * 10; // 传入函数的需要乘以10
        float pos_o = pos_rad / PI * 180.0f;
        // float spd_o = spd_rad / PI * 180.0f;
        // 不同的指令有不同的倍率
        // float spd_x10 = spd_o * 10.0f;
        switch(state[i])
        {
            case ZEROS_SETTING_STEP:
                ankle_motor_ports[i]->MotorSetting(SETTING_ZERO_CMD);
            break;
            case MIXED_CONTROL_STEP:
                ankle_motor_ports[i]->send_motor_ctrl_cmd(kp[i], kd[i], pos_rad, spd_rad, tau[i]);
            break;
            case POS_CONTROL_STEP:
                ankle_motor_ports[i]->set_motor_position(pos_o, spd_rpm, limit_current[i], MIX_CONTROL_ACK);
            break;
            case SPEED_CONTROL_STEP:
                ankle_motor_ports[i]->set_motor_speed(spd_rpm, limit_current[i], MIX_CONTROL_ACK);
            break;
            default:
            break;
        }
        loop_rate.sleep();
    }
}

void AnkleMotor_Rcv_Fcn(int devs, int channel)
{
    Can_Msg rcv_msg[300];
    line_motor_comm_pkg::ankleMotorMsgBack ankle_motor_ret_msg;
    int reclen;
    int i = devs * 2 + channel;
    while(1)
    {
        if(reclen = CAN_Receive(devs, channel, rcv_msg, 300, 100)>0)
        {
            for(int j = 0; j < reclen; j++)
            {
                ankle_motor_ports[i]->RV_can_data_repack(rcv_msg[j], COMM_RESPONSE_MODE);
                ankle_motor_ret_msg.tau = ankle_motor_ports[i]->rv_motor_msg[0].current_actual_float * ANKLE_MOTOR_KT; // 返回力矩值
                ankle_motor_ret_msg.spd = ankle_motor_ports[i]->rv_motor_msg[0].speed_actual_rad; // 返回弧度
                ankle_motor_ret_msg.pos = ankle_motor_ports[i]->rv_motor_msg[0].angle_actual_rad; // 返回弧度
            }
            std::cout<<"pos_rad:"<<i<<":"<<ankle_motor_ret_msg.pos<<std::endl;
            ankle_motor_state_pub[i].publish(ankle_motor_ret_msg);
            //std::cout<<"spd:"<<i<<":"<<ankle_motor_ret_msg.spd<<std::endl;
        }
    }
}


int Can_Channel_Init(int devs, int channel, std::string str)
{
    /*******初始化can通信接口*******/
    int ret;
    Can_Config cancfg;           // can通信配置

    // 打开CAN通信接口
    ret = CAN_OpenDevice(devs, channel);
    if(ret != 0)
    {
        std::cout<<"打开" + str + "通道失败"<<std::endl;
        return -1;
    }
    std::cout<<"打开" + str + "通道成功"<<std::endl;
     // 复位CAN设备
    ret = CAN_Reset(devs, channel);
    if(ret != 0)
    {
        std::cout<<"复位" + str + "电机通道失败！"<<std::endl;
        return -1;
    }
    std::cout<<"复位" + str + "通道成功"<<std::endl;
    // 配置通信结构体
    cancfg.model = 0;
    cancfg.configs = 0;
    cancfg.baudrate = CAN_BAUDRATE;  //设置波特率1M(1000*1000)
    cancfg.configs |= 0x0001;        //接通内部匹配电阻
    cancfg.configs |= 0x0002;        //开启离线唤醒模式
    cancfg.configs |= 0x0004;        //开启自动重传

    ret = CAN_Init(devs, channel, &cancfg);
    if(ret != 0)
    {
        std::cout<<str + "电机初始化失败"<<std::endl;
        return -1;
    }
    std::cout<<str + "初始化成功"<<std::endl;
    CAN_SetFilter(devs,channel,0,0,0,0,1);

    return 0;
}
