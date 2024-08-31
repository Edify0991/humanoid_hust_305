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
                                "ankle_motor_ll",
                                "ankle_motor_lr",
                                "ankle_motor_rl",
                                "ankle_motor_rr"
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

// 踝关节电机接收到控制命令回调
void AnkleMotor_CmdCallback(const line_motor_comm_pkg::ankleMotorMsgCmd::ConstPtr& msg, int ankle_motor_thread_id)
{
    int state = msg->motor_state;
    float kp = msg->kp;
    float kd = msg->kd;
    float pos = msg->pos; // 传进来的为角度值
    float spd = msg->spd; // 传进来的为°/s
    float tau = msg->tau; // 传进来的为牛米
    float limit_current = msg->limit_current * 10.0f; // 传进来的为限制电流大小, 
    // 使用send_motor_ctrl_cmd，传入参数为弧度制
    float pos_rad = pos / 360.0f * 2 * PI;
    float spd_rad = spd / 360.0f * 2 * PI;
    // 不同的指令有不同的倍率
    float spd_x10 = spd * 10.0f;
    // 待加入电流与转矩的转换关系公式
    // spd和pos单位都是弧度制, 控制指令发送
    if(is_ankle_motor_control) 
    {
        switch(state)
        {
            case ZEROS_SETTING_STEP:
                ankle_motor_ports[ankle_motor_thread_id]->MotorSetting(SETTING_ZERO_CMD);
                sleep(1);
            break;
            case MIXED_CONTROL_STEP:
                ankle_motor_ports[ankle_motor_thread_id]->send_motor_ctrl_cmd(kp, kd, pos_rad, spd_rad, tau);
            break;
            case POS_CONTROL_STEP:
                ankle_motor_ports[ankle_motor_thread_id]->set_motor_position(pos, spd_x10, limit_current, MIX_CONTROL_ACK);
            break;
            case SPEED_CONTROL_STEP:
                ankle_motor_ports[ankle_motor_thread_id]->set_motor_speed(spd, limit_current, MIX_CONTROL_ACK);
            break;
            default:
            break;
        }
    }
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
    // Can_Channel_Init(ankle_motor_usb[2], ankle_motor_channel[2], ankle_canDevs_name[2]);
    // Can_Channel_Init(ankle_motor_usb[3], ankle_motor_channel[3], ankle_canDevs_name[3]);
    
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
    // std::thread AnkleMotor_rl_RcvThread(std::bind(AnkleMotor_Rcv_Fcn, ankle_motor_usb[2], ankle_motor_channel[2]));
    // if(AnkleMotor_rl_RcvThread.joinable())
    // {
    //     std::cout << "右侧左边脚踝电机接收线程创建成功" << std::endl;
    //     AnkleMotor_rl_RcvThread.detach();
    // } else {
    //     std::cerr << "右侧左边脚踝电机接收线程创建失败" << std::endl;
    // }
    // std::thread AnkleMotor_rr_RcvThread(std::bind(AnkleMotor_Rcv_Fcn, ankle_motor_usb[3], ankle_motor_channel[3]));
    // if(AnkleMotor_rr_RcvThread.joinable())
    // {
    //     std::cout << "右侧右边脚踝电机接收线程创建成功" << std::endl;
    //     AnkleMotor_rr_RcvThread.detach();
    // } else {
    //     std::cerr << "右侧右边脚踝电机接收线程创建失败" << std::endl;
    // }

    // 初始化踝关节电机类
    for(uint8_t i = 0; i < ankle_motor_name.size(); i++)
    {
        ankle_motor* ptr = new ankle_motor(ankle_motor_usb[i], ankle_motor_channel[i], ANKLE_MOTOR_ID);
        ankle_motor_ports.push_back(ptr);
        ankle_motor_state_pub[i] = nh.advertise<line_motor_comm_pkg::ankleMotorMsgBack>(ankle_motor_name[i] + "_state", 1);
        ankle_motor_threads[i] = std::thread(AnkleMotor_CmdThread, i, std::ref(nh));
    }

    for(auto& thread : ankle_motor_threads)
    {
        thread.join();
    }

    return 0;
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
                ankle_motor_ret_msg.tau = ankle_motor_ports[i]->rv_motor_msg[0].current_actual_float * ANKLE_MOTOR_KT;
                ankle_motor_ret_msg.spd = ankle_motor_ports[i]->rv_motor_msg[0].speed_actual_rad;
                ankle_motor_ret_msg.pos = ankle_motor_ports[i]->rv_motor_msg[0].angle_actual_rad;
            }
            ankle_motor_state_pub[i].publish(ankle_motor_ret_msg);
            std::cout<<"spd:"<<i<<":"<<ankle_motor_ret_msg.spd<<std::endl;
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
