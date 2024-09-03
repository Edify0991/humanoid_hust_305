#include "serialPort/SerialPort.h"
#include "ros/callback_queue.h"
#include "line_motor_comm_pkg/hipMotorMsgCmd.h"
#include "line_motor_comm_pkg/hipMotorMsgBack.h"
#include "ros/ros.h"
#include <thread>
#include <vector>
#include "Modbus_Comm.h" // 直线电机

/* 绑定串口设备 0827前
KERNELS=="3-5.3.1:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_ankle_left"
KERNELS=="3-5.3.4.4:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_ankle_right"
KERNELS=="3-5.3.3.2:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_hip_left"
KERNELS=="3-5.3.3.4:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_hip_right"
KERNELS=="3-5.3.2:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_imu"

KERNELS=="3-8.1:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_ankle_ll"
KERNELS=="3-5.3.1:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_ankle_lr"
KERNELS=="3-5.3.4.4:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_ankle_rl"
KERNELS=="3-8.3.1:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_ankle_rr"
KERNELS=="3-5.3.3.2:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_hip_ll"
KERNELS=="3-5.4:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_hip_lb"
KERNELS=="3-5.3.3.4:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_hip_rr"
KERNELS=="3-8.3.4:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_hip_rb"
*/
// 是否控制电机
const bool control_motor = true;
// 定义所有的电机的名字 第一个l/r表示左右腿，第二个l/r/f/b表示在腿上的位置
std::vector<std::string> motor_name_ = { 
                                // "imu",
								"hip_lb",
                                "hip_ll",  
                                "hip_rb",
                                "hip_rr", 
                                };

std::vector<std::thread> threads;
std::vector<ros::Publisher> motor_state_pub;
std::vector<SerialPort*> motor_ports;

auto start_time = std::chrono::high_resolution_clock::now();
void MotorCmdCallback(const line_motor_comm_pkg::hipMotorMsgCmd::ConstPtr& msg, int motor_id)
{
    // 发送数据回调, msg文件里面的所有数据均为减速器输出端的输出值
    MotorCmd motor_cmd;
    motor_cmd.id = msg->id;
    motor_cmd.motorType = static_cast<MotorType>(msg->motorType);
    motor_cmd.mode = msg->mode;
    motor_cmd.tau = msg->tau / queryGearRatio(MotorType::B1);
    motor_cmd.dq = msg->dq * queryGearRatio(MotorType::B1);
    motor_cmd.q = msg->q * queryGearRatio(MotorType::B1);
    motor_cmd.kp = msg->kp; // 这里待确定
    motor_cmd.kd = msg->kd;
    MotorData motor_ret;
    motor_ret.motorType = static_cast<MotorType>(msg->motorType);
    try {
        if (control_motor) {
            // 尝试发送和接收数据
            motor_ports[motor_id]->sendRecv(&motor_cmd, &motor_ret);
        }
    } catch (const std::exception& e) {
        // 捕获所有从 std::exception 派生的异常
        std::cerr << "串口发生错误,其id: " << motor_id << std::endl;
        // 这里可以添加更多的错误处理代码，例如重试、记录日志、清理资源等
    } catch (...) {
        // 捕获所有其他类型的异常
        std::cerr << "串口发生错误,其id: " << motor_id << std::endl;
        // 同样可以添加错误处理代码
    }

    // if(motor_id==5)
    // {
    //     auto end_time = std::chrono::high_resolution_clock::now();
    //     auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time);
    //     int duration_count = duration_us.count();
    //     std::cout << "motor_" << (int)motor_id << ": "<< duration_count << std::endl;
    //     start_time = end_time;
    // }
    

    line_motor_comm_pkg::hipMotorMsgBack motor_ret_msg;
    motor_ret_msg.id = motor_id;
    motor_ret_msg.tau = motor_ret.tau * queryGearRatio(MotorType::B1);
    motor_ret_msg.dq = motor_ret.dq / queryGearRatio(MotorType::B1);
    motor_ret_msg.q = motor_ret.q / queryGearRatio(MotorType::B1);
    
    motor_state_pub[motor_id].publish(motor_ret_msg);
    
    
}

// 宇树电机线程，用于接收电机的命令
void UnitreeMotor_CmdThread(int motor_id, ros::NodeHandle nh)
{
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    ros::Subscriber sub = nh.subscribe<line_motor_comm_pkg::hipMotorMsgCmd>(
            motor_name_[motor_id] + "_cmd", 
            1, 
            boost::bind(MotorCmdCallback, 
            _1, 
            motor_id));
    while (ros::ok())
    {
        queue.callAvailable(ros::WallDuration());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;

    threads.resize(motor_name_.size());
    motor_state_pub.resize(motor_name_.size());

    // 初始化串口
    for(int i=0;i<motor_name_.size();i++)
    {

        motor_ports.push_back(new SerialPort("/dev/usb_" + motor_name_[i]));
        motor_state_pub[i] = nh.advertise<line_motor_comm_pkg::hipMotorMsgBack>(motor_name_[i] + "_state", 1);
        threads[i] = std::thread(UnitreeMotor_CmdThread, i, std::ref(nh));    
    }

    std::cout << "所有电机已启动" << std::endl;

    for(auto& thread : threads)
    {
        thread.detach();
    }
    while(ros::ok()) { 
        usleep(1000000);
    }
    return 0;
}