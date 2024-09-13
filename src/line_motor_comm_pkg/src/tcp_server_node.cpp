#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "line_motor_comm_pkg/hipMotorMsgBack.h"
#include "line_motor_comm_pkg/hipMotorMsgCmd.h"
#include "line_motor_comm_pkg/ankleMotorMsgBack.h"
#include "line_motor_comm_pkg/ankleMotorMsgCmd.h"
#include "line_motor_comm_pkg/linemotorMsgBack.h"
#include "line_motor_comm_pkg/linemotorMsgCmd.h"
#include "line_motor_comm_pkg/allMotorsMsgCmd.h"
#include "line_motor_comm_pkg/allMotorsMsgBack.h"


#define PORT 1037

int server_fd, new_socket;
struct sockaddr_in address;
int addrlen = sizeof(address);

// 创建TCP服务器
void setupServer() {
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
		ROS_INFO("Socket creation");
    }
    
	int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        perror("Setsockopt failed");
        close(server_fd);
        exit(EXIT_FAILURE);
		ROS_INFO("Setsockopt");
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
		ROS_INFO("bind");
    }
    
    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
		ROS_INFO("listen");
    }
    
    if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
        perror("Accept failed");
        exit(EXIT_FAILURE);
		ROS_INFO("accept");
    }
}

// 订阅回调函数，将接收到的数据通过TCP发送出去
void motorCallback(const line_motor_comm_pkg::allMotorsMsgBack::ConstPtr& msg) {
    float ankle_motor_pos[4];
    ankle_motor_pos[0] = msg->ankle_motor_ll_back.pos;
    ankle_motor_pos[1] = msg->ankle_motor_lr_back.pos;
    ankle_motor_pos[2] = msg->ankle_motor_rl_back.pos;
    ankle_motor_pos[3] = msg->ankle_motor_rr_back.pos;
    float hip_motor_pos[4];
    hip_motor_pos[0] = msg->hip_motor_l_pitch_back.q;
    hip_motor_pos[1] = msg->hip_motor_l_rotate_back.q;
    hip_motor_pos[2] = msg->hip_motor_r_pitch_back.q;
    hip_motor_pos[3] = msg->hip_motor_r_rotate_back.q;
    float knee_motor_pos[2];
    knee_motor_pos[0] = msg->line_motor_l_back.x;
    knee_motor_pos[1] = msg->line_motor_r_back.x;
    float ankle_motor_spd[4];
    ankle_motor_spd[0] = msg->ankle_motor_ll_back.spd;
    ankle_motor_spd[1] = msg->ankle_motor_lr_back.spd;
    ankle_motor_spd[2] = msg->ankle_motor_rl_back.spd;
    ankle_motor_spd[3] = msg->ankle_motor_rr_back.spd;
    float hip_motor_spd[4];
    hip_motor_spd[0] = msg->hip_motor_l_pitch_back.dq;
    hip_motor_spd[1] = msg->hip_motor_l_rotate_back.dq;
    hip_motor_spd[2] = msg->hip_motor_r_pitch_back.dq;
    hip_motor_spd[3] = msg->hip_motor_r_rotate_back.dq;
    float knee_motor_spd[2];
    knee_motor_spd[0] = msg->line_motor_l_back.dx;
    knee_motor_spd[1] = msg->line_motor_r_back.dx;
    
    // 将浮点数转换为字符串
    std::ostringstream oss;
    oss << "joint" << ankle_motor_pos[0] << "d" << ankle_motor_pos[1] << "d" << ankle_motor_pos[2] << "d" << ankle_motor_pos[3] << "d" << hip_motor_pos[0] << "d" << hip_motor_pos[1] << "d" << hip_motor_pos[2] << "d" << hip_motor_pos[3] << "d" << knee_motor_pos[0] << "d" << knee_motor_pos[1] << "d" << ankle_motor_spd[0] << "d" << ankle_motor_spd[1] << "d" << ankle_motor_spd[2] << "d" << ankle_motor_spd[3] << "d" << hip_motor_spd[0] << "d" << hip_motor_spd[1] << "d" << hip_motor_spd[2] << "d" << hip_motor_spd[3] << "d" << knee_motor_spd[0] << "d" << knee_motor_spd[1] << "end";
    std::string motor_value_str = oss.str();
    
    // 通过Socket发送数据
    send(new_socket, motor_value_str.c_str(), motor_value_str.size(), 0);
    ROS_INFO("Data sent: %s", motor_value_str.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_to_tcp_node");
    ros::NodeHandle nh;
    
    // 设置TCP服务器
	ROS_INFO("开始设置tcp服务器");
    setupServer();
	ROS_INFO("tcp服务器设置完毕");
    
    // // 订阅话题
    ros::Subscriber sub = nh.subscribe("/motor_to_rl", 1000, motorCallback);
    
    // 设置循环频率为100Hz
    ros::Rate loop_rate(10);
    
    // ROS主循环
    while (ros::ok()) {
        ros::spinOnce(); // 处理回调函数
        
        // 根据设置的频率休眠
        loop_rate.sleep();
    }
    
    // 关闭socket
    close(new_socket);
    close(server_fd);
    
    return 0;
}
