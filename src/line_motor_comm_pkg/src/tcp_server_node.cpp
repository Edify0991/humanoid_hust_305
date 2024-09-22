#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sensor_msgs/Imu.h>
#include "line_motor_comm_pkg/hipMotorMsgBack.h"
#include "line_motor_comm_pkg/hipMotorMsgCmd.h"
#include "line_motor_comm_pkg/ankleMotorMsgBack.h"
#include "line_motor_comm_pkg/ankleMotorMsgCmd.h"
#include "line_motor_comm_pkg/linemotorMsgBack.h"
#include "line_motor_comm_pkg/linemotorMsgCmd.h"
#include "line_motor_comm_pkg/allMotorsMsgCmd.h"
#include "line_motor_comm_pkg/allMotorsMsgBack.h"

#define PORT1 1037
#define PORT2 1038
#define PORT3 1039
#define PORT4 1040
#define PORT5 1041
#define PORT6 1042
#define PORT7 1043
#define PORT8 1044
#define PORT9 1045
#define PORT10 1046
#define PORT11 1047

// 定义用于存储多个套接字的全局变量
// std::vector<int> sockets;
int server_fd[11];
int sockets[11];

struct sockaddr_in address;
int addrlen = sizeof(address);

// 创建TCP服务器
int setupServer(int port) {
    int server_fd;
    
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        perror("Setsockopt failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    ROS_INFO("Server setup on port: %d", port);
    return server_fd;
}

// 处理来自客户端的连接
int handleClient(int server_fd) {
    int new_socket;
    if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
        perror("Accept failed");
        return -1;
    }

    ROS_INFO("Client connected on socket: %d", server_fd);
    return new_socket;
    // while(ros::ok()) {
    // // TODO: 在此处理客户端的请求
    // char buffer[1024] = {0};
    // int valread = read(new_socket, buffer, 1024);
    // ROS_INFO("Data received: %s", buffer);
    // }
    
    // close(new_socket);
    // close(socket_fd);
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
    const int num_socket = 10;
    std::ostringstream tcp_oss[num_socket];
    std::string tcp_str[num_socket];
    tcp_oss[0] << "joint" << ankle_motor_pos[0] << "d" << ankle_motor_pos[1] << "end";
    tcp_oss[1] << "joint" << ankle_motor_pos[2] << "d" << ankle_motor_pos[3] << "end";
    tcp_oss[2] << "joint" << hip_motor_pos[0] << "d" << hip_motor_pos[1] << "end";
    tcp_oss[3] << "joint" << hip_motor_pos[2] << "d" << hip_motor_pos[3] << "end";
    tcp_oss[4] << "joint" << knee_motor_pos[0] << "d" << knee_motor_pos[1] << "end";
    tcp_oss[5] << "joint" << ankle_motor_spd[0] << "d" << ankle_motor_spd[1] << "end"; 
    tcp_oss[6] << "joint" << ankle_motor_spd[2] << "d" << ankle_motor_spd[3] << "end";
    tcp_oss[7] << "joint" << hip_motor_spd[0] << "d" << hip_motor_spd[1] << "end";
    tcp_oss[8] << "joint" << hip_motor_spd[2] << "d" << hip_motor_spd[3] << "end";
    tcp_oss[9] << "joint" << knee_motor_spd[0] << "d" << knee_motor_spd[1] << "end";
    
    for (int i = 0; i < num_socket; i++) {
        tcp_str[i] = tcp_oss[i].str();
        // tcp_str[i] = "joint0d0end";
        send(sockets[i], tcp_str[i].c_str(), tcp_str[i].size(), 0);  // 发送字符串数据
        // ROS_INFO("%d",sockets[i]);
        ROS_INFO("Data sent to socket %d: %s", i + 1, tcp_str[i].c_str());
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q;
    tf2::Matrix3x3 m;
    double roll, pitch, yaw;

    // 从msg中获取四元数
    q = tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    // 将四元数转换为RPY角
    m.setRotation(q);
    m.getRPY(roll, pitch, yaw);

    // 将RPY角转换为字符串
    std::ostringstream oss;
    oss.precision(4); // 设置精度
    oss << "joint" << roll << "d" << pitch << "d" << yaw << "end";
    std::string imu_str = oss.str();

    send(sockets[10], imu_str.c_str(), imu_str.size(), 0);  // 发送字符串数据
    ROS_INFO("Data sent to socket %d: %s", 11, imu_str.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_to_tcp_node");
    ros::NodeHandle nh;

    // 设置TCP服务器
    ROS_INFO("开始设置TCP服务器");
    
    // 添加10个服务器端口
    int server_fd1 = setupServer(PORT1);
    int server_fd2 = setupServer(PORT2);
    int server_fd3 = setupServer(PORT3);
    int server_fd4 = setupServer(PORT4);
    int server_fd5 = setupServer(PORT5);
    int server_fd6 = setupServer(PORT6);
    int server_fd7 = setupServer(PORT7);
    int server_fd8 = setupServer(PORT8);
    int server_fd9 = setupServer(PORT9);
    int server_fd10 = setupServer(PORT10);
    int server_fd11 = setupServer(PORT11);
    
    // 将所有套接字存储在sockets中
    server_fd[0] = server_fd1;
    server_fd[1] = server_fd2;
    server_fd[2] = server_fd3;
    server_fd[3] = server_fd4;
    server_fd[4] = server_fd5;
    server_fd[5] = server_fd6;
    server_fd[6] = server_fd7;
    server_fd[7] = server_fd8;
    server_fd[8] = server_fd9;
    server_fd[9] = server_fd10;
    server_fd[10] = server_fd11;

    // sockets.push_back(server_fd1);
    // sockets.push_back(server_fd2);
    // sockets.push_back(server_fd3);
    // sockets.push_back(server_fd4);
    // sockets.push_back(server_fd5);
    // sockets.push_back(server_fd6);
    // sockets.push_back(server_fd7);
    // sockets.push_back(server_fd8);
    // sockets.push_back(server_fd9);
    // sockets.push_back(server_fd10);
    for(int i = 0;i<11;i++){
        sockets[i] = handleClient(server_fd[i]);
    }

    ROS_INFO("TCP服务器设置完毕");

    // 订阅话题
    ros::Subscriber sub = nh.subscribe("/motor_to_rl", 1000, motorCallback);
    ros::Subscriber sub2 = nh.subscribe("/imu_data", 1000, imuCallback);

    // // 启动线程处理每个端口的客户端连接
    // std::thread clientThread1(handleClient, server_fd1);
    // std::thread clientThread2(handleClient, server_fd2);
    // std::thread clientThread3(handleClient, server_fd3);
    // std::thread clientThread4(handleClient, server_fd4);
    // std::thread clientThread5(handleClient, server_fd5);
    // std::thread clientThread6(handleClient, server_fd6);
    // std::thread clientThread7(handleClient, server_fd7);
    // std::thread clientThread8(handleClient, server_fd8);
    // std::thread clientThread9(handleClient, server_fd9);
    // std::thread clientThread10(handleClient, server_fd10);

    // 设置循环频率为10Hz
    ros::Rate loop_rate(10);

    // ROS主循环
    while (ros::ok()) {

        // 使用两个线程的AsyncSpinner
        ros::AsyncSpinner spinner(2); // 使用2个线程
        spinner.start();

        ros::spinOnce();  // 处理回调函数
        
        // 使用 loop_rate 控制循环频率
        loop_rate.sleep();
    }

    // 关闭套接字
    // for (auto& sock : sockets) {
    //     close(sock);
    // }
    for (int i = 0; i < 11; i++) {
        close(sockets[i]);
        close(server_fd[i]);
    }

    // // 等待所有线程结束
    // clientThread1.join();
    // clientThread2.join();
    // clientThread3.join();
    // clientThread4.join();
    // clientThread5.join();
    // clientThread6.join();
    // clientThread7.join();
    // clientThread8.join();
    // clientThread9.join();
    // clientThread10.join();

    return 0;
}
