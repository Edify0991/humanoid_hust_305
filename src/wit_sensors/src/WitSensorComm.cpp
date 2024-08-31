#include "WitSensorDrv.h"
#include <signal.h>

using namespace std;

void SignalHandler(int signal);

pthread_t thread_id;
int comm_run = 1;

volatile float acceleration[3] = {0};
volatile float angular_vel[3] = {0};
volatile float magnetometer[3] = {0};
volatile float angular[3] = {0};
tf::Quaternion q(0,0,0,0);

WitSensor hwt901b; // 初始化串口

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wit_sensor_node");
    ros::NodeHandle nh;

    signal(SIGINT, SignalHandler);

    // 创建发布者
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag_data", 10);
    //hwt901b.StartCalibrating();
    sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
    sensor_msgs::MagneticField mag_msg = sensor_msgs::MagneticField();

     // 开始校准IMU数据
    hwt901b.StartCalibrating();
    sleep(4);
    // 创建接收姿态传感器数据线程
    int ret;
    ret = pthread_create(&thread_id, NULL, ReceiveWitSensor, &comm_run);
    if(!ret)
    {
        cout<<"姿态传感器接收线程创建成功"<<endl;
    }
    ros::Rate loop_rate(20); // 20Hz，50ms读取一次数据
    while(ros::ok)
    {
        if(hwt901b.data_ready)
        {
            hwt901b.ExtractData();
            // 数据换算
            for(int i = 0; i < 3; i++)
            {
                acceleration[i] = hwt901b.acc_XYZ[i] / 32768.0f * 16.0f * 9.8f;
                angular_vel[i] = hwt901b.dang_XYZ[i] / 32768.0f * 2000 * PI / 180;
                magnetometer[i] = hwt901b.mag_XYZ[i]; 
                angular[i] = hwt901b.ang_XYZ[i] / 32768.0f * PI;  
            }
            // 发布数据
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "base_link";
            mag_msg.header.stamp = ros::Time::now();
            mag_msg.header.frame_id = "base_link";
            
            q.setRPY(angular[0], angular[1], angular[2]);
            imu_msg.orientation.x = q[0];
            imu_msg.orientation.y = q[1];
            imu_msg.orientation.z = q[2];
            imu_msg.orientation.w = q[3];

            imu_msg.angular_velocity.x = angular_vel[0];
            imu_msg.angular_velocity.y = angular_vel[1];
            imu_msg.angular_velocity.z = angular_vel[2];

            imu_msg.linear_acceleration.x = acceleration[0];
            imu_msg.linear_acceleration.y = acceleration[1];
            imu_msg.linear_acceleration.z = acceleration[2];

            mag_msg.magnetic_field.x = magnetometer[0];
            mag_msg.magnetic_field.y = magnetometer[1];
            mag_msg.magnetic_field.z = magnetometer[2];

            imu_pub.publish(imu_msg);
            mag_pub.publish(mag_msg);

            ROS_INFO("x:%f",imu_msg.orientation.x);
            ROS_INFO("y:%f",imu_msg.orientation.y);
            ROS_INFO("z:%f",imu_msg.orientation.z);
            ROS_INFO("w:%f",imu_msg.orientation.w);

            ROS_INFO("roll:%f",angular[0]);
            ROS_INFO("pitch:%f",angular[1]);
            ROS_INFO("yaw:%f",angular[2]);

            ROS_INFO("roll_vel:%f",angular_vel[0]);
            ROS_INFO("pitch_vel:%f",angular_vel[1]);
            ROS_INFO("yaw_vel:%f",angular_vel[2]);

            ROS_INFO("mag_x:%f",magnetometer[0]);
            ROS_INFO("mag_y:%f",magnetometer[1]);
            ROS_INFO("mag_z:%f",magnetometer[2]);

            ROS_INFO("accx:%f",imu_msg.linear_acceleration.x);
            ROS_INFO("accy:%f",imu_msg.linear_acceleration.y);
            ROS_INFO("accz:%f",imu_msg.linear_acceleration.z);

            hwt901b.data_ready = 0;
        }
        hwt901b.ReadHoldReg();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void SignalHandler(int signal)
{
	comm_run = 0;
	ros::shutdown();
}