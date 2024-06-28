#include "Modbus_Comm.h"

using namespace std;

serial::Serial LineMotorSerial;

unsigned char sendBuffer[100];
unsigned char recvBuffer[100];

string LineMotorSerialPort = "/dev/ttyUSB0";
int Baudrate = 25600;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_motor_comm_node");
    ros::NodeHandle nh("~");

    // 测试用例
    LineMotor LeftMotor(1, 100);
    //LeftMotor.Enable_VDI(&LineMotorSerial);

    try
    {
        LineMotorSerial.setPort(LineMotorSerialPort);
        LineMotorSerial.setBaudrate(Baudrate);
        serial::Timeout TimeOutValue = serial::Timeout::simpleTimeout(1000);
        LineMotorSerial.setTimeout(TimeOutValue);
        LineMotorSerial.open();        
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open the serial port!");
        return -1;
    }

    if(LineMotorSerial.isOpen())
    {
        ROS_INFO_STREAM("Serial port has been initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(5); // 5Hz
    while(ros::ok())
    {
        //LineMotorSerial.write();
    }

    return 0;
}
