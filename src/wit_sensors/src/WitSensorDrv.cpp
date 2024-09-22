#include "WitSensorDrv.h"

using namespace std;

WitSensor::WitSensor()
{
    data_ready = 0;
    serial::Timeout _time =serial::Timeout::simpleTimeout(2000); 
    // serial_port.setPort("/dev/USB_imu");
    serial_port.setPort("/dev/ttyUSB22");
    serial_port.open();
    if(serial_port.isOpen())
    {
        serial_port.setBaudrate(115200);
        serial_port.setStopbits(serial::stopbits_one);
        serial_port.setParity(serial::parity_none);
        serial_port.setTimeout(_time);
        cout<<"Sensor serialport has been opened!"<<endl;
    }
    else
    {
        ROS_ERROR_STREAM("Sensor serialport open failed!");
    }
}

void WitSensor::ReadHoldReg(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = READ_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = AX;
    send_data[DATALH] = 0X00;
    send_data[DATALL] = READ_LEN;
    send_data[CRCH] = 0X09;
    send_data[CRCL] = 0X80;
    serial_port.write(&send_data[0],sizeof(send_data));
}

void WitSensor::ReadAcc(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = READ_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0X34;
    send_data[DATALH] = 0X00;
    send_data[DATALL] = 0X03;
    send_data[CRCH] = 0X49;
    send_data[CRCL] = 0X84;
    serial_port.write(&send_data[0],sizeof(send_data));
}

void WitSensor::ReadAngSpeed(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = READ_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0X37;
    send_data[DATALH] = 0X00;
    send_data[DATALL] = 0X03;
    send_data[CRCH] = 0XB9;
    send_data[CRCL] = 0X84;
    serial_port.write(send_data,sizeof(send_data));
}

void WitSensor::ReadAng(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = READ_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0X3D;
    send_data[DATALH] = 0X00;
    send_data[DATALL] = 0X03;
    send_data[CRCH] = 0X99;
    send_data[CRCL] = 0X86;
    serial_port.write(send_data,sizeof(send_data));
}

void WitSensor::UnlockSensor(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = WRITE_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0x69;
    send_data[DATALH] = 0xb5;
    send_data[DATALL] = 0x88;
    send_data[CRCH] = 0x22;
    send_data[CRCL] = 0xa1;
    serial_port.write(send_data,sizeof(send_data));
}

void WitSensor::AccCaliFcn(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = WRITE_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0x01;
    send_data[DATALH] = 0x00;
    send_data[DATALL] = 0x01;
    send_data[CRCH] = 0x14;
    send_data[CRCL] = 0x4B;
    serial_port.write(send_data,sizeof(send_data));
}

void WitSensor::AngCaliFcn(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = WRITE_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0x01;
    send_data[DATALH] = 0x00;
    send_data[DATALL] = 0x08;
    send_data[CRCH] = 0xD4;
    send_data[CRCL] = 0x4D;
    serial_port.write(send_data,sizeof(send_data));
}

void WitSensor::GyroCaliFcn(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = WRITE_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0x61;
    send_data[DATALH] = 0x00;
    send_data[DATALL] = 0x01;
    send_data[CRCH] = 0x14;
    send_data[CRCL] = 0x55;
    serial_port.write(send_data,sizeof(send_data));
}

void WitSensor::WriteEeprom(void)
{
    send_data[ADDR] = 0x50;
    send_data[CMD] = WRITE_DATA;
    send_data[REG_H] = 0X00;
    send_data[REG_L] = 0x00;
    send_data[DATALH] = 0x00;
    send_data[DATALL] = 0x00;
    send_data[CRCH] = 0x84;
    send_data[CRCL] = 0x4B;
    serial_port.write(send_data,sizeof(send_data));
}

void WitSensor::ExtractData(void)
{
    acc_XYZ[X] = ((short)rcv_buffer[3]<<8)|rcv_buffer[4];
    acc_XYZ[Y] = ((short)rcv_buffer[5]<<8)|rcv_buffer[6];
    acc_XYZ[Z] = ((short)rcv_buffer[7]<<8)|rcv_buffer[8];

    dang_XYZ[X] = ((short)rcv_buffer[9]<<8)|rcv_buffer[10];
    dang_XYZ[Y] = ((short)rcv_buffer[11]<<8)|rcv_buffer[12];
    dang_XYZ[Z] = ((short)rcv_buffer[13]<<8)|rcv_buffer[14];

    mag_XYZ[X] = ((short)rcv_buffer[15]<<8)|rcv_buffer[16];
    mag_XYZ[Y] = ((short)rcv_buffer[17]<<8)|rcv_buffer[18];
    mag_XYZ[Z] = ((short)rcv_buffer[19]<<8)|rcv_buffer[20];

    ang_XYZ[X] = ((short)rcv_buffer[21]<<8)|rcv_buffer[22];
    ang_XYZ[Y] = ((short)rcv_buffer[23]<<8)|rcv_buffer[24];
    ang_XYZ[Z] = ((short)rcv_buffer[25]<<8)|rcv_buffer[26];
}

void WitSensor::StartCalibrating(void)
{
    // 开启加速度计校准
    UnlockSensor();
    usleep(100000);
    AccCaliFcn();
    sleep(2);
    WriteEeprom();
    usleep(10000);
    // 开启角度校准
    UnlockSensor();
    usleep(100000);
    AccCaliFcn();
    usleep(100000);
    WriteEeprom();
    usleep(10000);
    // 开启陀螺仪校准
    UnlockSensor();
    usleep(100000);
    AccCaliFcn();
    usleep(10000);
    WriteEeprom();
    usleep(10000);
    cout<<"校准完成！"<<endl;
}

