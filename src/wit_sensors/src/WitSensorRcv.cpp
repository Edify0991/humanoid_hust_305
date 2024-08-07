#include "WitSensorDrv.h"

std::vector<uint8_t> data_buff;

void *ReceiveWitSensor(void* param)
{
    int *run = (int*)param;
    while((*run) & 0x0f)
    {
        if(hwt901b.serial_port.available())
        {
            uint8_t data;
            hwt901b.serial_port.read(&data, 1);
            data_buff.push_back(data);
            if(data_buff.size() >= 29 && data_buff[0] == 0x50 && data_buff[1] == 0x03)
            {
                // 获取全部数据
                hwt901b.rcv_buffer.clear();
                hwt901b.rcv_buffer.swap(data_buff);
                hwt901b.data_ready = 1;
            }
            else if(data_buff[0] != 0x50)
            {
                data_buff.clear();
            }
            else if(data_buff.size() > 2 && data_buff[1] != 0x03)
            {
                data_buff.clear();
            }
        }
    }
    printf("sensor thread exit\n");
    pthread_exit(0);
}
