
#ifndef CANBUS_H
#define CANBUS_H

#include <stdio.h>

#if defined(_WIN32) || defined(__CYGWIN__) || defined(_WIN32_WCE)
#define LIBUSB_CALL __stdcall
#else
#define LIBUSB_CALL
#endif

typedef unsigned int u32 ;
typedef unsigned char u8 ;

typedef  struct  _Can_Msg {
    u32             ID;
    u32             TimeStamp;
    char            FrameType;
    char            DataLen;
    char	        Data[8];
    char	        ExternFlag;
    char	        RemoteFlag;
    char            BusSatus;
    char            ErrSatus;
    char            TECounter;
    char            RECounter;
}Can_Msg, *P_Can_Msg;

typedef struct _Can_Status {
    char            BusSatus;
    char            ErrSatus;
    char            TECounter;
    char            RECounter;
}Can_Status, *P_Can_Status;

typedef struct _Can_Config {
    u32             baudrate;
    u32             configs;
    u32             model;
}Can_Config, *P_Can_Config;


#ifdef __cplusplus
extern "C" {
#endif

// devNum表示设备索引号，chNum表示通道号  当插入一个双通道CAN设备时 devNum为0， chNum为0或者1；
//  当插入N个双通道CAN设备时 devNum为0至(N-1)， chNum为0或者1；

int LIBUSB_CALL  Reg_HotPlug_Func(void(*pfunc)(void));
int LIBUSB_CALL  CAN_ScanDevice(void);
int LIBUSB_CALL  CAN_OpenDevice(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_CloseDevice(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_Init(u32 devNum, u32 chNum, P_Can_Config pInitConfig);
int LIBUSB_CALL  CAN_SetFilter(u32 devNum, u32 chNum, char namber, char type, u32 ftID, u32 ftMask, char enable);
int LIBUSB_CALL  CAN_Reset(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_GetReceiveNum(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_Transmit(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 items, u32 timeou);
int LIBUSB_CALL  CAN_Receive(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 Len, u32 timeou);
int LIBUSB_CALL  CAN_GetStatus(u32 devNum, u32 chNum, P_Can_Status status);
#ifdef __cplusplus
}
#endif

#endif



