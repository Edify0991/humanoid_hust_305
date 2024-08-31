#include "ankle_comm.h"
#include "math_ops.h"
//123
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f
union RV_TypeConvert
{
	float to_float;
	int to_int;
	unsigned int to_uint;
	uint8_t buf[4];
}rv_type_convert;

union RV_TypeConvert2
{
	int16_t to_int16;
	uint16_t to_uint16;
	uint8_t buf[2];
}rv_type_convert2;

ankle_motor::ankle_motor(int usb, int channel, uint16_t id)
{
	usb_num = usb;
	can_channel = channel;
	motor_id = id;
}

ankle_motor::~ankle_motor(){}

//MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void ankle_motor::MotorSetting(uint8_t cmd)
{
	Can_Msg tx_msg;
	tx_msg.ID = 0x7FF;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 4;
	tx_msg.Data[0] = motor_id >> 8;
	tx_msg.Data[1] = motor_id & 0xff;
	tx_msg.Data[2] = 0;
	tx_msg.Data[3] = cmd;
	
	CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

void ankle_motor::MotorIDReset(void)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 6;
	
    tx_msg.Data[0]=0x7F;
	tx_msg.Data[1]=0x7F;
	tx_msg.Data[2]=0x00;
	tx_msg.Data[3]=0x05;
	tx_msg.Data[4]=0x7F;
	tx_msg.Data[5]=0x7F;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

void ankle_motor::MotorIDReading(void)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 4;
	
    tx_msg.Data[0]=0xFF;
	tx_msg.Data[1]=0xFF;
	tx_msg.Data[2]=0x00;
	tx_msg.Data[3]=0x82;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void ankle_motor::send_motor_ctrl_cmd(float kp,float kd,float pos,float spd,float tor)
{
	int kp_int;
	int kd_int;
	int pos_int;            
	int spd_int;
	int tor_int;
	
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 8;
		
	if(kp > KP_MAX) kp = KP_MAX;
		else if(kp < KP_MIN) kp = KP_MIN;
	if(kd > KD_MAX ) kd = KD_MAX;
		else if(kd < KD_MIN) kd = KD_MIN;	
	if(pos > POS_MAX)	pos = POS_MAX;
		else if(pos < POS_MIN) pos = POS_MIN;
	if(spd > SPD_MAX)	spd = SPD_MAX;
		else if(spd < SPD_MIN) spd = SPD_MIN;
	if(tor > T_MAX)	tor = T_MAX;
		else if(tor < T_MIN) tor = T_MIN;

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
	pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);            
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);	
	
    tx_msg.Data[0] = 0x00|(kp_int>>7);//kp5
	tx_msg.Data[1] = ((kp_int&0x7F)<<1)|((kd_int&0x100)>>8);//kp7+kd1
	tx_msg.Data[2] = kd_int&0xFF;
	tx_msg.Data[3] = pos_int>>8;
	tx_msg.Data[4] = pos_int&0xFF;
	tx_msg.Data[5] = spd_int>>4;
	tx_msg.Data[6] = (spd_int&0x0F)<<4|(tor_int>>8);
	tx_msg.Data[7] = tor_int&0xff;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
pos:float
spd:0~18000
cur:0~3000
ack_status:0~3
*/
void ankle_motor::set_motor_position(float pos,uint16_t spd,uint16_t cur,uint8_t ack_status)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 8;

	if(ack_status>3) 
		return;
	
	rv_type_convert.to_float = pos;
    tx_msg.Data[0] = 0x20|(rv_type_convert.buf[3]>>3);
	tx_msg.Data[1] = (rv_type_convert.buf[3]<<5)|(rv_type_convert.buf[2]>>3);
	tx_msg.Data[2] = (rv_type_convert.buf[2]<<5)|(rv_type_convert.buf[1]>>3);
	tx_msg.Data[3] = (rv_type_convert.buf[1]<<5)|(rv_type_convert.buf[0]>>3);
	tx_msg.Data[4] = (rv_type_convert.buf[0]<<5)|(spd>>10);
	tx_msg.Data[5] = (spd&0x3FC)>>2;
	tx_msg.Data[6] = (spd&0x03)<<6|(cur>>6);
	tx_msg.Data[7] = (cur&0x3F)<<2|ack_status;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
spd:-18000~18000
cur:0~3000
ack_status:0~3
*/
void ankle_motor::set_motor_speed(float spd,uint16_t cur,uint8_t ack_status)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 7;

	rv_type_convert.to_float = spd;
    tx_msg.Data[0] = 0x40|ack_status;
	tx_msg.Data[1] = rv_type_convert.buf[3];
	tx_msg.Data[2] = rv_type_convert.buf[2];
	tx_msg.Data[3] = rv_type_convert.buf[1];
	tx_msg.Data[4] = rv_type_convert.buf[0];
	tx_msg.Data[5] = cur>>8;
	tx_msg.Data[6] = cur&0xff;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

void ankle_motor::set_motor_cur_tor(int16_t cur_tor,uint8_t ctrl_status,uint8_t ack_status)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 3;

	if(ack_status > 3) 
		return;
	if(ctrl_status > 7)
		return;
	if(ctrl_status) //enter torque control mode or brake mode
	{
		if(cur_tor > 3000) cur_tor = 3000;
		else if(cur_tor < -3000) cur_tor = -3000;
	}
	else
	{
		if(cur_tor > 2000) cur_tor = 2000;
		else if(cur_tor < -2000) cur_tor = -2000;
	}
	
    tx_msg.Data[0] = 0x60|ctrl_status<<2|ack_status;
	tx_msg.Data[1] = cur_tor>>8;
	tx_msg.Data[2] = cur_tor&0xff;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}
// This function use in ask communication mode.
/*
motor_id:1~0x7FE
acc:0~2000
ack_status:0~3
*/
void ankle_motor::set_motor_acceleration(uint16_t acc,uint8_t ack_status)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 4;

	if(ack_status>2) 
		return;
	if(acc>2000) acc=2000;
	
    tx_msg.Data[0]=0xC0|ack_status;
	tx_msg.Data[1]=0x01;
	tx_msg.Data[2]=acc>>8;
	tx_msg.Data[3]=acc&0xff;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}
// This function use in ask communication mode.
/*
motor_id:1~0x7FE
linkage:0~10000
speedKI:0~10000
ack_status:0/1
*/
void ankle_motor::set_motor_linkage_speedKI(uint16_t linkage,uint16_t speedKI,uint8_t ack_status)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 6;

	if(ack_status>2) 
		return;
	if(linkage>10000) linkage=10000;
	if(speedKI>10000) speedKI=10000;

    tx_msg.Data[0] = 0xC0|ack_status;
	tx_msg.Data[1] = 0x02;
	tx_msg.Data[2] = linkage>>8;
	tx_msg.Data[3] = linkage&0xff;
	tx_msg.Data[4] = speedKI>>8;
	tx_msg.Data[5] = speedKI&0xff;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}
// This function use in ask communication mode.
/*
motor_id:1~0x7FE
fdbKP:0~10000
fbdKD:0~10000
ack_status:0/1
*/
void ankle_motor::set_motor_feedbackKP_KD(uint16_t fdbKP,uint16_t fdbKD,uint8_t ack_status)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 6;

	if(ack_status>2) 
		return;
	if(fdbKP>10000) fdbKP=10000;
	if(fdbKD>10000) fdbKD=10000;

    tx_msg.Data[0]=0xC0|ack_status;
	tx_msg.Data[1]=0x03;
	tx_msg.Data[2]=fdbKP>>8;
	tx_msg.Data[3]=fdbKP&0xff;
	tx_msg.Data[4]=fdbKD>>8;
	tx_msg.Data[5]=fdbKD&0xff;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

void ankle_motor::get_motor_parameter(uint8_t param_cmd)
{
	Can_Msg tx_msg;

	tx_msg.ID = motor_id;
	tx_msg.ExternFlag = 0;
	tx_msg.RemoteFlag = 0;
	tx_msg.DataLen = 2;

	tx_msg.Data[0] = 0xE0;
	tx_msg.Data[1] = param_cmd;
	
    CAN_Transmit(usb_num, can_channel, &tx_msg, 1, 100);
}

void ankle_motor::RV_can_data_repack(Can_Msg RxMessage,uint8_t comm_mode)
{
	uint8_t motor_id_t=0;
	uint8_t ack_status=0;
	uint16_t pos_int=0;            
	uint16_t spd_int=0;
	uint16_t cur_int=0;

	if(RxMessage.ID == 0x7ff)
	{
		if(RxMessage.Data[2]!=0x01)//determine whether it is a motor feedback instruction
			return;			//it is not a motor feedback instruction 
		if((RxMessage.Data[0]==0xff)&&(RxMessage.Data[1]==0xFF))
		{
			motor_comm_fbd.motor_id=RxMessage.Data[3]<<8|RxMessage.Data[4];
			motor_comm_fbd.motor_fbd=0x01;
		}
		else if((RxMessage.Data[0]==0x80)&&(RxMessage.Data[1]==0x80))//inquire failed
		{
			motor_comm_fbd.motor_id=0;
			motor_comm_fbd.motor_fbd=0x80;
		}
		else if((RxMessage.Data[0]==0x7F)&&(RxMessage.Data[1]==0x7F))//reset ID succeed
		{
			motor_comm_fbd.motor_id=1;
			motor_comm_fbd.motor_fbd=0x05;
		}
		else
		{
			motor_comm_fbd.motor_id=RxMessage.Data[0]<<8|RxMessage.Data[1];
			motor_comm_fbd.motor_fbd=RxMessage.Data[3];
		}
	}
	else if(comm_mode==0x00)//Response mode
	{
		ack_status=RxMessage.Data[0]>>5;
		motor_id_t=RxMessage.ID-1;motor_id_check=RxMessage.ID;
		rv_motor_msg[motor_id_t].motor_id=motor_id_t;
		rv_motor_msg[motor_id_t].error=RxMessage.Data[0]&0x1F;
		if(ack_status==1)//response frame 1
		{
			pos_int=(uint16_t)((uint8_t)(RxMessage.Data[1])<<8)|(uint8_t)(RxMessage.Data[2]);
			spd_int=(uint16_t)((uint8_t)(RxMessage.Data[3])<<4)|((uint8_t)(RxMessage.Data[4])&0xF0)>>4;
			cur_int=(uint16_t)(((uint8_t)(RxMessage.Data[4])&0x0F)<<8)|(uint8_t)(RxMessage.Data[5]);
			
			rv_motor_msg[motor_id_t].angle_actual_rad=uint_to_float(pos_int,POS_MIN,POS_MAX,16);
			rv_motor_msg[motor_id_t].speed_actual_rad=uint_to_float(spd_int,SPD_MIN,SPD_MAX,12);
			rv_motor_msg[motor_id_t].current_actual_float=uint_to_float(cur_int,I_MIN,I_MAX,12);
			rv_motor_msg[motor_id_t].temperature=(RxMessage.Data[6]-50)/2;
		}
		else if(ack_status==2)//response frame 2
		{
			rv_type_convert.buf[0]=RxMessage.Data[4];
			rv_type_convert.buf[1]=RxMessage.Data[3];
			rv_type_convert.buf[2]=RxMessage.Data[2];
			rv_type_convert.buf[3]=RxMessage.Data[1];
			rv_motor_msg[motor_id_t].angle_actual_float=rv_type_convert.to_float;
			rv_motor_msg[motor_id_t].current_actual_int=RxMessage.Data[5]<<8|RxMessage.Data[6];
			rv_motor_msg[motor_id_t].temperature=(RxMessage.Data[7]-50)/2;
			rv_motor_msg[motor_id_t].current_actual_float=rv_motor_msg[motor_id_t].current_actual_int/100.0f;
		}
		else if(ack_status==3)//response frame 3
		{
			rv_type_convert.buf[0]=RxMessage.Data[4];
			rv_type_convert.buf[1]=RxMessage.Data[3];
			rv_type_convert.buf[2]=RxMessage.Data[2];
			rv_type_convert.buf[3]=RxMessage.Data[1];
			rv_motor_msg[motor_id_t].speed_actual_float=rv_type_convert.to_float;
			rv_motor_msg[motor_id_t].current_actual_int=RxMessage.Data[5]<<8|RxMessage.Data[6];
			rv_motor_msg[motor_id_t].temperature=(RxMessage.Data[7]-50)/2;
			rv_motor_msg[motor_id_t].current_actual_float=rv_motor_msg[motor_id_t].current_actual_int/100.0f;
		}
		else if(ack_status==4)//response frame 4
		{
			if(RxMessage.DataLen!=3)	return;
			motor_comm_fbd.INS_code=RxMessage.Data[1];
			motor_comm_fbd.motor_fbd=RxMessage.Data[2];
		}
		else if(ack_status==5)//response frame 5
		{
			motor_comm_fbd.INS_code=RxMessage.Data[1];
			if(motor_comm_fbd.INS_code==1&RxMessage.DataLen==6)//get position
			{
				rv_type_convert.buf[0]=RxMessage.Data[5];
				rv_type_convert.buf[1]=RxMessage.Data[4];
				rv_type_convert.buf[2]=RxMessage.Data[3];
				rv_type_convert.buf[3]=RxMessage.Data[2];
				rv_motor_msg[motor_id_t].angle_actual_float=rv_type_convert.to_float;
			}
			else if(motor_comm_fbd.INS_code==2&RxMessage.DataLen==6)//get speed
			{
				rv_type_convert.buf[0]=RxMessage.Data[5];
				rv_type_convert.buf[1]=RxMessage.Data[4];
				rv_type_convert.buf[2]=RxMessage.Data[3];
				rv_type_convert.buf[3]=RxMessage.Data[2];
				rv_motor_msg[motor_id_t].speed_actual_float=rv_type_convert.to_float;
			}
			else if(motor_comm_fbd.INS_code==3&RxMessage.DataLen==6)//get current
			{
				rv_type_convert.buf[0]=RxMessage.Data[5];
				rv_type_convert.buf[1]=RxMessage.Data[4];
				rv_type_convert.buf[2]=RxMessage.Data[3];
				rv_type_convert.buf[3]=RxMessage.Data[2];
				rv_motor_msg[motor_id_t].current_actual_float=rv_type_convert.to_float;
			}
			else if(motor_comm_fbd.INS_code==4&RxMessage.DataLen==6)//get power
			{
				rv_type_convert.buf[0]=RxMessage.Data[5];
				rv_type_convert.buf[1]=RxMessage.Data[4];
				rv_type_convert.buf[2]=RxMessage.Data[3];
				rv_type_convert.buf[3]=RxMessage.Data[2];
				rv_motor_msg[motor_id_t].power=rv_type_convert.to_float;
			}
			else if(motor_comm_fbd.INS_code==5&RxMessage.DataLen==4)//get acceleration
			{
				rv_motor_msg[motor_id_t].acceleration=RxMessage.Data[2]<<8|RxMessage.Data[3];
			}
			else if(motor_comm_fbd.INS_code==6&RxMessage.DataLen==4)//get linkage_KP
			{
				rv_motor_msg[motor_id_t].linkage_KP=RxMessage.Data[2]<<8|RxMessage.Data[3];
			}
			else if(motor_comm_fbd.INS_code==7&RxMessage.DataLen==4)//get speed_KI
			{
				rv_motor_msg[motor_id_t].speed_KI=RxMessage.Data[2]<<8|RxMessage.Data[3];
			}
			else if(motor_comm_fbd.INS_code==8&RxMessage.DataLen==4)//get feedback_KP
			{
				rv_motor_msg[motor_id_t].feedback_KP=RxMessage.Data[2]<<8|RxMessage.Data[3];
			}
			else if(motor_comm_fbd.INS_code==9&RxMessage.DataLen==4)//get feedback_KD
			{
				rv_motor_msg[motor_id_t].feedback_KD=RxMessage.Data[2]<<8|RxMessage.Data[3];
			}
		}
	}
	else if(comm_mode==0x01)//automatic feedback mode
	{
		motor_id_t=RxMessage.ID-0x205;
		rv_motor_msg[motor_id_t].angle_actual_int=(uint16_t)(RxMessage.Data[0]<<8|RxMessage.Data[1]);
		rv_motor_msg[motor_id_t].speed_actual_int=(int16_t)(RxMessage.Data[2]<<8|RxMessage.Data[3]);
		rv_motor_msg[motor_id_t].current_actual_int=(RxMessage.Data[4]<<8|RxMessage.Data[5]);
		rv_motor_msg[motor_id_t].temperature=RxMessage.Data[6];
		rv_motor_msg[motor_id_t].error=RxMessage.Data[7];
	}
}