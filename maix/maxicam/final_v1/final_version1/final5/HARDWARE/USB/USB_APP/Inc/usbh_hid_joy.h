#ifndef __USBH_HID_JOY_H
#define __USBH_HID_JOY_H

#include "system.h"

extern HID_cb_TypeDef HID_JOY_cb;

//有线ps2手柄设备标识符
#define Normal_PS2_VID 0x0810
#define Normal_PS2_PID 0x0001

//无线ps2手柄安卓模式设备标识符
#define _2_4G_Android_PS2_VID 0x045E
#define _2_4G_Android_PS2_PID 0x028E

//无线ps2手柄PC模式设备标识符
#define _2_4G_PC_PS2_VID 0x0079
#define _2_4G_PC_PS2_PID 0x0126

extern u8 ps2_type;

enum {
  Normal_PS2 = 1,
  _2_4G_Android_PS2,
	_2_4G_PC_PS2
};


//手柄的数据类型
typedef struct _PS2_
{
	uint8_t    LX;       //4个方向摇杆值 0~255
	uint8_t    LY; 
	uint8_t    RX;  
	uint8_t    RY;
	uint16_t   KeyState; //16个按键值
}PS2_TypeDef;

extern PS2_TypeDef ps2_data;
enum {
	KEY_OFF=0,
	KEY_ON = 1
};

//PS2按键枚举
enum 
{
	//选择按键
	SELECT_KEY		   = (1 << 0), 
	
	//左右摇杆按下键值
	LF_ROCKER_KEY      = (1 << 1),
	RT_ROCKER_KEY      = (1 << 2),
	
	//开始按键
	START_KEY          = (1 << 3),
	
	//左按键区域
	LF_UP              = (1 << 4), 
	LF_RIGHT           = (1 << 5),
	LF_DOWN            = (1 << 6),
	LF_LEFT            = (1 << 7),
	
	//左右扳机按键值
	L2_KEY             = (1 << 8),	
	R2_KEY             = (1 << 9),
	L1_KEY             = (1 << 10),  
	R1_KEY             = (1 << 11),
	
	//右按键区域
	RT_GREEN           = (1 << 12),
	RT_RED             = (1 << 13), 
	RT_BLUE            = (1 << 14), 
	RT_PINK            = (1 << 15),
};
#define Get_PS2_KEY(mask) ((ps2_data.KeyState & (mask)) ? KEY_ON : KEY_OFF)

//可直接输出字符串
//#define Get_PS2_KEY(mask)      do { if (ps2_data.KeyState & (mask)) printf(#mask "\r\n"); } while (0)


#endif
