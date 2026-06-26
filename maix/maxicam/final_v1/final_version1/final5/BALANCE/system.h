#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//引用所有需要用到的头文件
#include "FreeRTOSConfig.h"
//FreeRTOS相关头文件 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "show.h"								   
#include "pstwo.h"
#include "key.h"
#include "robot_select_init.h"
#include "I2C.h"
#include "MPU6050.h"
#include "AutoRecharge.h"
#include "usb_core.h"
#include "usbh_core.h"
#include "usbh_hid_core.h"
#include "usbh_usr.h"
#include "ICM20948.h"
#include "imu_task.h"
#include "wit_c_sdk.h"
// Enumeration of car types
//小车型号的枚举定义
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car,
	Test
} CarMode;

// 定义解析结果结构体
typedef struct {
    uint8_t cmd;        // 命令标志位
    int16_t value1;    // 合并后的第一个16位数据
    int16_t value2;    // 合并后的第二个16位数据
    uint8_t error;      // 错误标志 (0: 正常)
	uint16_t len; 
} PacketData;

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern u8 Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart1_ON_Flag, Usart5_ON_Flag;
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send;                                            										                 
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int Check, Checking, Checked, CheckCount, CheckPhrase1, CheckPhrase2;
extern long int ErrorCode; 
extern float fAcc[3], fGyro[3], fAngle[3];
extern float fAngle1;
extern u8 Get_Charging_HardWare;
extern u8  charger_check;
void Find_Charging_HardWare(void);

extern USBH_HOST  USB_Host;
extern USB_OTG_CORE_HANDLE  USB_OTG_Core_dev;

extern u8 Proc_Flag;
extern int servo_direction[6],servo_pwm_count;
extern int check_time_count_motor_forward,check_time_count_motor_retreat;
extern int POT_val;
extern int Servo_Count[6];
extern u8 uart3_receive_message[50];
extern u8 uart3_send_flag;
extern u8 message_count;
extern u8 uart2_receive_message[50];
extern u8 uart2_send_flag;
extern u8 app_count;
extern int Full_rotation;

void systemInit(void);

extern int use;

//Task handle	//任务句柄
extern TaskHandle_t 	StartTask_Handler;          //创建任务任务句柄
extern TaskHandle_t 	Balance_task_Handler;       //创建Balance_task任务  任务句柄
extern TaskHandle_t 	MPU6050_task_Handler;       //创建MPU6050_task任务  任务句柄
extern TaskHandle_t 	led_task_Handler;           //创建led_task    任务  任务句柄
extern TaskHandle_t 	pstwo_task_Handler;         //创建pstwo_task  任务  任务句柄
extern TaskHandle_t 	data_task_Handler;          //创建data_task   任务  任务句柄
extern TaskHandle_t 	show_task_Handler;          //创建show_task   任务  任务句柄
extern  QueueHandle_t xUART5_getinISR_Queue;             // 串口5数据队列(从串口中断函数中得到串口信息并发送到此队列)
extern  QueueHandle_t xUSART1_getinISR_Queue;       //MPU6050task数据队列(从MPU6050task中将得到fAngle1发送到此队列)
extern int uart5_error_count;
extern uint8_t rx_buffer[11];
extern uint8_t dma2_usart6_rx_buffer[20];
extern uint8_t dma2_usart6_tx_buffer[20];

extern uint32_t received_len;

extern  uint8_t ucTemp;

extern PacketData result;


/***Macros define***/ /***宏定义***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值，目前有6种小车类型
#define CAR_NUMBER    6      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
