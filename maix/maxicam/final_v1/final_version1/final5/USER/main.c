/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V5.0
修改时间：2022-05-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V5.0
Update：2022-05-05

All rights reserved
***********************************************/
#include "system.h"

//Task priority    //任务优先级
#define START_TASK_PRIO	1

//Task stack size //任务堆栈大小	
#define START_STK_SIZE 	256  

//队列配置信息
#define UART5_QUEUE_LENGTH  1  // 队列最多存放20条数据
#define UART5_ITEM_SIZE     sizeof(uint16_t) // 每个数据是1字节
	
#define USART1_QUEUE_LENGTH  1  // 队列最多存放20条数据
#define USART1_ITEM_SIZE     sizeof(uint16_t) // 每个数据是1字节

//Task handle	//任务句柄
TaskHandle_t 	StartTask_Handler;          //创建任务任务句柄
TaskHandle_t 	Balance_task_Handler;       //创建Balance_task任务  任务句柄
TaskHandle_t 	MPU6050_task_Handler;       //创建MPU6050_task任务  任务句柄
TaskHandle_t 	led_task_Handler;           //创建led_task    任务  任务句柄
TaskHandle_t 	pstwo_task_Handler;         //创建pstwo_task  任务  任务句柄
TaskHandle_t 	data_task_Handler;          //创建data_task   任务  任务句柄
TaskHandle_t 	show_task_Handler;          //创建show_task   任务  任务句柄

//创建消息队列

QueueHandle_t xUART5_getinISR_Queue;             // 串口5数据队列(从串口5中断函数中得到串口信息并发送到此队列)

QueueHandle_t xUSART1_getinISR_Queue;             // 串口1数据队列(从串口1中断函数中得到串口信息并发送到此队列)
int use=0;

//Task function   //任务函数
void start_task(void *pvParameters);

//Main function //主函数
int main(void)
{ 
	xUART5_getinISR_Queue = xQueueCreate(UART5_QUEUE_LENGTH, UART5_ITEM_SIZE); //创建队列，长度为1，单个数据长度为sizeof(unsigned char)
 
	xUSART1_getinISR_Queue = xQueueCreate(USART1_QUEUE_LENGTH, USART1_ITEM_SIZE); //创建队列，长度为1，单个数据长度为sizeof(unsigned char)
 
	systemInit(); //Hardware initialization //硬件初始化

	// xUART5_getinISR_Queue = xQueueCreate(UART5_QUEUE_LENGTH, UART5_ITEM_SIZE); //创建队列，长度为2，单个数据长度为sizeof(unsigned char)

	
	//Create the start task //创建开始任务
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //任务函数
							(const char*    )"start_task",          //Task name       //任务名称
							(uint16_t       )START_STK_SIZE,        //Task stack size //任务堆栈大小
							(void*          )NULL,                  //Arguments passed to the task function //传递给任务函数的参数
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //任务优先级
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //任务句柄    					
	vTaskStartScheduler();  //Enables task scheduling //开启任务调度	
}
 
//Start task task function //开始任务任务函数 
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //进入临界区
	
    //Create the task //创建任务
	xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  &Balance_task_Handler);	//Vehicle motion control task //小车运动控制任务
	xTaskCreate(MPU6050_task,  "IMU_task",  	IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  &MPU6050_task_Handler);
    xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     &show_task_Handler); //The OLED display displays tasks //OLED显示屏显示任务
	xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      &led_task_Handler);	//LED light flashing task //LED灯闪烁任务
	xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      &pstwo_task_Handler);	//Read the PS2 controller task //读取PS2手柄任务
	xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     &data_task_Handler);	//Usartx3, Usartx1 and CAN send data task //串口3、串口1、CAN发送数据任务
	
    vTaskDelete(StartTask_Handler); //Delete the start task //删除开始任务

    taskEXIT_CRITICAL();            //Exit the critical section//退出临界区
}







