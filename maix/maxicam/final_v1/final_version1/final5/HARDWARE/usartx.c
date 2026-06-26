#include "usartx.h"
#include <string.h>
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;
float angle = 0.0;
#define USART_MAX_LEN 100
uint32_t received_len=0;
//ROS传输数据的标志位
int ROS_rx_start=0;
int ROS_rx_end=0;
int buffer_finish=1;
u16 RX_DATA[100]={0};
u16 RX_DATA_Buffer[100]={0};
u16 SEND_FLAG_BUFFER[10]={0,1,2,3,4,5,6,7,8,9};
int uart5_error_count=0;
uint8_t ucTemp=0;
 __attribute__((section(".dma_buffer")))  uint8_t rx_buffer[11]={0};
uint8_t dma2_usart6_rx_buffer[20]={1};

uint16_t usart6_rx_len=0;
volatile uint8_t packet_ready = 0;
uint16_t usart1_rx_len = 0;    //接收帧数据的长度
// 定义解析结果结构体


PacketData result={0};


// 错误码定义
#define PARSE_OK        0
#define HEADER_ERROR    1
#define TAIL_ERROR      2
#define LENGTH_ERROR    3

// 数据包格式定义
#define PACKET_HEAD     0x31
#define PACKET_TAIL     0x55
#define PACKET_SIZE     7

// 解包函数声明
PacketData parse_packet(const uint8_t* buffer,uint16_t len);
//
/**************************************************************************
Function: Usartx3, Usartx1,Usartx5 and CAN send data task 
Input   : none
Output  : none
函数功能：串口3、串口1、串口5、CAN发送数据任务
入口参数：无
返回  值：无
**************************************************************************/
void data_task(void *pvParameters)
{                  
	u32 lastWakeTime = getSysTickCnt();
	uint16_t  received_data1=0;
	while(1)
    {	
			//此任务以50Hz的频率运行
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));
		
		//if (xQueueReceive(xUSART1_getinISR_Queue, &received_data1, portMAX_DELAY) == pdPASS)
		//{	
		result = parse_packet(dma2_usart6_rx_buffer,usart1_rx_len);
			
//		MOTOR_A.Encoder=result.cmd;
//		MOTOR_B.Encoder=result.error;  
//		MOTOR_C.Encoder=result.len;
//		MOTOR_D.Encoder=result.value1 ;
		
		uart5_error_count=result.value1;


		//uart5_error_count=usart1_rx_len;
		//}
	}
}
/**************************************************************************
Function: 解包函数
Input   : none
Output  : none
函数功能：对接收的数据包进行处理
入口参数：无
返回  值：无
**************************************************************************/

PacketData parse_packet(const uint8_t* buffer,uint16_t len) {
   
	PacketData result = { 0 };
	
	result.len=len;
	
    // 检查数据包长度
    if(buffer == NULL) {
        result.error = LENGTH_ERROR;
        return result;
    }

    // 验证帧头
    if(buffer[0] != PACKET_HEAD) {
        result.error = HEADER_ERROR;
        return result;
    }

    // 验证帧尾
    if(buffer[PACKET_SIZE-1] != PACKET_TAIL) {
        result.error = TAIL_ERROR;
        return result;
    }

    // 提取命令
    result.cmd = buffer[1];

    // 合并数据（大端序）
    result.value1 = ( (buffer[3] << 8) | buffer[2] ) -480;  // data0 + data1
	  
	
    result.value2 = (buffer[5] << 8) | buffer[4];  // data2 + data3

    result.error = PARSE_OK;
    return result;
}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{

	///////////////////////自动回充相关变量赋值/////////////////////
	
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{		
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART3_SEND(void)
{
	  usart3_send(SEND_FRAME_HEADER);
		usart3_send(SEND_FLAG_BUFFER[0]);
	  usart3_send(SEND_FLAG_BUFFER[1]);
	  usart3_send(SEND_FLAG_BUFFER[2]);
    usart3_send(SEND_FRAME_TAIL);
	
}
void USART3_Return(void)
{
	for(int i=0; i<message_count; i++)
	{
		usart3_send(uart3_receive_message[i]);
	}
	usart3_send('\r');
	usart3_send('\n');
}
void USART2_Return(void)
{
	printf("{#");
	for(int i=0; i<app_count; i++)
	{
		printf("%c",uart2_receive_message[i]);
	}
	printf("}$");
	printf("\r\n");
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
函数功能：串口5发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART5_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart5_send(Send_Data.buffer[i]);
	}	 
	if(Get_Charging_HardWare==1)
	{
		//存在回充装备时，向上层发送自动回充相关变量
		for(i=0; i<8; i++)
		{
			usart5_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}
/**************************************************************************
Function: CAN sends data
Input   : none
Output  : none
函数功能：CAN发送数据
入口参数：无
返 回 值：无
**************************************************************************/
void CAN_SEND(void) 
{
	u8 CAN_SENT[8],i;
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
	
	//////////////////自动回充相关数据发送//////////////////
	if(Get_Charging_HardWare) CAN_Send_AutoRecharge();
}

/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
//void uart2_init(u32 bound)
//{  	 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //使能USART时钟
//	
//	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
//	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
//	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
//	
//	//UsartNVIC configuration //UsartNVIC配置
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	//Preempt priority //抢占优先级
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10;
//	//Subpriority //子优先级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//  //Enable the IRQ channel //IRQ通道使能	
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  //Initialize the VIC register with the specified parameters 
//	//根据指定的参数初始化VIC寄存器		
//	NVIC_Init(&NVIC_InitStructure);	
//	
//	//USART Initialization Settings 初始化设置
//	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
//	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
//	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //初始化串口2
//	
////	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
//	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //使能串口2 
//}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure ;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
	
  //UsartNVIC configuration //UsartNVIC配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8 ;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //初始化串口3
	
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //使能串口3 
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
函数功能：串口5初始化
入口参数：无
返回  值：无
串口5的rx为PD2，tx为PC12，使用dma为DMA1 Stream0 Channel4(对应rx)
**************************************************************************/
 void uart5_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	  //使能GPIO 时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	  //使能GPIO 时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);     //使能USART时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 , ENABLE);     //使能DMA1 时钟

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		      //初始化

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		      //初始化
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);

	//DMA配置
	DMA_InitTypeDef DMA_InitStructure;

	DMA_InitStructure.DMA_Channel=DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART5->DR; 		// 外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)rx_buffer;       		// 内存缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 外设→内存
    DMA_InitStructure.DMA_BufferSize = 100;                					// 缓冲区大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  		// 外设地址固定
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           		// 内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                   		// 循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不开启FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //FIFO阈值
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	
    DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream0, ENABLE); 
	
	//USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; 									//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 					//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 							//一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; 							//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
	USART_Init(UART5, &USART_InitStructure);      									//初始化串口5
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE); 									//开启串口接受中断
	
	// 启用 USART1 的 DMA 接收请求
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(UART5, ENABLE);
}

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：dma1 stream5 channel4
返 回 值：无
**************************************************************************/

void uart1_init(u32 bound)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //使能USART时钟
	
 	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
	
	//UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //初始化串口2
	
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  	// 开启串口DMA接收


    /* 配置串口DMA接收*/
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  					// 开启DMA时钟
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 							//通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;		//DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dma2_usart6_rx_buffer;	//DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   				//存储器到外设模式
    DMA_InitStructure.DMA_BufferSize = USART_MAX_LEN;						//数据传输量
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;				    //高等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不开启FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //FIFO阈值
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream5, ENABLE); //使能DMA2_Stream5通道
									//不使能
//	
    USART_Cmd(USART2, ENABLE);  //使能串口1

}


void USART2_IRQHandler(void)  //串口1中断服务程序
{
//	V1_0_LED=~V1_0_LED; 
	uint8_t status_value=taskENTER_CRITICAL_FROM_ISR();
	
	//BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    //TickType_t xTimeNow,xTimeNow1;
	//xTimeNow = xTaskGetTickCountFromISR();
	
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET) 	//空闲中断触发
    {
		 // 清除IDLE标志
        volatile uint32_t tmp = USART2->SR;
        tmp = USART2->DR;
        (void)tmp; 
		
    	DMA_Cmd(DMA1_Stream5, DISABLE);  					   /* 暂时关闭dma，数据尚未处理 */
    	usart1_rx_len = USART_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);/* 获取接收到的数据长度 单位为字节*/
		
		//xQueueSendFromISR(xUSART1_getinISR_Queue, &usart1_rx_len, NULL);

		//DMA_USART1_Send(dma2_usart6_rx_buffer, usart1_rx_len); // 将接收的数据回显
		
    	DMA_SetCurrDataCounter(DMA1_Stream5,USART_MAX_LEN);	/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
    	DMA_Cmd(DMA1_Stream5, ENABLE);      				/*打开DMA*/
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);  		/* 清DMA标志位 */
		USART_ClearITPendingBit(USART2,USART_IT_IDLE);			//清除空闲中断标志位（接收函数有清标志位的作用）
		V1_0_LED=~V1_0_LED; 
//				xTimeNow1 = xTaskGetTickCountFromISR();
//		uart5_error_count+=xTimeNow1-xTimeNow;
//		uart5_error_count+=1;
    }
	taskEXIT_CRITICAL_FROM_ISR(status_value);
		

	//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//void DMA2_Stream7_IRQHandler(void)
//{

//	//清除标志
//	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
//	{
//		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); //清除DMA2_Steam7传输完成标志
//   		DMA_Cmd(DMA2_Stream7,DISABLE);				//关闭使能
//    	USART_ITConfig(USART1,USART_IT_TC,ENABLE);  //打开串口发送完成中断
//	}
////}

//void DMA_USART1_Send(u8 *data,u16 size)
//{
//	//memcpy(dma2_usart6_tx_buffer,data,size);				//复制数据到DMA发送缓存区
//	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	//确保DMA可以被设置
//	DMA_SetCurrDataCounter(DMA2_Stream7,size);			//设置数据传输长度
//	DMA_Cmd(DMA2_Stream7,ENABLE);						//打开DMA数据流，开始发送
//}


/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
//int USART2_IRQHandler(void)
//{
//	int Usart_Receive;
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
//	{
//		static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
//		static float Data;

//		Usart_Receive=USART2->DR; //Read the data //读取数据
//		
//		usart2_send(Usart_Receive);

//		USART_ClearFlag(USART2,USART_IT_RXNE);
//	}
//  return 0;	
//}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/

int USART3_IRQHandler(void)
{	
	static u8 Count=0;
	u16 Usart_Receive;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
	{
		Usart_Receive = USART_ReceiveData(USART3);//Read the data //读取数据
		
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //开机10秒前不处理数据
		  return 0;	

		//Fill the array with serial data
		//串口数据填入数组
    
		
		// Ensure that the first data in the array is FRAME_HEADER
		//确保数组第一个数据为FRAME_HEADER
		if(Usart_Receive == FRAME_HEADER&&ROS_rx_start==0&&buffer_finish==1) 
		{
			RX_DATA[Count]=Usart_Receive;
			Count++;
			ROS_rx_start=1;
		} 
		else if(ROS_rx_start==0)
		{
			if(buffer_finish==0)
			{}
			else
			{
			Count=0;
			memset(RX_DATA, 0,sizeof(RX_DATA));
			}
		}
		if(ROS_rx_start==1&&Usart_Receive!=FRAME_TAIL)
		{
			RX_DATA[Count]=Usart_Receive;
			Count++;
		}
		else if(ROS_rx_start==1&&Usart_Receive==FRAME_TAIL)
		{
		RX_DATA[Count]=Usart_Receive;
		ROS_rx_end=1;
		ROS_rx_start=0;
		Count=0;
		buffer_finish=0;
		}
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
			} 
		if(USART_GetFlagStatus(USART3,USART_FLAG_ORE) == SET) // 检查 ORE 标志
		{	
			USART_ClearFlag(USART3,USART_FLAG_ORE);
			USART_ReceiveData(USART3);
		}
  return 0;	
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
函数功能：串口5接收中断
入口参数：无
返回  值：无
**************************************************************************/
void UART5_IRQHandler(void)
{
//	V1_0_LED=~V1_0_LED; 
	uint8_t status_value=taskENTER_CRITICAL_FROM_ISR();

	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	
	if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
	{
//		TickType_t xTimeNow,xTimeNow1;
//		xTimeNow = xTaskGetTickCountFromISR();
		volatile uint32_t tmp = UART5->SR;
        tmp = UART5->DR;
        (void)tmp;
    	
		DMA_Cmd(DMA1_Stream0, DISABLE);  					/* 暂时关闭dma，数据尚未处理 */
		
		received_len = 100 - DMA_GetCurrDataCounter(DMA1_Stream0);
				
    	DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);  		/* 清DMA标志位 */
		
    	DMA_SetCurrDataCounter(DMA1_Stream0,100);			/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		
    	DMA_Cmd(DMA1_Stream0, ENABLE);      				/*打开DMA*/
		
    	//USART_ClearITPendingBit(UART5,USART_IT_IDLE);							//清除空闲中断标志位（接收函数有清标志位的作用）
//		uart5_error_count+=1;
//		xTimeNow1 = xTaskGetTickCountFromISR();
//		uart5_error_count+=xTimeNow1-xTimeNow;
	}	
		taskEXIT_CRITICAL_FROM_ISR(status_value);
//		V1_0_LED=~V1_0_LED; 
		
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来目标前进速度Vx、目标角速度Vz，转换为阿克曼小车的右前轮转角
入口参数：目标前进速度Vx、目标角速度Vz，单位：m/s，rad/s
返回  值：阿克曼小车的右前轮转角，单位：rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//阿克曼小车需要设置最小转弯半径
	//如果目标速度要求的转弯半径小于最小转弯半径，
	//会导致小车运动摩擦力大大提高，严重影响控制效果
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//如果目标速度要求的转弯半径小于最小转弯半径
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s						
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
函数功能：串口5发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}
/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//对接收到的数据进行校验
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

//自动回充发送字节专用校验函数
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_AutoCharge_Data.buffer[k];
	}
	
	return check_sum;	
}


//蓝牙AT指令抓包，防止指令干扰到机器人正常的蓝牙通信
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	蓝牙链接时发送的字符，00:11:22:33:44:55为蓝牙的MAC地址
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	共44个字符
	
	蓝牙断开时发送的字符
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	共34个字符
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //蓝牙接受时指针记录器
	static u8 bt_line = 0; //表示现在在第几行
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//断开连接
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//开始连接
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//特殊标识符，开始警惕(使用时要-1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //收到‘+’，表示切换了行数	
		disconnect++,connect++;
		return 1;//抓包，禁止控制
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//开始追踪数据是否符合断开的特征，符合时全部屏蔽，不符合时取消屏蔽
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//抓包，禁止控制
		}			

		//追踪连接特征 (bt_line==1&&connect>=13)区段是蓝牙MAC地址，每一个蓝牙MAC地址都不相同，所以直接屏蔽过去
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//抓包，禁止控制
		}	

		//在抓包期间收到其他命令，停止抓包
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//非禁止数据，可以控制
		}			
	}
	
	return 0;//非禁止数据，可以控制
}

//软复位进BootLoader区域
void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符“reset”，执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//进行软件复位，复位后执行 BootLoader 程序
		}
	}
}

 float concrete_angle;
int turn_angle(int angle,int direct)
{
static float angle_now;
static int angle_rd_flag=0;
static int finish_flag=0;
if(angle_rd_flag==0)
{

  angle_now=fAngle1;
}
if(direct==1&&angle_rd_flag==0)//右转
{
concrete_angle=angle_now+angle;
	if(concrete_angle>180)
		concrete_angle-=360;
		angle_rd_flag=1;
}	
else if(direct==0&&angle_rd_flag==0)
{
concrete_angle=angle_now-angle;
	if(concrete_angle<=-180)
		concrete_angle+=360;
		angle_rd_flag=1;
}
if(abs(concrete_angle-fAngle1)<=0.8)
{
	finish_flag++;
	if(finish_flag==20)
	{
		angle_rd_flag=0;
		finish_flag=0;
		Drive_Motor(0,0,0);
    return 0;
	}
}
else if(finish_flag<20)
Drive_Motor(0,0,rotation(concrete_angle,fAngle1));
return 0;
}


