#include "led.h"

int Led_Count=500; //LED flicker time control //LED闪烁时间控制

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void V1_0_LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  LED_PIN;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
}

void V1_1_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    // 配置GPIO_InitStructure结构体
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // PE8对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使用上下拉
    // 初始化GPIOE
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_SetBits(GPIOE, GPIO_Pin_8);
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
函数功能：蜂鸣器接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
}
/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
函数功能：LED灯闪烁任务
入口参数：无 
返回  值：无
**************************************************************************/
void led_task(void *pvParameters)
{
	    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {		
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

		//printf("samples:%f,%f,%f,%f,%f,%f,%f,%f\n",MOTOR_A.Encoder,MOTOR_B.Encoder,MOTOR_C.Encoder,MOTOR_D.Encoder,MOTOR_A.Target,MOTOR_B.Target,MOTOR_C.Target,MOTOR_D.Target);

    }
}  

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) V1_0_LED=0;
	  else		if(++temp==time)	V1_0_LED=~V1_0_LED,temp=0;
}

