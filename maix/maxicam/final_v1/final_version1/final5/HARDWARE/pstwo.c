#include "pstwo.h"
#include "usbh_hid_joy.h"

extern HID_Machine_TypeDef  HID_Machine;
u8 USBH_Check_HIDCommDead(USB_OTG_CORE_HANDLE *pcore,HID_Machine_TypeDef *phidm);

u8 usb_ps2_ready = 0;

#define DELAY_TIME  delay_us(5); 
#define width 960
#define heigth 672
#define theta_max 80.0
#define WINDOW_SIZE 5
#define DEADZONE 0.5f  // 0.5度死区
PID_Controller x_pid = {0.8, 0.05, 0.1, 0, 0}; // X轴参数
PID_Controller y_pid = {0.0005, 0, 0, 0, 0}; // Y轴参数
float angle_buffer[WINDOW_SIZE];
int buffer_index = 0;
//Button value reading, zero time storage
//按键值读取，零时存储
u16 Handkey;	
//Start the order. Request data
//开始命令。请求数据
u8 Comd[2]={0x01,0x42};	
//Data store array
//数据存储数组
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	}; //Key value and key name //按键值与按键名

u8 USB_PS2_DataKey(void){
	if(Get_PS2_KEY(SELECT_KEY)==KEY_ON) return PSB_SELECT;
	if(Get_PS2_KEY(LF_ROCKER_KEY)==KEY_ON) return PSB_L3;
	if(Get_PS2_KEY(RT_ROCKER_KEY)==KEY_ON) return PSB_R3;
	if(Get_PS2_KEY(START_KEY)==KEY_ON) return PSB_START;
	if(Get_PS2_KEY(LF_UP)==KEY_ON) return PSB_PAD_UP;
	if(Get_PS2_KEY(LF_RIGHT)==KEY_ON) return PSB_PAD_RIGHT;
	if(Get_PS2_KEY(LF_DOWN)==KEY_ON) return PSB_PAD_DOWN;
	if(Get_PS2_KEY(LF_LEFT)==KEY_ON) return PSB_PAD_LEFT;
	if(Get_PS2_KEY(L2_KEY)==KEY_ON) return PSB_L2;
	if(Get_PS2_KEY(R2_KEY)==KEY_ON) return PSB_R2;
	if(Get_PS2_KEY(L1_KEY)==KEY_ON) return PSB_L1;
	if(Get_PS2_KEY(R1_KEY)==KEY_ON) return PSB_R1;
	if(Get_PS2_KEY(RT_GREEN)==KEY_ON) return PSB_GREEN;
	if(Get_PS2_KEY(RT_RED)==KEY_ON) return PSB_RED;
	if(Get_PS2_KEY(RT_BLUE)==KEY_ON) return PSB_BLUE;
	if(Get_PS2_KEY(RT_PINK)==KEY_ON) return PSB_PINK;
	else return 0;
	
}
void USB_PS2_Read(void){
	static int Strat;
	//usb数据读取与解码
	USBH_Process(&USB_OTG_Core_dev, &USB_Host);
	if( usb_ps2_ready==1 )
	{
		//手柄数据读取
		PS2_KEY=USB_PS2_DataKey();
		PS2_LX = ps2_data.LX;
		PS2_LY = ps2_data.LY;
		PS2_RX = ps2_data.RX;
		PS2_RY = ps2_data.RY;
		if( PS2_KEY==4 && PS2_ON_Flag==0 )
		//The start button on the // handle is pressed
		//手柄上的Start按键被按下
			Strat=1; 
		if(Strat&&(PS2_LY<118)&&PS2_ON_Flag==0&&SysVal.Time_count>=CONTROL_DELAY)
			//When the button is pressed, you need to push the right side forward to the formal ps2 control car
			//Start按键被按下后，需要推下右边前进杆，才可以正式PS2控制小车
			PS2_ON_Flag=1,Remote_ON_Flag=0,APP_ON_Flag=0,CAN_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;  
		
	}
	else
	{
		if(PS2_ON_Flag==1)
		{
			PS2_ON_Flag=0;
			Move_X=0,Move_Y=0,Move_Z=0;//控制时热插拔,清空目标速度
		}
		PS2_RX = 128;
		PS2_RY = 128;
		PS2_LX = 128;
		PS2_LY = 128;
		PS2_KEY=0;
	}
}
/**************************************************************************
Function: Ps2 handle task
Input   : none
Output  : none
函数功能：PS2手柄任务
入口参数：无
返回  值：无
**************************************************************************/	
void pstwo_task(void *pvParameters)
{
	uint8_t received_data1;
    u32 lastWakeTime = getSysTickCnt();   
	float res=0;
//	char *task_list_buffer = pvPortMalloc(1024); // 分配足够大的缓冲区（根据任务数量调整）
    while(1)
    {
		
		//此任务以100Hz的频率运行 	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		//逻辑判断
		float out = bias2angle( result.value1);
		out=moving_average(out);
		out=apply_deadzone(out);
		float position_vx=PID_Calculate(&y_pid,672-result.value2);//位置环
		if(result.cmd == 0x01)//寻找目标
		{
			res = rotation(fAngle1-out,fAngle1);
			if(result.value2>=672/2)
			{
				res=0.5*res;
			}
				Drive_Motor(position_vx,0,res);
		}
		else if(result.cmd == 0x02)//旋转模式 
		{
			Drive_Motor(0,0,0.2);
		 
		}
		else if(result.cmd == 0x03)//倒退模式
		{
			Drive_Motor(0.2,0,0);
			vTaskDelay(400);
			Drive_Motor(-0.2,0,0);
			vTaskDelay(2000);
		}
		else if(result.cmd == 0x04)//停车模式
		{
			Drive_Motor(0,0,0);
		
		}
		else if(result.cmd == 0x05)//倒车模式
		{
			Drive_Motor(0.1,0,rotation1(-90,fAngle1));
		}
		if(result.cmd == 0x06)//寻找目标
		{
			
			res = rotation(fAngle1-out,fAngle1);
			if(result.value2>=672/2)
			{
				res=0.5*res;
			}
			Drive_Motor(0.08,0,res);		
		}
		if(result.cmd == 0x07)//寻找目标
		{
			Drive_Motor(0.1,0,0);		
		}
		}
    
}  



//位置环
// 定义PID结构体



float PID_Calculate(PID_Controller *pid, float error) {
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
	float result=pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
	if(result>0.25)result=0.25;
	if(result<-0.25)result=-0.25;
    return result  ;
}















float bias2angle(float bias)
{
	return (bias / (width/2.0) ) * (theta_max / 2);
}



float moving_average(float new_angle) 
{
	angle_buffer[buffer_index] = new_angle;
	buffer_index = (buffer_index + 1) % WINDOW_SIZE;
	float sum = 0;
	for (int i=0; i<WINDOW_SIZE; i++) sum += angle_buffer[i];
	return sum / WINDOW_SIZE;
}
float apply_deadzone(float error) 
{
	if (fabs(error) < DEADZONE) return 0;
	return error;
}

////////////////////// 注：以下标准PS2接口函数已弃用,目前使用usb接口PS2函数 ////////////////////////////////////

/**************************************************************************
Function: Ps2 handle initializer
Input   : none
Output  : none
函数功能：PS2手柄初始化
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable the ability port clock
	//使能端口时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		//下拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12;		  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;          
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;        
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;     
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;         
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
函数功能：读取PS2手柄的控制量
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_Read(void)
{
static int Strat;

	//Reading key
  //读取按键键值
	PS2_KEY=PS2_DataKey(); 
	//Read the analog of the remote sensing x axis on the left
  //读取左边遥感X轴方向的模拟量	
	PS2_LX=PS2_AnologData(PSS_LX); 
	//Read the analog of the directional direction of remote sensing on the left
  //读取左边遥感Y轴方向的模拟量	
	PS2_LY=PS2_AnologData(PSS_LY);
	//Read the analog of the remote sensing x axis
  //读取右边遥感X轴方向的模拟量  
	PS2_RX=PS2_AnologData(PSS_RX);
	//Read the analog of the directional direction of the remote sensing y axis
  //读取右边遥感Y轴方向的模拟量  
	PS2_RY=PS2_AnologData(PSS_RY);  

	if(PS2_KEY==4&&PS2_ON_Flag==0) 
		//The start button on the // handle is pressed
		//手柄上的Start按键被按下
		Strat=1; 
	
	if(Strat&&(PS2_LY<118)&&PS2_ON_Flag==0&&SysVal.Time_count>=CONTROL_DELAY)
		//When the button is pressed, you need to push the right side forward to the formal ps2 control car
		//Start按键被按下后，需要推下右边前进杆，才可以正式PS2控制小车
		PS2_ON_Flag=1,Remote_ON_Flag=0,APP_ON_Flag=0,CAN_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0; 
	
}
/**************************************************************************
Function: Send commands to the handle
Input   : none
Output  : none
函数功能：向手柄发送命令
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;     //Output a control bit //输出一位控制位
		}
		else DO_L;

		CLK_H;      //Clock lift //时钟拉高
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}
/**************************************************************************
Function: Whether it is a red light mode, 0x41= analog green light, 0x73= analog red light
Input   : none
Output  : 0: red light mode, other: other modes
函数功能：判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
入口参数：无
返回  值：0：红灯模式，其他：其他模式
**************************************************************************/	
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders //开始命令
	PS2_Cmd(Comd[1]);  //Request data //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
/**************************************************************************
Function: Read the handle data
Input   : none
Output  : none
函数功能：读取手柄数据
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders //开始命令
	PS2_Cmd(Comd[1]);  //Request data //请求数据
	for(byte=2;byte<9;byte++) //Start receiving data //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}
/**************************************************************************
Function: Handle the data of the read 2 and handle only the key parts
Input   : none
Output  : 0: only one button presses the next time; No press
函数功能：对读出来的PS2的数据进行处理,只处理按键部分 
入口参数：无
返回  值：0: 只有一个按键按下时按下; 1: 未按下
**************************************************************************/	
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3]; //This is 16 buttons, pressed down to 0, and not pressed for 1  //这是16个按键，按下为0，未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;  //No buttons //没有任何按键按下
}
/**************************************************************************
Function: Get a simulation of a rocker
Input   : Rocker
Output  : Simulation of rocker, range 0~ 256
函数功能：得到一个摇杆的模拟量
入口参数：摇杆
返回  值：摇杆的模拟量, 范围0~256
**************************************************************************/
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}
/**************************************************************************
Function: Clear data buffer
Input   : none
Output  : none
函数功能：清除数据缓冲区
入口参数：无
返回  值：无
**************************************************************************/
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function: Handle vibration function
Input   : motor1: the right small vibrator, 0x00, other
          motor2: the left big shock motor 0x40~ 0xff motor is open, and the value is greater
Output  : none
函数功能：手柄震动函数
入口参数：motor1:右侧小震动电机 0x00关，其他开
	        motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
返回  值：无
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
  PS2_Cmd(0x01); //Start order //开始命令
	PS2_Cmd(0x42); //Request data //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
/**************************************************************************
Function: Short press
Input   : none
Output  : none
函数功能：短按
入口参数：无
返回  值：无
**************************************************************************/
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Enter configuration
Input   : none
Output  : none
函数功能：进入配置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_EnterConfing(void)
{
  CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Send mode Settings
Input   : none
Output  : none
函数功能：发送模式设置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  Software Settings send mode 软件设置发送模式
	PS2_Cmd(0x03); //0x03 lock storage setup, which cannot be set by the key "mode" set mode. //0x03锁存设置，即不可通过按键“MODE”设置模式。
				         //0xee non-locking software Settings can be set by the key "mode" set mode.//0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Vibration setting
Input   : none
Output  : none
函数功能：振动设置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Complete and save the configuration
Input   : none
Output  : none
函数功能：完成并保存配置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Handle configuration initialization
Input   : none
Output  : none
函数功能：手柄配置初始化
入口参数：无
返回  值：无
**************************************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		  //Enter configuration mode //进入配置模式
	PS2_TurnOnAnalogMode();	//The "traffic light" configuration mode and select whether to save //“红绿灯”配置模式，并选择是否保存
	//PS2_VibrationMode();	//Open vibration mode //开启震动模式
	PS2_ExitConfing();		  //Complete and save the configuration //完成并保存配置
}
/**************************************************************************
Function: Read the handle information
Input   : none
Output  : none
函数功能：读取手柄信息
入口参数：无
返回  值：无
**************************************************************************/
void PS2_Receive (void)
{
		if(PS2_ON_Flag)
		{
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_RX=PS2_AnologData(PSS_RX);
		PS2_RY=PS2_AnologData(PSS_RY);
		}
		PS2_KEY=PS2_DataKey();
}


