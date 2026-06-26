#include "balance.h"
#include "usbh_hid_joy.h"

//int Time_count=0; //Time variable //计时变量

u32 Buzzer_count1 = 0;

#define Bias_Limit 200
#define filter 1

// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 
float rotation_Kp = 0.005,  rotation_Kd = 0.001,  rotation_Ki = 0.0;//0.010 0.001 0.0
float rotation_Kp1 = 0.010  ,  rotation_Kd1 = 0.001,  rotation_Ki1 = 0.0;//0.010 0.001 0.0

short test_num;
u8 command_lost_count=0;//

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     

//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0;//标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //标记开始清除PWM
u8 clear_done_once = 0; //清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
 float angle_error;
 float angle_error1;//转角
 float angle_error2;
 float output;
 float output1;//转角
 float output2;//直行
// float Yaw = 0.0f,Pitch = 0.0f,Roll =0.0f;
/*------------------------------------*/


/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无vx>0往前，vy》0往右，vz》0，往右
**************************************************************************/
 void Drive_Motor(float Vx,float Vy,float Vz )
{
	float amplitude=3.5*2000; 							//车轮目标速度限幅

	Vx=target_limit_float(Vx,-amplitude,amplitude);
	Vy=target_limit_float(Vy,-amplitude,amplitude);
	Vz=target_limit_float(Vz,-amplitude,amplitude);

													//运动学逆解
	MOTOR_A.Target   = (+Vy+Vx-Vz*(Axle_spacing+Wheel_spacing))*2000;
	MOTOR_B.Target   = (-Vy+Vx-Vz*(Axle_spacing+Wheel_spacing))*2000;
	MOTOR_C.Target   = (+Vy+Vx+Vz*(Axle_spacing+Wheel_spacing))*2000;
	MOTOR_D.Target   = (-Vy+Vx+Vz*(Axle_spacing+Wheel_spacing))*2000;
	//Servo = SERVO_INIT;								//舵机零点
													//车轮(电机)目标速度限幅
	MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
	MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
	MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
	MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
	
	//Servo=target_limit_int(Servo,800,2200);			//舵机PWM值限幅
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
		//此任务以100Hz的频率运行（10ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_200_HZ)); 
			
		//时间计数，30秒后不再需要
		if(SysVal.Time_count<3000) SysVal.Time_count++;
		Buzzer_count1++;

		//printf("111");
		//获取编码器数据，即车轮实时速度，并转换位国际单位
		Get_Velocity_Form_Encoder();   
		MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
		MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
		MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
		MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

		
		Limit_Pwm(16700);

		Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 1500);
	}			
}  

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
static float FIFO_ENCODER[4][11];
int Incremental_PI_A (float Encoder,float Target)
{   
	 static float Bias,Pwm,Last_bias,i_out,i_out_last;
	 
	 Bias=Target-Encoder;
	
	 i_out=Velocity_KI*Bias; 
	 
	 if(Bias>Bias_Limit)  Bias=Bias_Limit;
	 if(Bias<-Bias_Limit)  Bias=-Bias_Limit;
	 
	 Pwm+=Velocity_KP*(Bias-Last_bias)+i_out*filter+i_out_last*(1-filter); 
	 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 i_out_last=i_out;
	 
	 return Pwm;
}
int Incremental_PI_B (float Encoder,float Target)
{  	 
	 static float Bias,Pwm,Last_bias,i_out,i_out_last;
	 
	 Bias=Target-Encoder;
	
	 i_out=Velocity_KI*Bias; 
	 
	 if(Bias>Bias_Limit)  Bias=Bias_Limit;
	 if(Bias<-Bias_Limit)  Bias=-Bias_Limit;
	 
	 Pwm+=Velocity_KP*(Bias-Last_bias)+i_out*filter+i_out_last*(1-filter); 
	 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 i_out_last=i_out;
	 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  	 
	 static float Bias,Pwm,Last_bias,i_out,i_out_last;
	 
	 Bias=Target-Encoder;
	
	 i_out=Velocity_KI*Bias; 
	 
	 if(Bias>Bias_Limit)  Bias=Bias_Limit;
	 if(Bias<-Bias_Limit)  Bias=-Bias_Limit;
	 
	 Pwm+=Velocity_KP*(Bias-Last_bias)+i_out*filter+i_out_last*(1-filter); 
	 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 i_out_last=i_out;
	 
	 return Pwm;
}
int Incremental_PI_D (float Encoder,float Target)
{  	 
	 static float Bias,Pwm,Last_bias,i_out,i_out_last;
	 
	 Bias=Target-Encoder;
	
	 i_out=Velocity_KI*Bias; 
	 
	 if(Bias>Bias_Limit)  Bias=Bias_Limit;
	 if(Bias<-Bias_Limit)  Bias=-Bias_Limit;
	 
	 Pwm+=Velocity_KP*(Bias-Last_bias)+i_out*filter+i_out_last*(1-filter); 
	 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 i_out_last=i_out;
	 
	 return Pwm;
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
	static u8 acc_dec_filter = 0;		
	
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	if(++acc_dec_filter==15)
	{
		acc_dec_filter=0;
	  if (PS2_KEY==11)	    RC_Velocity+=5;  //To accelerate//加速
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速	
	}

	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //对PS2手柄控制命令进行处理
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
    u8 tmp;
    //传入任务的频率
    tmp=KEY_Scan(RATE_100_HZ,0);
		if(Check==0)
		{
    //单击 或 手柄同时按下两边的下扳机，开启自动回充
    if(tmp==single_click || ( Get_PS2_KEY(L2_KEY) && Get_PS2_KEY(R2_KEY) ) )
	{
		Allow_Recharge=!Allow_Recharge;
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}		

    //双击 或 手柄同时按下两边的摇杆,更新陀螺仪
    else if(tmp==double_click || ( Get_PS2_KEY(LF_ROCKER_KEY) && Get_PS2_KEY(RT_ROCKER_KEY) )  ) 
	{
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}

    //长按 切换页面
    else if(tmp==long_click )
    {
        oled_refresh_flag=1;
        oled_page++;
        if(oled_page>OLED_MAX_Page-1) oled_page=0;
    }
	}
		else if(Check==1)
		{
			if(tmp==single_click)		
			{
				Proc_Flag++;
				if(Proc_Flag==21)			
				{
					Check = 0;
					Buzzer = 0;
					Proc_Flag = 0;
					check_time_count_motor_forward=300;
					check_time_count_motor_retreat=500;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
				}
			}
			else if(tmp==double_click)
			{
				Check = 0;
				Buzzer = 0;
				Proc_Flag = 0;
				check_time_count_motor_forward=300;
				check_time_count_motor_retreat=500;
				Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
				servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
				TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
			}
		}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		static float Encoder_A_pr=0;
		float	Encoder_B_pr=0,Encoder_C_pr=0,Encoder_D_pr=0;
		static float Encoder_A_pr1=0;
		OriginalEncoder.A=Read_Encoder(2);
		OriginalEncoder.B=Read_Encoder(3);
		OriginalEncoder.C=Read_Encoder(4);
		OriginalEncoder.D=Read_Encoder(5);

		Encoder_A_pr1= OriginalEncoder.A;
		if (Encoder_A_pr1==0)
			Encoder_A_pr=Encoder_A_pr;
		else
			Encoder_A_pr=Encoder_A_pr1;
		Encoder_B_pr= OriginalEncoder.B;
		Encoder_C_pr=-OriginalEncoder.C;
		Encoder_D_pr=-OriginalEncoder.D; 
	
    	//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision*2000;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision*2000;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision*2000; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision*2000; 

}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;
	if(PS2_ON_Flag)
	{
		step=0.05;
	}
	else
	{
		step=0.01;
	}
	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)	smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)	smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)	smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//如果连续6次接近满幅输出，判断为电机乱转，让电机失能	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

//PWM消除函数
void auto_pwm_clear(void)
{
	//小车姿态简易判断
	float y_accle = (float)(imu.accel.y/1671.84f);//Y轴加速度实际值
	float z_accle = (float)(imu.accel.z/1671.84f);//Z轴加速度实际值
	float diff;
	
	//计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM消除检测
	if( MOTOR_A.Target !=0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f )
	{
		start_check_flag = 1;//标记需要清空PWM
		wait_clear_times = 0;//复位清空计时
		start_clear = 0;     //复位清除标志
		
		
		//运动时斜坡检测的数据复位
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //当目标速度由非0变0时，开始计时 2.5 秒，若小车不在斜坡状态下，清空pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//小车在水平面上时才标记清空pwm，防止小车在斜坡上运动出现溜坡
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//开启清除pwm
				else clear_done_once = 1;//小车在斜坡上，标记已完成清除
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//完成了清除后，若出现推车行为，pwm积累一定数值后将在10秒后再次清空
	if( clear_done_once )
	{
		//小车接近于水平面时才作积累消除，防止小车在斜坡上溜车
		if( diff > 8.8f )
		{
			//完成清除后pwm再次积累，重新清除
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//开启清除pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}

/**************************************************************************
函数功能：角度pid
入口参数：三轴目标速度
返回  值：无
**************************************************************************/

float converted_angle(float angle)	
{
	float converted_angle = (angle + 180.0f);  // 偏移到 [0, 360]
    if (converted_angle < 0) 
	{
        converted_angle += 360.0f;  // 处理负值
	}
    return fmod(converted_angle, 360.0f); 
}

//自转
float rotation(float Set_turn, float Gyro_Z)
{
	int32_t sum=0;

	static float previous_error1 = 0.0,  integral1 = 0.0;

	angle_error1 = Set_turn-Gyro_Z;

	if(angle_error1>180)angle_error1=angle_error1-360;
	if(angle_error1<-180)angle_error1=angle_error1+360;

	output1 = rotation_Kp * (angle_error1) +  rotation_Kd* (angle_error1-previous_error1)+rotation_Ki*integral1;

	if(output1>4)output1=4;
	if(output1<-4)output1=-4;	

	if(abs(angle_error1)<0.7)	output1=0;

	previous_error1=angle_error1;
	return -output1;
}
float rotation1(float Set_turn, float Gyro_Z)
{
	int32_t sum=0;

	static float previous_error1 = 0.0,  integral1 = 0.0;

	angle_error1 = Set_turn-Gyro_Z;

	if(angle_error1>180)angle_error1=angle_error1-360;
	if(angle_error1<-180)angle_error1=angle_error1+360;

	output1 = rotation_Kp1 * (angle_error1) +  rotation_Kd1* (angle_error1-previous_error1)+rotation_Ki1*integral1;

	if(output1>4)output1=4;
	if(output1<-4)output1=-4;	

	if(abs(angle_error1)<0.7)	output1=0;

	previous_error1=angle_error1;
	return -output1;
}
//转相对的一个角度

//直行
float angle_con;

int go_str_PLL(float vx,float vy,int flag)
{
	static float angle_now, angle_rd_flag = 0 , finish_flag = 0;
	if(angle_rd_flag==0)
	{
		angle_now=fAngle1;
		angle_rd_flag=1;
	}
	if(flag==0)
	{
		Drive_Motor(0,0,0);
		angle_rd_flag=0;
		return 0;
	}
	else
		Drive_Motor(vx,vy,rotation(angle_now,fAngle1));
}

/**************************************************************************
函数功能：舵机控制
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
//TIM12_SERVO_Init(9999,84-1);ARR PSC 10ms 周期10ms ccr为250 -90 ，ccr750为角度为90
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM12, Compare);		//设置CCR2的值
}

void Servo_SetAngle(float Angle)
{
	PWM_SetCompare2((Angle-90)/ 180 *1000 + 1500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}

//static float q0=1,q1=0,q2=0,q3=0;
//static float exInt=0,eyInt=0,ezInt=0;
/*
void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az)
{	
	float Kp = 30.0f;
	float Ki = 30.0f;
	float halfT = 0.0051f;
	float norm;
	float vx,vy,vz;
	float ex,ey,ez;
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax/norm;
	ay = ay/norm;
	az = az/norm;

	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q3*q2);
	vz = q0*q0-q1*q1-q2*q2+q3*q3;

	ex = (ay*vz-az*vy);
	ey = (az*vx-ax*vz);
	ez = (ax*vy-ay*vx);
	
	exInt = exInt+ex*Ki;
	eyInt = eyInt+ey*Ki;
	ezInt = ezInt+ez*Ki;

//调整后的陀螺仪测量

	gx=gx+Kp*ex+exInt;

	gy=gy+Kp*ey+eyInt;

	gz=gz+Kp*ez+ezInt;

//整合四元数率和正常化

	q0=q0+(-q1*gx-q2*gy-q3*gz)*halfT;

	q1=q1+(q0*gx+q2*gz-q3*gy)*halfT;

	q2=q2+(q0*gy-q1*gz+q3*gx)*halfT;

	q3=q3+(q0*gz+q1*gy-q2*gx)*halfT;

	//正常化四元

	norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);

	q0=q0/norm;

	q1=q1/norm;

	q2=q2/norm;

	q3=q3/norm;

	Pitch=asin(-2*q1*q3+2*q0*q2)*57.3;//pitch，转换为度数

	Roll=atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3;//rollv

	Yaw=atan2(2*(q1*q2+q0*q3),1-2*(q2*q2+q3*q3))*57.3;//此处没有价值，注掉
	//converted_angle(Yaw);
}
*/
