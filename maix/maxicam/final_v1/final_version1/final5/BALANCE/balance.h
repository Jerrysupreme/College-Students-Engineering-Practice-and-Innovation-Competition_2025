#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"
#include "math.h"

#define BALANCE_TASK_PRIO		4     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车运动学分析参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)
extern float rotation_Kp,  rotation_Kd ,  rotation_Ki ;
extern short test_num;
extern int robot_mode_check_flag;
extern u8 command_lost_count;
void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Get_RC(void);
void Remote_Control(void);
void Drive_Motor(float Vx,float Vy,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
void PS2_control(void);
float float_abs(float insert);
void robot_mode_check(void);
float converted_angle(float angle);
float rotation(float Set_turn, float Gyro_Z);
float rotation1(float Set_turn, float Gyro_Z);

float Go_straight(float Set_turn, float Gyro_Z);
float PI_angle( float Target);
void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az);
void Servo_SetAngle(float Angle);
void PWM_SetCompare2(uint16_t Compare);
extern float angle_error;
extern float angle_error1;//自转
extern float angle_error2;//直行
extern float output;
extern float output1;//自转
extern float output2;//直行
int go_str_PLL(float vx,float vy,int flag);
//extern 	float Yaw,Pitch,Roll;
#endif  

