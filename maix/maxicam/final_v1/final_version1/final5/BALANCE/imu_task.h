#ifndef __IMU_TASK_H
#define __IMU_TASK_H
#include "system.h"
#include "balance.h"
#define IMU_TASK_PRIO      5
#define IMU_STK_SIZE       256
#define IMU_TASK_RATE      RATE_100_HZ

typedef struct{
	short x;
	short y;
	short z;
}IMU_BASE_t;


//imu三轴数据,包含偏差值和原始值
typedef struct{
	IMU_BASE_t Deviation_accel;//偏差值,用于纠正误差.该值采集后不会发生改变
	IMU_BASE_t Deviation_gyro;
	IMU_BASE_t Original_accel;
	IMU_BASE_t Original_gyro;
	IMU_BASE_t gyro;
	IMU_BASE_t accel;
}IMU_DATA_t;

extern IMU_DATA_t imu;

//对外接口
void MPU6050_task(void *pvParameters);
//惯导配置函数
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
void Delayms(uint16_t ucMs);
void Uart2Send(unsigned char *p_data, unsigned int uiSize);
void JY61P_START(void);

void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val);
float Scale_Transform(short Sample_Value, float URV, float LRV);
extern float pwm_turn;
extern float TargetAngle;
#endif
