#include "imu_task.h"
#include "wit_c_sdk.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
float fAngle1;
float pwm_turn;
float TargetAngle = 0.0;

//imu数据结构体
IMU_DATA_t imu;

uint8_t  JY61P_ULOCK_CMD[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; //解锁
uint8_t  JY61P_BAUD_CMD [5] = {0xFF, 0xAA, 0x04, 0x06, 0x00}; //波特率修改为115200
uint8_t  JY61P_SAVE_CMD [5] = {0xFF, 0xAA, 0x00, 0x00, 0x00}; //保存
uint8_t  JY61P_XY0_CMD  [5] = {0xFF, 0xAA, 0x01, 0x08, 0x00}; //XY角度归零
uint8_t  JY61P_Z0_CMD   [5] = {0xFF, 0xAA, 0x01, 0x04, 0x00}; //Z轴归零
/***********************************************************************************
MPU6050_task：处理惯导传来的数据转换为当前角度fAngle1
**********************************************************************************/
void MPU6050_task(void *pvParameters)
{
	uint16_t received_data;
    u32 lastWakeTime = getSysTickCnt();
//	char *task_list_buffer = pvPortMalloc(1024); // 分配足够大的缓冲区（根据任务数量调整）
    while(1)
    {
		//此任务以100Hz的频率运行
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_500_HZ));
		

		
			for(int i=0;i<received_len;i++)
			{
				WitSerialDataIn(rx_buffer[i]);
			}
			if(s_cDataUpdate)
			{
				fAngle1 = sReg[Roll+2] / 32768.0f * 180.0f;

				if(s_cDataUpdate & ANGLE_UPDATE)
				{
					s_cDataUpdate &= ~ANGLE_UPDATE;
				}
			}
		
	}
}
/**************************************************************************
函数功能：复制imu的数值
入口参数：需要赋值的结构体变量，被复制的结构体变量
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val)
{
	req_val->x = copied_val -> x;
	req_val->y = copied_val -> y;
	req_val->z = copied_val -> z;
}


float Scale_Transform(short Sample_Value, float URV, float LRV)
{
    float Data;             
    float Value_L = -32767.0; 
    float Value_U = 32767.0;  
    

    Data = (Sample_Value - Value_L) / (Value_U - Value_L) * (URV - LRV) + LRV;
           
    return Data;
}
 

/**************************************************************************
定义函数：惯导模块
**************************************************************************/
void JY61P_START(void)
{
    Uart2Send(JY61P_ULOCK_CMD, sizeof(JY61P_ULOCK_CMD));  //解锁
    delay_ms(200);//延时200ms
    Uart2Send(JY61P_XY0_CMD, sizeof(JY61P_XY0_CMD));    //XY轴归零
    delay_ms(200);
    Uart2Send(JY61P_Z0_CMD, sizeof(JY61P_Z0_CMD));      //Z轴归零
    delay_ms(200);
    Uart2Send(JY61P_SAVE_CMD, sizeof(JY61P_SAVE_CMD));    //保存
    delay_ms(200);
}



void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}

}

void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart2Send(p_data, uiSize);
}

void Delayms(uint16_t ucMs)
{
	delay_ms(ucMs);
}

void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}



//串口5发送函数
void Uart2Send(unsigned char *p_data, unsigned int uiSize)
{	
	unsigned int i;
	for(i = 0; i < uiSize; i++)
	{
		while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
				USART_SendData(UART5, *p_data++);		
	}
			while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
}


