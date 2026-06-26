/**
  ******************************************************************************
  * @file    usbh_core.c 
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file implements the functions for the core state machine process
  *          the enumeration and the control transfer process
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      <http://www.st.com/SLA0044>
  *
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/

#include "usbh_ioreq.h"
#include "usb_bsp.h"
#include "usbh_hcs.h"
#include "usbh_stdreq.h"
#include "usbh_core.h"
#include "usb_hcd_int.h"


/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_LIB_CORE
* @{
*/

/** @defgroup USBH_CORE 
  * @brief This file handles the basic enumeration when a device is connected 
  *          to the host.
  * @{
  */ 

/** @defgroup USBH_CORE_Private_TypesDefinitions
  * @{
  */ 
uint8_t USBH_Disconnected (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_Connected (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_SOF (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_PortEnabled (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_PortDisabled (USB_OTG_CORE_HANDLE *pdev); 

USBH_HCD_INT_cb_TypeDef USBH_HCD_INT_cb = 
{
  USBH_SOF,
  USBH_Connected, 
  USBH_Disconnected,
  USBH_PortEnabled,
  USBH_PortDisabled
};

USBH_HCD_INT_cb_TypeDef  *USBH_HCD_INT_fops = &USBH_HCD_INT_cb;

/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Variables
  * @{
  */ 
__IO uint32_t suspend_flag = 0;
/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_FunctionPrototypes
  * @{
  */
static USBH_Status USBH_HandleEnum(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);
USBH_Status USBH_HandleControl (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);

void USB_OTG_BSP_Resume(USB_OTG_CORE_HANDLE *pdev);                                                                 
void USB_OTG_BSP_Suspend(USB_OTG_CORE_HANDLE *pdev);

/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Functions
  * @{
  */ 


/**
  * @brief  USBH_Connected
  *         USB Connect callback function from the Interrupt. 
  * @param  selected device
  * @retval Status
*/
/**
  * @brief  USBH_Connected
  *         USB连接回调函数来自中断。
  * @param  selected device
  * @retval Status
*/

uint8_t USBH_Connected (USB_OTG_CORE_HANDLE *pdev)
{
  pdev->host.ConnSts = 1;
  return 0;
}

/**
  * @brief  USBH_PortEnabled
  *         USB Port Enable function. 
  * @param  selected device
  * @retval Status
*/
/**
  * @brief  USBH_PortEnabled
  *         USB端口启用函数。
  * @param  selected device
  * @retval Status
*/

uint8_t USBH_PortEnabled (USB_OTG_CORE_HANDLE *pdev)
{
  pdev->host.PortEnabled = 1;
  return 0;
}

/**
  * @brief  USBH_PortDisabled
  *         USB Port Disable function.
  * @param  selected device
  * @retval Status
*/
/**
  * @brief  USBH_PortDisabled
  *         USB端口禁用函数。
  * @param  selected device
  * @retval Status
*/

uint8_t USBH_PortDisabled (USB_OTG_CORE_HANDLE *pdev)
{
  pdev->host.PortEnabled = 0;
  return 0;
}

/**
* @brief  USBH_Disconnected
*         USB Disconnect callback function from the Interrupt. 
* @param  selected device
* @retval Status
*/
/**
* @brief  USBH_Disconnected
*         USB断开连接回调函数来自中断。
* @param  selected device
* @retval Status
*/

uint8_t USBH_Disconnected (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_BSP_DriveVBUS(pdev,0);
  /* Disable all interrupts. */
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTMSK, 0);
  
  /* Clear any pending interrupts. */
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, 0xFFFFFFFF);
  USB_OTG_DisableGlobalInt(pdev);
  pdev->host.ConnSts = 0;
  return 0;  
}

/**
  * @brief  USBH_SOF
  *         USB SOF callback function from the Interrupt. 
  * @param  selected device
  * @retval Status
  */
/**
  * @brief  USBH_SOF
  *         USB SOF回调函数来自中断。
  * @param  selected device
  * @retval Status
  */

uint8_t USBH_SOF (USB_OTG_CORE_HANDLE *pdev)
{
  /* This callback could be used to implement a scheduler process */
/* 此回调可用于实现调度器进程 */
  return 0;  
}
/**
  * @brief  USBH_Init
  *         Host hardware and stack initializations 
  * @param  class_cb: Class callback structure address
  * @param  usr_cb: User callback structure address
  * @retval None
  */
/**
* @brief  USB_OTG_EnableCommonInt
*         初始化通用中断，用于设备和主机模式
* @param  pdev : 选定的设备
* @retval None
*/
void USBH_Init(USB_OTG_CORE_HANDLE *pdev,
               USB_OTG_CORE_ID_TypeDef coreID,
               USBH_HOST *phost,               
               USBH_Class_cb_TypeDef *class_cb, 
               USBH_Usr_cb_TypeDef *usr_cb)
{
     
	/* Hardware Init */
	//启动Host流程步骤1：硬件初始化,PA11、PA12
	USB_OTG_BSP_Init(pdev);  //启动Host流程步骤1：硬件初始化,PA11、PA12
  
	/* configure GPIO pin used for switching VBUS power */
	//配置用于Vbus的GPIO
	USB_OTG_BSP_ConfigVBUS(0);  //启动Host流程步骤2：其他功能性io初始化
  
	/* Host de-initializations */
	//主机去初始化				//启动Host流程步骤3：复位host
	USBH_DeInit(pdev, phost);
  
  /*Register class and user callbacks */
//指定函数调用的结构体内容
  phost->class_cb = class_cb; //启动Host流程步骤4：指定类的回调函数实例
	
//指定函数调用的结构体内容
  phost->usr_cb = usr_cb;   //启动Host流程步骤5：指定用户的回调函数实例
    
  /* Start the USB OTG core */    
 //启动主机内核
   HCD_Init(pdev , coreID); //启动Host流程步骤6：启动host内核。pdev是usb外设核心数据结构的指针,coreID是用于指定要初始化的USB外设核心
   
  /* Upon Init call usr call back */
  //调用用户的初始化函数   
  phost->usr_cb->Init(); //启动Host流程步骤7：调用用户指定的初始化函数,此处调用的是void USBH_USR_Init(void)
  
  /* Enable Interrupts */
  //使能中断
  USB_OTG_BSP_EnableInterrupt();//启动Host流程步骤8：开启host的中断
}

/**
  * @brief  USBH_DeInit 
  *         Re-Initialize Host
  * @param  None 
  * @retval status: USBH_Status
  */
/**
  * @brief  USBH_DeInit 
  *         重新初始化主机
  * @param  None 
  * @retval status: USBH_Status
  */
USBH_Status USBH_DeInit(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  /* Software Init */
  
  phost->gState = HOST_IDLE;
  phost->gStateBkp = HOST_IDLE; 
  phost->EnumState = ENUM_IDLE;
  phost->RequestState = CMD_SEND;  
  
  phost->Control.state = CTRL_SETUP;
  phost->Control.ep0size = USB_OTG_MAX_EP0_SIZE;  
  
  phost->device_prop.address = USBH_DEVICE_ADDRESS_DEFAULT;
  phost->device_prop.speed = HPRT0_PRTSPD_FULL_SPEED;
  
  USBH_Free_Channel  (pdev, phost->Control.hc_num_in);
  USBH_Free_Channel  (pdev, phost->Control.hc_num_out);  
  return USBH_OK;
}

/**
* @brief  USBH_Process
*         USB Host core main state machine process
* @param  None 
* @retval None
*/

//HID重新连接
void USBH_HID_Reconnect(void)
{
	//关闭之前的连接
	USBH_DeInit(&USB_OTG_Core_dev,&USB_Host);	//复位USB HOST
	USB_OTG_StopHost(&USB_OTG_Core_dev);		//停止USBhost
	if(USB_Host.usr_cb->DeviceDisconnected)		//存在,才禁止
	{
		USB_Host.usr_cb->DeviceDisconnected(); 	//用户函数,设备被拔出了
		USBH_DeInit(&USB_OTG_Core_dev, &USB_Host);
		USB_Host.usr_cb->DeInit();//用户反初始化函数
		USB_Host.class_cb->DeInit(&USB_OTG_Core_dev,&USB_Host.device_prop);
	}
	USB_OTG_DisableGlobalInt(&USB_OTG_Core_dev);//关闭所有中断
	//重新复位USB

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS,DISABLE);
	memset(&USB_OTG_Core_dev,0,sizeof(USB_OTG_CORE_HANDLE));
	memset(&USB_Host,0,sizeof(USB_Host));
	//重新连接USB HID设备
	USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks);  
}



/**
* @brief  USBH_Process
*         USB主机核心主状态机处理
* @param  None 
* @retval None
*/
/*
处理逻辑：
状态机默认为空闲状态
1.检查主机端口的事件状态。若主机状态非空闲,又检测到没有设备时,把状态机设置为断开连接
2.状态机为空闲状态时,将状态机设置为等待prt,然后复位一次usb设备
3.状态机为等待prt状态时,期间不停的执行小延迟,等待端口被连接,若usb设备连接,则将状态机设置为已连接状态。
4.状态机为已连接状态时,复位管道,复位usb.访问usb设备,获取是什么速度的设备,同时打开所有管道
5.
*/
#define STATE_DBUG 0

volatile static u16 kale=0;
volatile u16 int_noresponse=0;
	
//主要两个问题,识别低速设备后,提示设备被拔出,也就是异常中断了
//一个是无限卡中断里

void USBH_Process(USB_OTG_CORE_HANDLE *pdev , USBH_HOST *phost)
{
  volatile USBH_Status status = USBH_FAIL;

	/* check for Host port events */
	/* 检查主机端口事件 */
  if (((HCD_IsDeviceConnected(pdev) == 0)|| (HCD_IsPortEnabled(pdev) == 0))&& (phost->gState != HOST_IDLE)) 
  {
    if(phost->gState != HOST_DEV_DISCONNECTED) 
    {
      phost->gState = HOST_DEV_DISCONNECTED;	
    }
  }
   
  //状态机
  switch (phost->gState)
  {
  
  case HOST_IDLE ://空闲状态
	 vTaskDelay(50);
  
	#if STATE_DBUG
	printf("状态机为空闲状态\r\n");
	#endif
    if (HCD_IsDeviceConnected(pdev))  
    {
      phost->gState = HOST_WAIT_PRT_ENABLED;//将状态设置为等待prt使能
		vTaskDelay(100);
      HCD_ResetPort(pdev);//提交复位,实际有用的函数
      phost->usr_cb->ResetDevice();//复位usb客户回调函数void USBH_USR_ResetDevice(void)
    }
    break;
    
  case HOST_WAIT_PRT_ENABLED://等待prt使能
	#if STATE_DBUG
	printf("状态机进入PRT等待状态！\r\n");
	#endif

    if (pdev->host.PortEnabled == 1)
    { 
      phost->gState = HOST_DEV_ATTACHED; 
		vTaskDelay(50);
    }
    break;
      
  case HOST_DEV_ATTACHED ://设备已连接
	  int_noresponse=0;
	#if STATE_DBUG
	printf("状态机进入设备连接状态！\r\n");
	#endif

    phost->usr_cb->DeviceAttached();//调用用户端函数,这里调用的是void USBH_USR_DeviceAttached(void)
	
	//给设备分配新的通道
    phost->Control.hc_num_out = USBH_Alloc_Channel(pdev, 0x00);
    phost->Control.hc_num_in = USBH_Alloc_Channel(pdev, 0x80);  
	
    //复位usb
    if ( HCD_ResetPort(pdev) == 0)
    {
      phost->usr_cb->ResetDevice();//调用用户端函数,这里是void USBH_USR_ResetDevice(void)
     
	/* Host is Now ready to start the Enumeration */
	/* 主机现在已准备好开始枚举 */
      phost->device_prop.speed = HCD_GetCurrentSpeed(pdev);//获取设备的速度
      phost->gState = HOST_ENUMERATION;//设置主机状态为准备枚举
      phost->usr_cb->DeviceSpeedDetected(phost->device_prop.speed);//调用用户端函数，提示获取到的设备速度。这里调用的是void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
      vTaskDelay(50);
		
	/* Open Control pipes */	
	/* 打开控制管道 */
	USBH_Open_Channel (pdev,
					 phost->Control.hc_num_in,
					 phost->device_prop.address,
					 phost->device_prop.speed,
					 EP_TYPE_CTRL,
					 phost->Control.ep0size); 
      
	/* Open Control pipes */
	/* 打开控制管道 */
	USBH_Open_Channel (pdev,
					 phost->Control.hc_num_out,
					 phost->device_prop.address,
					 phost->device_prop.speed,
					 EP_TYPE_CTRL,
					 phost->Control.ep0size);          
    }
    break;
    
  case HOST_ENUMERATION://准备枚举状态 
	  int_noresponse=0;
	#if STATE_DBUG
	printf("状态机进入枚举状态\r\n");
	#endif
	
	vTaskDelay(5);
	/* Check for enumeration status */  
	/* 检查枚举状态 */
    if ( USBH_HandleEnum(pdev , phost) == USBH_OK)//枚举完成 
    { 
		/* The function shall return USBH_OK when full enumeration is complete */
		/* user callback for end of device basic enumeration */
		/* 当完整枚举完成时，该函数应返回USBH_OK */
		/* 用户回调函数，用于设备基本枚举结束 */
		phost->usr_cb->EnumerationDone(); //调用USBH_USR_EnumerationDone
      phost->gState  = HOST_USR_INPUT;
    }
	else
	{
		enum_error++;//枚举错误
		if( enum_error>50 ) NVIC_SystemReset();//枚举失败次数太多,避免卡死,重启系统
	}

    break;//若枚举没有完成,跳过本次循环继续等待枚举
    	
	
  case HOST_USR_INPUT://用户确认状态，等待用户按键确认。备注：一般不要用户二次确认,在对应函数直接返回ok
	  enum_error = 0;
	#if STATE_DBUG
	printf("状态机进入二次确认状态！\r\n");
	#endif
	/*The function should return user response true to move to class state */
	/* 该函数应返回用户响应true以转到类状态 */
	//用户按键确认后,进入类请求状态。使用时不要二次确认,直接返回USBH_USR_RESP_OK就好
	//这里调用的是用户函数UserInput()等待用户确认,实际调用函数为USBH_USR_Status USBH_USR_UserInput(void)
    if ( phost->usr_cb->UserInput() == USBH_USR_RESP_OK)
    {
      if((phost->class_cb->Init(pdev, phost))\
        == USBH_OK) //用户确认后,调用了设备初始化函数,这里实际调用的函数是
	                //static USBH_Status USBH_HID_InterfaceInit ( USB_OTG_CORE_HANDLE *pdev, void *phost)
					//注意这里面对设备的实例进行了区分，从抽象层转换到了实际应用层。函数HID_Machine.cb = 也被指定了,后续调用会用到
      {	

        phost->gState  = HOST_CLASS_REQUEST;//初始化成功,状态机进入类请求模式
      }     
    }   
    break;
    
  case HOST_CLASS_REQUEST://进入类请求模式
	/* process class standard control requests state machine */ 
	/* 进程类标准控制请求状态机 */
	#if STATE_DBUG
	printf("状态机进入请求设备类数据的状态！\r\n");
	#endif
  
	kale = 0;
  //这里phost->class_cb->Requests(pdev, phost);调用的函数是
  //static USBH_Status USBH_HID_ClassRequest(USB_OTG_CORE_HANDLE *pdev, void *phost)
	status = phost->class_cb->Requests(pdev, phost);
  
	//获取设备的描述符等信息,等待执行完毕
	if(status == USBH_OK)
	{
		phost->gState  = HOST_CLASS;//描述符、协议等信息设置完毕,进入下一个状态
	}  
	else
	{
		USBH_ErrorHandle(phost, status);//设备描述符信息请求失败
	}
    break;    
	
  case HOST_CLASS: //状态机到这里就结束了,后面是不断的读取数据、处理数据的过程
	#if STATE_DBUG
	printf("状态机进入数据处理状态！\r\n");
	#endif
  
	int_noresponse = 0;
  
	/* process class state machine */
	/* 流程类状态机 */
	//数据处理函数。这里调用的是static USBH_Status USBH_HID_Handle(USB_OTG_CORE_HANDLE *pdev , void *phost)
    status = phost->class_cb->Machine(pdev, phost);
  
	//status默认返回ok,无特殊情况不会响应错误处理函数
    USBH_ErrorHandle(phost, status);
    break;       
    
  case HOST_CTRL_XFER:
	#if STATE_DBUG
	printf("状态机进入类识别传输状态%d！！！！！！！\r\n",kale);
	#endif
	vTaskDelay(5);
	if(kale>150)
	{
		kale=0,phost->gState = HOST_DEV_DISCONNECTED;//初始化卡注了,让状态机进入复位模式
	}		
	kale++;
    /* process control transfer state machine */
  /* 进程控制传输状态机 */
    USBH_HandleControl(pdev, phost);    
    break;
  
#if defined (USB_OTG_FS_LOW_PWR_MGMT_SUPPORT) || defined (USB_OTG_HS_LOW_PWR_MGMT_SUPPORT)
  case HOST_SUSPENDED:
    if (USBH_SetDeviceFeature(pdev, phost, FEATURE_SELECTOR_DEVICE, 0)==USBH_OK)
    {
      suspend_flag = 1;
      USB_OTG_BSP_Suspend(pdev);
      phost->usr_cb->UserInput();
      PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
      /* After wakeup got to HOST_WAKEUP state */
      phost->gState  = HOST_WAKEUP;
    }
    break;
      
  case HOST_WAKEUP:
    /* issue  a ClearDeviceFeature request */
    if (USBH_ClearDeviceFeature(pdev, phost, FEATURE_SELECTOR_DEVICE, 0)==USBH_OK)
    {
      phost->gState  = HOST_USR_INPUT;
    }
    break;
#endif /* USE_HOST_MODE */
  case HOST_ERROR_STATE://错误状态,由USBH_ErrorHandle函数进入
	#if STATE_DBUG
	printf("状态机进入错误！！！！！！！\r\n");
	#endif
    /* Re-Initialize Host for new Enumeration */
    USBH_DeInit(pdev, phost);
    phost->usr_cb->DeInit();
    phost->class_cb->DeInit(pdev, &phost->device_prop);
    break;
    
  case HOST_DEV_DISCONNECTED :
	#if STATE_DBUG
	printf("状态机进入断开连接状态\r\n");
	#endif
  
	usb_ps2_ready=0;//设备拔出

	USBH_HID_Reconnect();//重新初始化主机
  
    /* Manage User disconnect operations*/
	/* 管理用户断开连接的操作,调用用户端回调函数*/
//    phost->usr_cb->DeviceDisconnected();//设备被拔出了
//    
//    /* Re-Initialize Host for new Enumeration */
//	/* 为新的枚举重新初始化主机 */
//    USBH_DeInit(pdev, phost);
//    phost->usr_cb->DeInit(); //用户反初始化函数执行 调用 用户反初始化  USBH_USR_DeInit(用户层)
//    phost->class_cb->DeInit(pdev, &phost->device_prop);//调用 USBH_HID_InterfaceDeInit(抽象层)
//    USBH_DeAllocate_AllChannel(pdev);  //关闭所有管道 
//    phost->gState = HOST_IDLE;
//   
//	//IO口关闭
//	USB_OTG_DeInit();//IO口反初始化
//	
//	//中断关闭
//	USB_OTG_BSP_DisableInterrupt();
//  
////    /* Re-Initialize Host for new Enumeration */
////  /* 为新的枚举重新初始化主机 */
////    HCD_Init(pdev,USB_OTG_FS_CORE_ID);
//	
//	vTaskDelay(500);
//	
//	#if STATE_DBUG
//	printf("等待重新初始化");
//	#endif
//	//重新初始化主机
//	USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks);
//	#if STATE_DBUG
//	printf("已重新启动了一个HOST主机\r\n");
//	#endif
	
	
    break;
    
  default :
    break;
  }

}


/**
  * @brief  USBH_ErrorHandle 
  *         This function handles the Error on Host side.
  * @param  errType : Type of Error or Busy/OK state
  * @retval None
  */
/**
  * @brief USBH_ErrorHandle 
  * 此函数处理主机端的错误。
  * @param errType ： 错误类型或忙/确定状态
  * 无
  */
void USBH_ErrorHandle(USBH_HOST *phost, USBH_Status errType)
{
  /* Error unrecovered or not supported device speed */
	/* 未修复的错误或不支持的设备速度 */
  if ( (errType == USBH_ERROR_SPEED_UNKNOWN) ||
       (errType == USBH_UNRECOVERED_ERROR) )
  {
    phost->usr_cb->UnrecoveredError(); 
    phost->gState = HOST_ERROR_STATE;   
  }  
  /* USB host restart requested from application layer */
 /* 应用层请求重启 USB 主机 */ 
  else if(errType == USBH_APPLY_DEINIT)
  {
    phost->gState = HOST_ERROR_STATE;  
	/* user callback for initialization */
	/* 用户初始化回调 */
    phost->usr_cb->Init();
  } 
}


/**
  * @brief  USBH_HandleEnum 
  *         This function includes the complete enumeration process
  * @param  pdev: Selected device
  * @retval USBH_Status
  */
/**
  * @brief USBH_HandleEnum 
  * 该函数包括完整的枚举过程
  * @param pdev：选中的设备
  * @retval USBH_Status
  */
#define STATE_DBUG_Enum 0

static USBH_Status USBH_HandleEnum(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  USBH_Status Status = USBH_BUSY;  
  uint8_t Local_Buffer[64];
  
  switch (phost->EnumState)
  {
  case ENUM_IDLE:  
    /* Get Device Desc for only 1st 8 bytes : To get EP0 MaxPacketSize */
    if ( USBH_Get_DevDesc(pdev , phost, 8) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("第1步枚举ok\r\n");
		#endif
		
      phost->Control.ep0size = phost->device_prop.Dev_Desc.bMaxPacketSize;
      
      phost->EnumState = ENUM_GET_FULL_DEV_DESC;
      
      /* modify control channels configuration for MaxPacket size */
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_out,
                           0,
                           0,
                           0,
                           phost->Control.ep0size);
      
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_in,
                           0,
                           0,
                           0,
                           phost->Control.ep0size);      
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!第1步枚举不OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_FULL_DEV_DESC:  
    /* Get FULL Device Desc  */
    if ( USBH_Get_DevDesc(pdev, phost, USB_DEVICE_DESC_SIZE)\
      == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("第2步枚举ok\r\n");
		#endif
      /* user callback for device descriptor available */
      phost->usr_cb->DeviceDescAvailable(&phost->device_prop.Dev_Desc);      
      phost->EnumState = ENUM_SET_ADDR;
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!第2步枚举不OK\r\n");
		#endif
	}
    break;
   
  case ENUM_SET_ADDR: 
    /* set address */
    if ( USBH_SetAddress(pdev, phost, USBH_DEVICE_ADDRESS) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("第3步枚举ok\r\n");
		#endif
      USB_OTG_BSP_mDelay(2);
      phost->device_prop.address = USBH_DEVICE_ADDRESS;
      
      /* user callback for device address assigned */
      phost->usr_cb->DeviceAddressAssigned();
      phost->EnumState = ENUM_GET_CFG_DESC;
      
      /* modify control channels to update device address */
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_in,
                           phost->device_prop.address,
                           0,
                           0,
                           0);
      
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_out,
                           phost->device_prop.address,
                           0,
                           0,
                           0);         
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!第3步枚举不OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_CFG_DESC:  
    /* get standard configuration descriptor */
    if ( USBH_Get_CfgDesc(pdev, 
                          phost,
                          USB_CONFIGURATION_DESC_SIZE) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("第4步枚举ok\r\n");
		#endif
      /* before getting full config descriptor, check if it does not exceed 
      buffer size allocated to config descriptor USBH_MAX_DATA_BUFFER
      in the usbh_conf.h*/
      if (phost->device_prop.Cfg_Desc.wTotalLength <= USBH_MAX_DATA_BUFFER)
      {
        phost->EnumState = ENUM_GET_FULL_CFG_DESC;
      }
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!第4步枚举不OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_FULL_CFG_DESC:  
    /* get FULL config descriptor (config, interface, endpoints) */
    if (USBH_Get_CfgDesc(pdev, 
                         phost,
                         phost->device_prop.Cfg_Desc.wTotalLength) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("第5步枚举ok\r\n");
		#endif
      /* User callback for configuration descriptors available */
      phost->usr_cb->ConfigurationDescAvailable(&phost->device_prop.Cfg_Desc,
                                                      phost->device_prop.Itf_Desc,
                                                      phost->device_prop.Ep_Desc[0]);
      
      phost->EnumState = ENUM_GET_MFC_STRING_DESC;
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!第5步枚举不OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_MFC_STRING_DESC:  
    if (phost->device_prop.Dev_Desc.iManufacturer != 0)
    { /* Check that Manufacturer String is available */
      
      if ( USBH_Get_StringDesc(pdev,
                               phost,
                               phost->device_prop.Dev_Desc.iManufacturer, 
                               Local_Buffer , 
                               0xff) == USBH_OK)
      {
		#if STATE_DBUG_Enum
		printf("第6步枚举ok\r\n");
		#endif
        /* User callback for Manufacturing string */
        phost->usr_cb->ManufacturerString(Local_Buffer);
        phost->EnumState = ENUM_GET_PRODUCT_STRING_DESC;
      }
    }
    else
    {
		#if STATE_DBUG_Enum
		printf("第6步枚举ok\r\n");
		#endif
      phost->usr_cb->ManufacturerString("N/A");      
      phost->EnumState = ENUM_GET_PRODUCT_STRING_DESC;
    }
    break;
    
  case ENUM_GET_PRODUCT_STRING_DESC:   
    if (phost->device_prop.Dev_Desc.iProduct != 0)
    { /* Check that Product string is available */
      if ( USBH_Get_StringDesc(pdev,
                               phost,
                               phost->device_prop.Dev_Desc.iProduct, 
                               Local_Buffer, 
                               0xff) == USBH_OK)
      {
		#if STATE_DBUG_Enum
		printf("第7步枚举ok\r\n");
		#endif
        /* User callback for Product string */
        phost->usr_cb->ProductString(Local_Buffer);
        phost->EnumState = ENUM_GET_SERIALNUM_STRING_DESC;
      }
    }
    else
    {
		#if STATE_DBUG_Enum
		printf("第7步枚举ok\r\n");
		#endif
      phost->usr_cb->ProductString("N/A");
      phost->EnumState = ENUM_GET_SERIALNUM_STRING_DESC;
    } 
    break;
    
  case ENUM_GET_SERIALNUM_STRING_DESC:   
    if (phost->device_prop.Dev_Desc.iSerialNumber != 0)
    { /* Check that Serial number string is available */    
      if ( USBH_Get_StringDesc(pdev, 
                               phost,
                               phost->device_prop.Dev_Desc.iSerialNumber, 
                               Local_Buffer, 
                               0xff) == USBH_OK)
      {
		#if STATE_DBUG_Enum
		printf("第8步枚举ok\r\n");
		#endif
        /* User callback for Serial number string */
        phost->usr_cb->SerialNumString(Local_Buffer);
        phost->EnumState = ENUM_SET_CONFIGURATION;
      }
    }
    else
    {
		#if STATE_DBUG_Enum
		printf("第8步枚举ok\r\n");
		#endif
      phost->usr_cb->SerialNumString("N/A");      
      phost->EnumState = ENUM_SET_CONFIGURATION;
    }  
    break;
      
  case ENUM_SET_CONFIGURATION:
    /* set configuration  (default config) */
    if (USBH_SetCfg(pdev, 
                    phost,
                    phost->device_prop.Cfg_Desc.bConfigurationValue) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("第9步枚举ok\r\n");
		#endif
      phost->EnumState = ENUM_DEV_CONFIGURED;
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!第9步枚举不OK\r\n");
		#endif
	}
    break;

    
  case ENUM_DEV_CONFIGURED:
		#if STATE_DBUG_Enum
		printf("第10步枚举ok\r\n");
		#endif
    /* user callback for enumeration done */
    Status = USBH_OK;
    break;
    
  default:
    break;
  }  
  return Status;
}


/**
  * @brief  USBH_HandleControl
  *         Handles the USB control transfer state machine
  * @param  pdev: Selected device
  * @retval Status
  */
/**
  * USBH_HandleControl
  * 处理 USB 控制传输状态机
  * @param pdev：选中的设备
  * @retval 状态
  */
USBH_Status USBH_HandleControl (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  uint8_t direction;  
  static uint16_t timeout = 0;
  USBH_Status status = USBH_OK;
  URB_STATE URB_Status = URB_IDLE;
  
  phost->Control.status = CTRL_START;

  
  switch (phost->Control.state)
  {
  case CTRL_SETUP:
    /* send a SETUP packet */
    USBH_CtlSendSetup     (pdev, 
	                   phost->Control.setup.d8 , 
	                   phost->Control.hc_num_out);  
    phost->Control.state = CTRL_SETUP_WAIT;  
    break; 
    
  case CTRL_SETUP_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_out); 
    /* case SETUP packet sent successfully */
    if(URB_Status == URB_DONE)
    { 
      direction = (phost->Control.setup.b.bmRequestType & USB_REQ_DIR_MASK);
      
      /* check if there is a data stage */
      if (phost->Control.setup.b.wLength.w != 0 )
      {        
        timeout = DATA_STAGE_TIMEOUT;
        if (direction == USB_D2H)
        {
          /* Data Direction is IN */
          phost->Control.state = CTRL_DATA_IN;
        }
        else
        {
          /* Data Direction is OUT */
          phost->Control.state = CTRL_DATA_OUT;
        } 
      }
      /* No DATA stage */
      else
      {
        timeout = NODATA_STAGE_TIMEOUT;
        
        /* If there is No Data Transfer Stage */
        if (direction == USB_D2H)
        {
          /* Data Direction is IN */
          phost->Control.state = CTRL_STATUS_OUT;
        }
        else
        {
          /* Data Direction is OUT */
          phost->Control.state = CTRL_STATUS_IN;
        } 
      }          
      /* Set the delay timer to enable timeout for data stage completion */
      phost->Control.timer = HCD_GetCurrentFrame(pdev);
    }
    else if(URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;     
      phost->Control.status = CTRL_XACTERR;
    }    
    break;
    
  case CTRL_DATA_IN:  
    /* Issue an IN token */ 
    USBH_CtlReceiveData(pdev,
                        phost->Control.buff, 
                        phost->Control.length,
                        phost->Control.hc_num_in);
 
    phost->Control.state = CTRL_DATA_IN_WAIT;
    break;    
    
  case CTRL_DATA_IN_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_in); 
    
    /* check is DATA packet transferred successfully */
    if  (URB_Status == URB_DONE)
    { 
      phost->Control.state = CTRL_STATUS_OUT;
    }
   
    /* manage error cases*/
    if  (URB_Status == URB_STALL) 
    { 
      /* In stall case, return to previous machine state*/
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_STALLED;  
    }   
    else if (URB_Status == URB_ERROR)
    {
      /* Device error */
      phost->Control.state = CTRL_ERROR;    
    }
    else if ((HCD_GetCurrentFrame(pdev)- phost->Control.timer) > timeout)
    {
      /* timeout for IN transfer */
      phost->Control.state = CTRL_ERROR; 
    }   
    break;
    
  case CTRL_DATA_OUT:
    /* Start DATA out transfer (only one DATA packet)*/
    pdev->host.hc[phost->Control.hc_num_out].toggle_out = 1; 
        
    USBH_CtlSendData (pdev,
                      phost->Control.buff, 
                      phost->Control.length , 
                      phost->Control.hc_num_out);
    
    phost->Control.state = CTRL_DATA_OUT_WAIT;
    break;
    
  case CTRL_DATA_OUT_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_out);     
    if  (URB_Status == URB_DONE)
    { /* If the Setup Pkt is sent successful, then change the state */
      phost->Control.state = CTRL_STATUS_IN;
    }
    
    /* handle error cases */
    else if  (URB_Status == URB_STALL) 
    { 
      /* In stall case, return to previous machine state*/
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_STALLED;  
    } 
    else if  (URB_Status == URB_NOTREADY)
    { 
      /* Nack received from device */
      phost->Control.state = CTRL_DATA_OUT;
    }    
    else if (URB_Status == URB_ERROR)
    {
      /* device error */
      phost->Control.state = CTRL_ERROR;      
    } 
    break;
    
    
  case CTRL_STATUS_IN:
    /* Send 0 bytes out packet */
    USBH_CtlReceiveData (pdev,
                         0,
                         0,
                         phost->Control.hc_num_in);
    
    phost->Control.state = CTRL_STATUS_IN_WAIT;
    
    break;
    
  case CTRL_STATUS_IN_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_in); 
    
    if  ( URB_Status == URB_DONE)
    { /* Control transfers completed, Exit the State Machine */
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_COMPLETE;
    }
    
    else if (URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;  
    }
    
    else if((HCD_GetCurrentFrame(pdev)\
      - phost->Control.timer) > timeout)
    {
      phost->Control.state = CTRL_ERROR; 
    }
     else if(URB_Status == URB_STALL)
    {
      /* Control transfers completed, Exit the State Machine */
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_STALLED;
      status = USBH_NOT_SUPPORTED;
    }
    break;
    
  case CTRL_STATUS_OUT:
    pdev->host.hc[phost->Control.hc_num_out].toggle_out ^= 1; 
    USBH_CtlSendData (pdev,
                      0,
                      0,
                      phost->Control.hc_num_out);
    
    phost->Control.state = CTRL_STATUS_OUT_WAIT;
    break;
    
  case CTRL_STATUS_OUT_WAIT: 
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_out);  
    if  (URB_Status == URB_DONE)
    { 
      phost->gState =   phost->gStateBkp; 
      phost->Control.state = CTRL_COMPLETE; 
    }
    else if  (URB_Status == URB_NOTREADY)
    { 
      phost->Control.state = CTRL_STATUS_OUT;
    }      
    else if (URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;      
    }
    break;
    
  case CTRL_ERROR:
    /* 
    After a halt condition is encountered or an error is detected by the 
    host, a control endpoint is allowed to recover by accepting the next Setup 
    PID; i.e., recovery actions via some other pipe are not required for control
    endpoints. For the Default Control Pipe, a device reset will ultimately be 
    required to clear the halt or error condition if the next Setup PID is not 
    accepted.
    */
  /*遇到停止条件或主机检测到错误后 
    主机检测到错误后，控制端点可通过接受下一个设置 
    也就是说，控制端点不需要通过其他管道进行恢复操作。
    即控制端点不需要通过其他管道进行恢复操作。对于默认控制管道，最终需要重置设备 
    如果不接受下一个设置 PID，则最终需要重置设备以清除停止或错误状态。*/
    if (++ phost->Control.errorcount <= USBH_MAX_ERROR_COUNT)
    {
      /* Do the transmission again, starting from SETUP Packet */
      phost->Control.state = CTRL_SETUP; 
    }
    else
    {
      phost->Control.status = CTRL_FAIL;
      phost->gState =   phost->gStateBkp;
      
      status = USBH_FAIL;
    }
    break;
    
  default:
    break;
  }
  return status;
}

/**
  * @brief  USB_OTG_BSP_Resume
  *         Handles the USB Resume from Suspend Mode
  * @param  pdev: Selected device
  * @retval Status
  */
/**
  * USB_OTG_BSP_Resume
  * 处理 USB 从挂起模式恢复
  * @param pdev：选中的设备
  * @retval 状态
  */
void USB_OTG_BSP_Resume(USB_OTG_CORE_HANDLE *pdev)                                                                 
{
  USB_OTG_HPRT0_TypeDef    hprt0;
  USB_OTG_PCGCCTL_TypeDef  power;
  hprt0.d32  = 0;
  /*  switch-off the clocks */
  power.d32 = 0;
  power.b.stoppclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL,power.d32, 0 );  
  
  power.b.gatehclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL,power.d32, 0);
  
  hprt0.d32  = 0;   
  hprt0.d32  = USB_OTG_ReadHPRT0(pdev);
  hprt0.b.prtsusp = 0;
  hprt0.b.prtres = 1;
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
  USB_OTG_BSP_mDelay (20);
  hprt0.b.prtres = 0;
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32); 
}

/**
  * @brief  USB_OTG_BSP_Suspend
  *         Handles the Enter USB to Suspend Mode
  * @param  pdev: Selected device
  * @retval Status
  */
/**
  * 处理进入 USB 挂起模式。
  * 处理进入 USB 挂起模式
  * @param pdev：选中的设备
  * @retval 状态
  */
void USB_OTG_BSP_Suspend(USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_HPRT0_TypeDef    hprt0;
  USB_OTG_PCGCCTL_TypeDef  power;
  hprt0.d32  = 0;
  hprt0.d32  = USB_OTG_ReadHPRT0(pdev);
  hprt0.b.prtsusp = 1; 
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
  
  /*  switch-off the clocks */
  power.d32 = 0;
  power.b.stoppclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL, 0, power.d32);  
  
  power.b.gatehclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL, 0, power.d32);
}


/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
