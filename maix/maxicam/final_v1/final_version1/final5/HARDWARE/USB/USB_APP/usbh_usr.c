/**
  ******************************************************************************
  * @file    usbh_usr.c
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file includes the user application layer
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

/* Includes ------------------------------------------------------------------ */
#include "usbh_usr.h"
#include "usb_hcd_int.h"
#include "usbh_hid_joy.h"
#include "system.h"

/** @addtogroup USBH_USER
* @{
*/

/** @addtogroup USBH_HID_DEMO_USER_CALLBACKS
* @{
*/

/** @defgroup USBH_USR
* @brief This file is the Header file for usbh_usr.c
* @{
*/


/** @defgroup USBH_CORE_Exported_TypesDefinitions
* @{
*/
#define STATE_DBUG 0

u8 usb_wait_EnumReady=EnumDone;
u16 enum_error=0;

extern volatile u16 int_noresponse;
void OTG_FS_IRQHandler(void)
{
	#if STATE_DBUG
	printf("进入中断\r\n");
	#endif
	int_noresponse++;
	if( int_noresponse>3000 ) 
	{
		int_noresponse=0;
		USB_OTG_BSP_DisableInterrupt();
		USB_Host.gState=HOST_DEV_DISCONNECTED;
		return;
	} 
	
	USBH_OTG_ISR_Handler(&USB_OTG_Core_dev);
}


/* Points to the DEVICE_PROP structure of current device */
/* The purpose of this register is to speed up the execution */
/* 指向当前设备的DEVICE_PROP结构体 */
/* 此寄存器的目的是加速执行 */
//指定用户的各个函数具体内容
USBH_Usr_cb_TypeDef USR_Callbacks = {
  USBH_USR_Init,//void (*Init)(void); 
  USBH_USR_DeInit,//void (*DeInit)(void); 
  USBH_USR_DeviceAttached,//void (*DeviceAttached)(void); 
  USBH_USR_ResetDevice,//void (*ResetDevice)(void);
  USBH_USR_DeviceDisconnected,//void (*DeviceDisconnected)(void); 
  USBH_USR_OverCurrentDetected,//void (*OverCurrentDetected)(void);  
  USBH_USR_DeviceSpeedDetected,//void (*DeviceSpeedDetected)(uint8_t DeviceSpeed);  
  USBH_USR_Device_DescAvailable,//void (*DeviceDescAvailable)(void *); 
  USBH_USR_DeviceAddressAssigned,//void (*DeviceAddressAssigned)(void);
  USBH_USR_Configuration_DescAvailable,//void (*ConfigurationDescAvailable)(USBH_CfgDesc_TypeDef *,USBH_InterfaceDesc_TypeDef *, USBH_EpDesc_TypeDef *); 
  USBH_USR_Manufacturer_String,// void (*ManufacturerString)(void *); 
  USBH_USR_Product_String,//void (*ProductString)(void *);
  USBH_USR_SerialNum_String,//void (*SerialNumString)(void *); 
  USBH_USR_EnumerationDone,//void (*EnumerationDone)(void); 
  USBH_USR_UserInput,//USBH_USR_Status (*UserInput)(void);
  0,// int  (*UserApplication) (void);
  USBH_USR_DeviceNotSupported,// void (*DeviceNotSupported)(void); 
  USBH_USR_UnrecoveredError//void (*UnrecoveredError)(void);
};

/**
* @}
*/

/** @defgroup USBH_USR_Private_Constants
* @{
*/

/**
* @}
*/



/** @defgroup USBH_USR_Private_FunctionPrototypes
* @{
*/
/**
* @}
*/


/** @defgroup USBH_USR_Private_Functions
* @{
*/





/**
* @brief  USBH_USR_Init
*         Displays the message on LCD for host lib initialization
* @param  None
* @retval None
*/
//用户操作的初始化函数,此处初始化LCD
void USBH_USR_Init(void)
{
	#if STATE_DBUG
	printf("用户初始化函数\r\n");
	#endif
}

/**
* @brief  USBH_USR_DeviceAttached
*         Displays the message on LCD on device attached
* @param  None
* @retval None
*/
void USBH_USR_DeviceAttached(void)
{
	#if STATE_DBUG
	printf("检测到usb设备插入\r\n");
	#endif
}

/**
* @brief  USBH_USR_UnrecoveredError
* @param  None
* @retval None
*/
void USBH_USR_UnrecoveredError(void)
{
	#if STATE_DBUG
	printf("检测到无法恢复的错误\r\n");
	#endif

}

/**
* @brief  USBH_DisconnectEvent
*         Device disconnect event
* @param  None
* @retval None
*/
void USBH_USR_DeviceDisconnected(void)
{
	#if STATE_DBUG
	printf("设备被拔出了\r\n");
	#endif
}

/**
* @brief  USBH_USR_ResetUSBDevice
*         Reset USB Device
* @param  None
* @retval None
*/
void USBH_USR_ResetDevice(void)
{
	#if STATE_DBUG
	printf("用户定义的复位设备函数\r\n");
	#endif
  /* Users can do their application actions here for the USB-Reset */

}


/**
* @brief  USBH_USR_DeviceSpeedDetected
*         Displays the message on LCD for device speed
* @param  Devicespeed : Device Speed
* @retval None
*/
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
	//设备接入,等待usb完成枚举
	usb_wait_EnumReady = EnumWait;
	
  if(DeviceSpeed == HPRT0_PRTSPD_HIGH_SPEED)
  {
	#if STATE_DBUG
	printf("高速HS usb 设备\r\n");
	#endif
  }  
  else if(DeviceSpeed == HPRT0_PRTSPD_FULL_SPEED)
  {
	#if STATE_DBUG
	printf("高全速FS usb 设备\r\n");
	#endif

  }
  else if(DeviceSpeed == HPRT0_PRTSPD_LOW_SPEED)
  {
	#if STATE_DBUG
	printf("低速LS usb 设备\r\n");
	#endif
	
  }
  else
  {
	#if STATE_DBUG
	printf("无法识别的错误\r\n");
	#endif
  }
}

/**
* @brief  USBH_USR_Device_DescAvailable
*         Displays the message on LCD for device descriptor
* @param  DeviceDesc : device descriptor
* @retval None
*/
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
	USBH_DevDesc_TypeDef *hs;
	hs = DeviceDesc;  
	
	if( (uint32_t)(*hs).idVendor==Normal_PS2_VID && (uint32_t)(*hs).idProduct==Normal_PS2_PID )
	{
		//有线ps2手柄
		ps2_type = Normal_PS2;
		
		#if STATE_DBUG
		printf("有线ps2\r\n");
		#endif
	}
	else if( (uint32_t)(*hs).idVendor == _2_4G_Android_PS2_VID && (uint32_t)(*hs).idProduct == _2_4G_Android_PS2_PID )
	{
		//无线ps2手柄
		ps2_type = _2_4G_Android_PS2;
//		printf("无线ps2安卓模式\r\n");
		#if STATE_DBUG
		printf("无线ps2安卓模式\r\n");
		#endif
	}
	else if( (uint32_t)(*hs).idVendor == _2_4G_PC_PS2_VID && (uint32_t)(*hs).idProduct == _2_4G_PC_PS2_PID )
	{
		//无线ps2手柄
		ps2_type = _2_4G_PC_PS2;
//		printf("无线ps2PC模式\r\n");
		#if STATE_DBUG
		printf("无线ps2PC模式\r\n");
		#endif
	}
	
	#if STATE_DBUG
	//设备的描述符 VID与PID
	printf("VID:%04Xh\r\n", (uint32_t)(*hs).idVendor);
	printf("PID:%04Xh\r\n", (uint32_t)(*hs).idProduct);
	#endif
//	printf("VID:%04Xh,%d\r\n", (uint32_t)(*hs).idVendor,(uint32_t)(*hs).idVendor);
//	printf("PID:%04Xh,%d\r\n", (uint32_t)(*hs).idProduct,(uint32_t)(*hs).idProduct);
}

/**
* @brief  USBH_USR_DeviceAddressAssigned
*         USB device is successfully assigned the Address
* @param  None
* @retval None
*/
void USBH_USR_DeviceAddressAssigned(void)
{
	#if STATE_DBUG
//从机地址分配成功
	printf("从机地址分配成功\r\n");
	#endif
}


/**
* @brief  USBH_USR_Conf_Desc
*         Displays the message on LCD for configuration descriptor
* @param  ConfDesc : Configuration descriptor
* @retval None
*/
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef * itfDesc,
                                          USBH_EpDesc_TypeDef * epDesc)
{
  USBH_InterfaceDesc_TypeDef *id;

  id = itfDesc;  
  
  if((*id).bInterfaceClass  == 0x08)
  {
	#if STATE_DBUG
	//识别到可移动存储器设备
	  printf("识别到可移动存储器设备\r\n");
	#endif

  }
  else if((*id).bInterfaceClass  == 0x03)
  {
	#if STATE_DBUG
    //识别到HID设备
	  printf("识别到HID设备\r\n");
	#endif
  }   
}

/**
* @brief  USBH_USR_Manufacturer_String
*         Displays the message on LCD for Manufacturer String
* @param  ManufacturerString : Manufacturer String of Device
* @retval None
*/
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
	#if STATE_DBUG
  //制造商的设备字符
  printf("Manufacturer : %s\r\n", (char *)ManufacturerString);
	#endif

}

/**
* @brief  USBH_USR_Product_String
*         Displays the message on LCD for Product String
* @param  ProductString : Product String of Device
* @retval None
*/
void USBH_USR_Product_String(void *ProductString)
{
	#if STATE_DBUG
	//接入设备的产品名称
	printf("Product : %s\r\n", (char *)ProductString);
	#endif
}

/**
* @brief  USBH_USR_SerialNum_String
*         Displays the message on LCD for SerialNum_String
* @param  SerialNumString : SerialNum_String of device
* @retval None
*/
void USBH_USR_SerialNum_String(void *SerialNumString)
{
	#if STATE_DBUG
	//产品的序列号
	printf("Serial Number : %s\n", (char *)SerialNumString);
	#endif

}

/**
* @brief  EnumerationDone
*         User response request is displayed to ask for
*         application jump to class
* @param  None
* @retval None
*/
void USBH_USR_EnumerationDone(void)
{
	#if STATE_DBUG
	//设备枚举已完成
	printf("设备枚举已完成\r\n");
	#endif
	usb_wait_EnumReady = EnumDone;
}

/**
* @brief  USBH_USR_DeviceNotSupported
*         Device is not supported
* @param  None
* @retval None
*/
void USBH_USR_DeviceNotSupported(void)
{
	#if STATE_DBUG
	//无法识别的usb
	printf("无法识别的usb\r\n");
	#endif
}


/**
* @brief  USBH_USR_UserInput
*         User Action for application state entry
* @param  None
* @retval USBH_USR_Status : User response for key button
*/
USBH_USR_Status USBH_USR_UserInput(void)
{
	#if STATE_DBUG
	//USB设备连接成功
	printf("USB设备连接成功\r\n");
	#endif

	return USBH_USR_RESP_OK;
}

/**
* @brief  USBH_USR_OverCurrentDetected
*         Device Overcurrent detection event
* @param  None
* @retval None
*/
void USBH_USR_OverCurrentDetected(void)
{
	#if STATE_DBUG
printf("设备端口电流过大\r\n");
	#endif
	

}

/**
* @brief  USR_MOUSE_Init
*         Init Mouse window
* @param  None
* @retval None
*/
//void USR_MOUSE_Init(void)
//{
//	printf("鼠标初始化\r\n");
//}

/**
* @brief  USR_MOUSE_ProcessData
*         Process Mouse data
* @param  data : Mouse data to be displayed
* @retval None
*/
/*
 @brief  USR_MOUSE_ProcessData
         处理鼠标数据
 @param  data : 要显示的鼠标数据
 @
*/
//void USR_MOUSE_ProcessData(HID_MOUSE_Data_TypeDef * data)
//{
//	printf("鼠标数据处理\r\n");
//}

/**
* @brief  USR_KEYBRD_Init
*         Init Keyboard window
* @param  None
* @retval None
*/
//void USR_KEYBRD_Init(void)
//{

//	printf("键盘初始化\r\n");
//}


/**
* @brief  USR_KEYBRD_ProcessData
*         Process Keyboard data
* @param  data : Keyboard data to be displayed
* @retval None
*/
//void USR_KEYBRD_ProcessData(uint8_t data)
//{
//	printf("键盘数据处理\r\n");
//}

/**
* @brief  USBH_USR_DeInit
*         Deinit User state and associated variables
* @param  None
* @retval None
*/
void USBH_USR_DeInit(void)
{
	#if STATE_DBUG
	printf("用户反初始化函数执行\r\n");
	#endif
	
}

//usb死机检测函数
u8 USBH_Check_HIDCommDead(USB_OTG_CORE_HANDLE *pcore,HID_Machine_TypeDef *phidm)
{
 	if(pcore->host.HC_Status[phidm->hc_num_in]==HC_DATATGLERR)//检测到DTERR错误
	{  
		return 1;
	}
	return 0;
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
