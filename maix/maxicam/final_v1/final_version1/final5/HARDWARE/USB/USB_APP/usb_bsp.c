/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file implements the board support package for the USB host library
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

#include "usb_bsp.h"


void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE * pdev)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);//使能USB OTG时钟	
	//GPIOA11,A12设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;//PA11/12复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_OTG_FS);//PA11,AF10(USB)
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_OTG_FS);//PA12,AF10(USB)
}

void USB_OTG_DeInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, DISABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
}

/**
  * @brief  USB_OTG_BSP_EnableInterrupt
  *         Configures USB Global interrupt
  * @param  None
  * @retval None
  */
/*
   @brief  USB_OTG_BSP_EnableInterrupt
           配置USB全局中断
   @param  None
   @retval None
  */
void USB_OTG_BSP_EnableInterrupt(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能通道
	NVIC_Init(&NVIC_InitStructure);//配置  
}

//USB OTG 中断设置,开启USB FS中断
//pdev:USB OTG内核结构体指针
void USB_OTG_BSP_DisableInterrupt(void)
{ 
	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//使能通道
	NVIC_Init(&NVIC_InitStructure);//配置  
}

void USB_OTG_BSP_DriveVBUS(USB_OTG_CORE_HANDLE *pdev, uint8_t state)
{ 
}

/**
  * @brief  USB_OTG_BSP_ConfigVBUS
  *         Configures the IO for the Vbus and OverCurrent
  * @param  None
  * @retval None
  */
/*
   @brief  USB_OTG_BSP_ConfigVBUS
           配置用于Vbus和过电流的IO
   @param  None
   @retval None
  */
//同样是普通IO
void USB_OTG_BSP_ConfigVBUS(USB_OTG_CORE_HANDLE * pdev)
{
}

/**
  * @brief  USB_OTG_BSP_uDelay
  *         This function provides delay time in micro sec
  * @param  usec : Value of delay required in micro sec
  * @retval None
  */
//延迟函数,直接用delay代替
void USB_OTG_BSP_uDelay(const uint32_t usec)
{
//	delay_us(usec);
	__nop();
	__nop();
	
}


/**
  * @brief  USB_OTG_BSP_mDelay
  *          This function provides delay time in milli sec
  * @param  msec : Value of delay required in milli sec
  * @retval None
  */
//延迟函数,直接用delay代替

void USB_OTG_BSP_mDelay(const uint32_t msec)
{
	delay_ms(msec);
//	vTaskDelay(msec);
}


