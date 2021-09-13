/**
 *******************************Copyright (c)************************************
 *
 *                   (c) Copyright 2021, CY, China, QD.
 *                           All Rights Reserved
 *
 *                           By(南京万福祥电子科技有限公司)
 *                           http://www.njwfx.com
 *
 *----------------------------------文件信息------------------------------------
 * 文件名称: bsp_delay.h
 * 创建人员: CY
 * 创建日期: 2021-04-17
 * 文档描述: 
 *
 *----------------------------------版本信息------------------------------------
 * 实验平台
 *          |--
 * 版本代号: V1.0
 * 版本说明: 
 *          |-1.0
 * 	         |-2021-04-17
 * 	          |-初始版本
 *------------------------------------------------------------------------------
 */

#ifndef __BSP_DELAY_H_
#define __BSP_DELAY_H_

#include "main.h"

//DWT高速计数器相关
#define  DWT_CR      *(volatile uint32_t *)0xE0001000
#define  DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define  DEM_CR      *(volatile uint32_t *)0xE000EDFC
#define  DEM_CR_TRCENA                   (1 << 24)
#define  DWT_CR_CYCCNTENA                (1 <<  0)

#define delayms(msec)         delayus(msec*1000)

//DWT高速计数器延时方案---------//
void dwt_delay_init(void);
void dwt_delay_us(uint32_t us);
//TIM外部定时器方案-------------//
void tim_delay_us(TIM_HandleTypeDef* HTIM,uint32_t Delay_us);
//------------------------------//
#endif

/********************************End of Head************************************/

