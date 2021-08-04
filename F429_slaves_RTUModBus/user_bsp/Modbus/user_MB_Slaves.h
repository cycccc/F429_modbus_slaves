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
 * 文件名称: user_MB_Slaves.h
 * 创建人员: CY
 * 创建日期: 2021-08-03
 * 文档描述:
 *
 *----------------------------------版本信息------------------------------------
 * 实验平台
 *          |--
 * 版本代号: V1.0
 * 版本说明:
 *          |-1.0
 * 	         |-2021-08-03
 * 	          |-初始版本
 *------------------------------------------------------------------------------
 */

#ifndef __USER__M_B__SLAVES_H_
#define __USER__M_B__SLAVES_H_


#include "main.h"


//-------------------------
/* 自定义参数宏定义 ----------------------------------------------------------*/
#ifndef BUFF_MAX
    #define BUFF_MAX   1024         //内存池大小 -> 注意：寄存器的大小是线圈大小的8倍
#endif


/* 宏定义 --------------------------------------------------------------------*/
#define MB_SLAVEADDR            0x0001      //本机地址
#define MB_REG_ADDR             0x0000      //寄存器地址
#define MB_ALLSLAVEADDR         0x00FF      //广播地址

#define MB_MSG_IDLE      0x0000    // 空闲状态
#define MB_MSG_RXING     0x0001    // 正在接收数据
#define MB_MSG_COM       0x0002    // 接收完成

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    UART_HandleTypeDef* pRTU_Usart;   //硬件串口设备
    GPIO_TypeDef* pRTU_DE_Port;       //硬件中DE线 端口
    uint16_t RTU_DE_Pin;             //硬件中DE线 引脚
	  uint8_t receiveFlagSts;     //串口接收标志状态
	  uint8_t tmpReceiveBuf;      //串口临时接收缓存
    /* ------------ */
    uint8_t* pRxBuff;		  // 接收内存首地址
    uint16_t RxCount;		  // 接收计数
    uint8_t* pMB_coilMemPool;      //线圈内存池
    uint16_t* PMB_registerMemPool; //寄存器内存池
} MB_RTU_TypeDef;

//外部变量声明
extern MB_RTU_TypeDef RTUhandle;      //初始化RTU句柄

//函数声明
void RTU_Parse_Data(void);  //提取数据帧,进行解析数据帧



#endif

/********************************End of Head************************************/

