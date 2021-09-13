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
 * 文件名称: bsp_at24cxx.h
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

#ifndef __BSP_AT24CXX_H_
#define __BSP_AT24CXX_H_

#include "main.h"

#include "../typedef/bsp_typedef.h"

//EEPROM 中各型号的内存大小
typedef enum {
    AT24C01  = 127,
    AT24C02  = 255,
    AT24C04  = 511,
    AT24C08  = 1023,
    AT24C16  = 2047,
    AT24C32  = 4095,
    AT24C64  = 8191,
    AT24C128 = 16383,
    AT24C256 = 32767,
} AT24CXX_TypeAndSize;
/**
 *是否使用软件IIC
 * 0：使用软件IIC
 * 1：不使用软件IIC，使用硬件IIC
 */
#define SOFTWARE_IIC  0
/*等待超时时间*/
#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
#define I2Cx_TIMEOUT_MAX                300
/* Maximum number of trials for HAL_I2C_IsDeviceReady() function */
#define EEPROM_MAX_TRIALS               300

#if (!SOFTWARE_IIC)       ///>使用硬件IIC
    #include "i2c.h"
#else                     ///>使用软件IIC
    //软件IIC使用的宏
    #define I2C_SCL_1(AT24CXX_HT)  HAL_GPIO_WritePin(AT24CXX_HT->IIC_SCL_PORT, AT24CXX_HT->IIC_SCL_PIN,GPIO_PIN_SET)		/* SCL = 1 */
    #define I2C_SCL_0(AT24CXX_HT)  HAL_GPIO_WritePin(AT24CXX_HT->IIC_SCL_PORT, AT24CXX_HT->IIC_SCL_PIN,GPIO_PIN_RESET)		/* SCL = 0 */

    #define I2C_SDA_1(AT24CXX_HT)  HAL_GPIO_WritePin(AT24CXX_HT->IIC_SDA_PORT, AT24CXX_HT->IIC_SDA_PIN,GPIO_PIN_SET)		/* SDA = 1 */
    #define I2C_SDA_0(AT24CXX_HT)  HAL_GPIO_WritePin(AT24CXX_HT->IIC_SDA_PORT, AT24CXX_HT->IIC_SDA_PIN,GPIO_PIN_RESET)		/* SDA = 0 */

    #define I2C_SDA_READ(AT24CXX_HT)  HAL_GPIO_ReadPin(AT24CXX_HT->IIC_SDA_PORT, AT24CXX_HT->IIC_SDA_PIN)	/* 读SDA口线状态 */
#endif

//定义EEPROM的类型
typedef struct {
    u8 EEPROM_ID;                ///>EEPROMID编号
    u8 ready;                    ///>EEPROM状态
    AT24CXX_TypeAndSize Type;    ///>芯片类型
    u8 address;                  ///>器件片选地址
    I2C_HandleTypeDef* HIIC;     ///>IIC总线号
} AT24CXX_HandleTypedef;


//函数声明
void I2C_EE_Init(AT24CXX_HandleTypedef* at24cxx_x);
uint32_t I2C_EE_BufferWrite(AT24CXX_HandleTypedef* at24cxx_x, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
uint32_t I2C_EE_BufferRead(AT24CXX_HandleTypedef* at24cxx_x, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
#endif

/********************************End of Head************************************/

