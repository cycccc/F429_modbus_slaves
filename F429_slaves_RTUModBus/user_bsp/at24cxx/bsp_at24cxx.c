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
 * 文件名称: bsp_at24cxx.c
 * 创建人员: CY
 * 创建日期: 2021-04-17
 * 文档描述:
 *
 *----------------------------------版本信息------------------------------------
 * 版本代号: V1.0
 * 版本说明:
 *          |-1.0
 * 	         |-2021-04-17
 * 	          |-初始版本
 *------------------------------------------------------------------------------
 */

#include "./bsp_at24cxx.h"

AT24CXX_HandleTypedef at24cxx_1 = {
    1,
    0,
    AT24C256,
    1 << 7 | 1 << 5,
      &hi2c1,
};


#if (!SOFTWARE_IIC)       ///>使用硬件IIC
/**
  * @brief  I2C 外设(EEPROM)初始化
  * @param
  *   @arg at24cxx_x :[输入] 芯片数据定义句柄
  * @retval 无
  */
void I2C_EE_Init(AT24CXX_HandleTypedef* at24cxx_x) {
    at24cxx_x->ready = 1;
}

/**
  * @brief   在EEPROM的一个写循环中可以写多个字节，但一次写入的字节数
  *          不能超过EEPROM页的大小，AT24C02每页有8个字节
  * @param
  *   @arg at24cxx_x :[输入] 芯片数据定义句柄
  *		@arg pBuffer:缓冲区指针
  *		@arg WriteAddr:写地址
  *   @arg NumByteToWrite:写的字节数
  * @retval  写入的状态
  */
uint32_t I2C_EE_PageWrite(AT24CXX_HandleTypedef* at24cxx_x, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite) {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t MemAdd_size;
    if(at24cxx_x->ready) {
        switch(at24cxx_x->Type) {
            case AT24C128:
            case AT24C256:
                MemAdd_size = I2C_MEMADD_SIZE_16BIT;
                break;
            default:
                MemAdd_size = I2C_MEMADD_SIZE_8BIT;
                break;
        }

        /* Write EEPROM_PAGESIZE */
        status = HAL_I2C_Mem_Write(at24cxx_x->HIIC, at24cxx_x->address, WriteAddr, MemAdd_size, (uint8_t*)(pBuffer), NumByteToWrite, I2CT_FLAG_TIMEOUT);

        while (HAL_I2C_GetState(at24cxx_x->HIIC) != HAL_I2C_STATE_READY) {
        }

        /* Check if the EEPROM is ready for a new operation */
        while (HAL_I2C_IsDeviceReady(at24cxx_x->HIIC, at24cxx_x->address, EEPROM_MAX_TRIALS, I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT);

        /* Wait for the end of the transfer */
        while (HAL_I2C_GetState(at24cxx_x->HIIC) != HAL_I2C_STATE_READY) {
        }
    }
    return status;
}



/**
  * @brief   将缓冲区中的数据写到I2C EEPROM中
  * @param
  *   @arg at24cxx_x :[输入] 芯片数据定义句柄
  *		@arg pBuffer:缓冲区指针
  *		@arg WriteAddr:写地址
  *   @arg NumByteToWrite:写的字节数
  * @retval  写入的状态
  */
uint32_t I2C_EE_BufferWrite(AT24CXX_HandleTypedef* at24cxx_x, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite) {
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, EEpromPageSize = 0;
    uint32_t status = HAL_ERROR;

    if(at24cxx_x->ready) {
        switch(at24cxx_x->Type) {
            case AT24C01:  /* AT24C01/02每页有8个字节 */
            case AT24C02:
                EEpromPageSize = 8;
                break;
            case AT24C04:  /* AT24C04/08/16每页有16个字节 */
            case AT24C08:
            case AT24C16:
                EEpromPageSize = 16;
                break;
            case AT24C32:  /* AT24C32/64每页有32个字节 */
            case AT24C64:
                EEpromPageSize = 16;
                break;
            case AT24C128: /* AT24C128/256每页有64个字节 */
            case AT24C256:
                EEpromPageSize = 16;
                break;
            default:       /* 其余没有的 默认每页有8个字节 */
                EEpromPageSize = 8;
                break;
        }

        Addr = WriteAddr % EEpromPageSize;
        count = EEpromPageSize - Addr;
        NumOfPage =  NumByteToWrite / EEpromPageSize;
        NumOfSingle = NumByteToWrite % EEpromPageSize;

        /* If WriteAddr is I2C_PageSize aligned  */
        if(Addr == 0) {
            /* If NumByteToWrite < I2C_PageSize */
            if(NumOfPage == 0) {
                status = I2C_EE_PageWrite(at24cxx_x, pBuffer, WriteAddr, NumOfSingle);
            }
            /* If NumByteToWrite > I2C_PageSize */
            else {
                while(NumOfPage--) {
                    status = I2C_EE_PageWrite(at24cxx_x, pBuffer, WriteAddr, EEpromPageSize);
                    WriteAddr +=  EEpromPageSize;
                    pBuffer += EEpromPageSize;
                }

                if(NumOfSingle != 0) {
                    status = I2C_EE_PageWrite(at24cxx_x, pBuffer, WriteAddr, NumOfSingle);
                }
            }
        }
        /* If WriteAddr is not I2C_PageSize aligned  */
        else {
            /* If NumByteToWrite < I2C_PageSize */
            if(NumOfPage == 0) {
                status = I2C_EE_PageWrite(at24cxx_x, pBuffer, WriteAddr, NumOfSingle);
            }
            /* If NumByteToWrite > I2C_PageSize */
            else {
                NumByteToWrite -= count;
                NumOfPage =  NumByteToWrite / EEpromPageSize;
                NumOfSingle = NumByteToWrite % EEpromPageSize;

                if(count != 0) {
                    status = I2C_EE_PageWrite(at24cxx_x, pBuffer, WriteAddr, count);
                    WriteAddr += count;
                    pBuffer += count;
                }

                while(NumOfPage--) {
                    status = I2C_EE_PageWrite(at24cxx_x, pBuffer, WriteAddr, EEpromPageSize);
                    WriteAddr +=  EEpromPageSize;
                    pBuffer += EEpromPageSize;
                }
                if(NumOfSingle != 0) {
                    status = I2C_EE_PageWrite(at24cxx_x, pBuffer, WriteAddr, NumOfSingle);
                }
            }
        }
    }
    return status;
}

/**
  * @brief   从EEPROM里面读取一块数据
  * @param
  *   @arg at24cxx_x :[输入] 芯片数据定义句柄
  *		@arg pBuffer:存放从EEPROM读取的数据的缓冲区指针
  *		@arg WriteAddr:接收数据的EEPROM的地址
  *     @arg NumByteToWrite:要从EEPROM读取的字节数
  * @retval  读取的状态
  */
uint32_t I2C_EE_BufferRead(AT24CXX_HandleTypedef* at24cxx_x, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead) {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t MemAdd_size;

    if(at24cxx_x->ready) {
        switch(at24cxx_x->Type) {
            case AT24C128:
            case AT24C256:
                MemAdd_size = I2C_MEMADD_SIZE_16BIT;
                break;
            default:
                MemAdd_size = I2C_MEMADD_SIZE_8BIT;
                break;
        }
        status = HAL_I2C_Mem_Read(at24cxx_x->HIIC, at24cxx_x->address | 1, ReadAddr, MemAdd_size, (uint8_t *)pBuffer, NumByteToRead, I2CT_FLAG_TIMEOUT);
    }
    return status;
}
#else      ///>使用软件IIC

#endif
/********************************End of File************************************/

