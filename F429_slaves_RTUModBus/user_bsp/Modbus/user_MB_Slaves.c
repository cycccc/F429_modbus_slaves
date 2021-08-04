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
 * 文件名称: user_MB_Slaves.c
 * 创建人员: CY
 * 创建日期: 2021-08-03
 * 文档描述: //模拟写一个modbus的从站文件
 *
 *----------------------------------版本信息------------------------------------
 * 版本代号: V1.0
 * 版本说明:
 *          |-1.0
 * 	         |-2021-08-03
 * 	          |-初始版本
 *------------------------------------------------------------------------------
 */


#include "user_MB_Slaves.h"
#include "usart.h"

//CRC---------------------
// CRC 高位字节值表
static const uint8_t auchCRCHi[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
// CRC 低位字节值表
static const uint8_t auchCRCLo[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
//variable--------------
uint8_t Rx_Buf[BUFF_MAX];                      //接收缓存

uint8_t  MB_coilMemoryPool[BUFF_MAX / 8];       //线圈内存池
uint16_t MB_registerMemoryPool[BUFF_MAX];        //寄存器内存池

/* 私有变量 ------------------------------------------------------------------*/
MB_RTU_TypeDef  RTUhandle =       //初始化RTU句柄
{
    .pRTU_Usart = &huart2,
    .pRTU_DE_Port = GPIOB,
    .RTU_DE_Pin = GPIO_PIN_8,
	  .receiveFlagSts = MB_MSG_IDLE,
	  .tmpReceiveBuf = 0,
	
    .pRxBuff = Rx_Buf,
    .RxCount = 0,
    .pMB_coilMemPool = MB_coilMemoryPool,
    .PMB_registerMemPool = MB_registerMemoryPool,
};

//functionDeclaration---
uint16_t RTU_CRC16(uint8_t* _pushMsg, uint8_t _usDataLen);  //CRC校验
void RTU_Exception_RSP(uint8_t _FunCode, uint8_t _ExCode);  //异常响应
void UART_Tx(uint8_t* Tx_Buf, uint8_t Byte_len);   //发送数据
void RTU_RSP_01_02(void);      //读线圈
void RTU_RSP_03_04(void);      //读寄存器
void RTU_RSP_05(void);      //写单个线圈
void RTU_RSP_06(void);      //写单个寄存器
void RTU_RSP_15(void);      //写多个线圈
void RTU_RSP_16(void);      //写多个寄存器

//串口发送函数
void UART_Tx(uint8_t* Tx_Buf, uint8_t Byte_len)
{
    HAL_GPIO_WritePin(RTUhandle.pRTU_DE_Port, RTUhandle.RTU_DE_Pin, GPIO_PIN_SET);   //RS485_TX_MODE;
    HAL_UART_Transmit(RTUhandle.pRTU_Usart, Tx_Buf, Byte_len, 0xffff);                //使用串口发送
    HAL_GPIO_WritePin(RTUhandle.pRTU_DE_Port, RTUhandle.RTU_DE_Pin, GPIO_PIN_RESET); //RS485_RX_MODE;
    RTUhandle.RxCount = 0;  //计数清零
}

// 提取数据帧,进行解析数据帧
void RTU_Parse_Data(void)
{
    //查看从站地址是否为本机地址
    if ((RTUhandle.pRxBuff[0] == MB_SLAVEADDR) || (RTUhandle.pRxBuff[0] == MB_ALLSLAVEADDR))
    {
        //查看CRC校验是否正确
        if (((RTUhandle.pRxBuff[RTUhandle.RxCount - 1] << 8) | RTUhandle.pRxBuff[RTUhandle.RxCount - 2])
                == RTU_CRC16(RTUhandle.pRxBuff, RTUhandle.RxCount - 2))
        {
            switch (RTUhandle.pRxBuff[1])
            {
                case 1:
                case 2:
                    RTU_RSP_01_02();
                    break;

                case 3:
                case 4:
                    RTU_RSP_03_04();
                    break;

                case 5:
                    RTU_RSP_05();
                    break;

                case 6:
                    RTU_RSP_06();
                    break;

                case 15:
                    RTU_RSP_15();
                    break;

                case 16:
                    RTU_RSP_16();
                    break;

                default:
                    RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x01);     //功能码错误回应
                    break;
            }
        }
    }

    RTUhandle.RxCount = 0;  //计数清零
    return;
}

/**
  * 函数功能: 读线圈
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void RTU_RSP_01_02(void)
{
    uint16_t P_Addr, P_RegNum, A_Leng, B_Leng;
    uint8_t P_ByteNum;
    uint8_t i, x;
    uint8_t Tx_Buf[35] = {0};
    uint8_t data[35] = {0};
    uint16_t crc = 0;

    P_Addr = ((RTUhandle.pRxBuff[2] << 8) | RTUhandle.pRxBuff[3]);      //寄存器地址
    P_RegNum = ((RTUhandle.pRxBuff[4] << 8) | RTUhandle.pRxBuff[5]);   //寄存器数量

    if (P_Addr < BUFF_MAX)
    {
        if ((P_RegNum + P_Addr <= BUFF_MAX) && P_RegNum <= 30)
        {
            Tx_Buf[0] = RTUhandle.pRxBuff[0];		 /* 从站地址 */
            Tx_Buf[1] = RTUhandle.pRxBuff[1];        /* 功能码   */
            P_ByteNum = P_RegNum / 8;       //字节数

            /* 如果位数还有余数，则字节数+1 || 当起始地址不为0，但是读取位数为0的情况
             * 比如:当起始地址为5，而位数为16时，如果根据P_ByteNum计算字节数时，需要加1，取后一数组的数据填充
             */
            if ((P_RegNum % 8) || ((P_RegNum % 8 == 0) && (P_Addr != 0)))
						{
                P_ByteNum += 1;  //如果位数还有余数，则字节数+1
						}
            Tx_Buf[2] = P_ByteNum;       //返回的字数
            A_Leng = P_Addr / 8; //按照数组位置存储，计算起始位置
            B_Leng = P_Addr % 8; //计算位

            /* 将数据分开放置，数组编号1、2   3、4   5、6 ....两个数组存放同样数据，方便以下计算 */
            for (i = 0; i < P_ByteNum; i++)
            {
                data[2 * i + 1] = RTUhandle.pMB_coilMemPool[A_Leng + i];
                data[2 * i + 2] = RTUhandle.pMB_coilMemPool[A_Leng + i];
            }

            /* 根据字节数赋值 */
            for (x = 0; x < P_ByteNum; x++)
            {
                /* 每次填充一个字节内容数据，但是会存在偏移的问题
                将数组1、3、5...等数组位偏移，也就是两个字节的首位偏移，如果偏移地址为零，则不需要偏移 */
                data[2 * x + 1] = data[2 * x + 1] >> B_Leng;

                /* 首先填充偏移后的数据内容，假设偏移了两位，那么就是还需要填充6位数据内容 */
                for (i = 0; i < 8 - B_Leng; i++)
                {
                    /* 判断偏移后的第一位，也是最低位 */
                    if ((data[2 * x + 1]) & 0x01)
										{
                        Tx_Buf[3 + x] |= (1 << i);
										}
                    /* 偏移一位后for循环继续判断 */
                    data[2 * x + 1] >>= 1;
                }

                /* 由于存在偏移，那么需要取高一位的数组数据内容填充上一个偏移后的数据内容，总共8位一字节 */
                for (i = 0; i < B_Leng; i++)
                {
                    /* 判断高一位数据，我们假设前面取data[1]，这里则取data[4]，data[6]，因为下一轮字节循环data[2*x+1]的值是data[3]，data[5] ...
                    		这样在判断赋值移位时，不影响原来的数据内容*/
                    if ((data[2 * x + 4]) & 0x01)
										{
                        Tx_Buf[3 + x] |= (1 << (8 + i - B_Leng));
										}
                    data[2 * x + 4] >>= 1;
                }
            }

            P_ByteNum += 3;
            crc = RTU_CRC16(Tx_Buf, P_ByteNum);
            Tx_Buf[P_ByteNum++] = (uint8_t)crc;	                  /* crc 低字节 */
            Tx_Buf[P_ByteNum++] = (uint8_t)(crc >> 8);		      /* crc 高字节 */
            UART_Tx(Tx_Buf, P_ByteNum);
        }
        else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x03);     //数值错误回应
    }
    else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x02);     //地址错误回应
}
/**
  * 函数功能: 读寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void RTU_RSP_03_04(void)
{
    uint16_t P_Addr, P_RegNum;
    uint8_t Tx_Buf[35] = { 0 };
    uint8_t P_ByteNum;
    uint16_t crc = 0;

    P_Addr = ((RTUhandle.pRxBuff[2] << 8) | RTUhandle.pRxBuff[3]);      //寄存器地址
    P_RegNum = ((RTUhandle.pRxBuff[4] << 8) | RTUhandle.pRxBuff[5]);   //寄存器数量

    if (P_Addr < BUFF_MAX)
    {
        if((P_RegNum + P_Addr <= BUFF_MAX) && P_RegNum <= 30)
        {
            Tx_Buf[0] = RTUhandle.pRxBuff[0];		 /* 从站地址 */
            Tx_Buf[1] = RTUhandle.pRxBuff[1];        /* 功能码   */
            P_ByteNum = P_RegNum * 2;         //字节数
            Tx_Buf[2] = P_ByteNum;          //返回的字节数

            for (uint16_t i = 0; i < P_RegNum; i++)
            {

                Tx_Buf[3 + i * 2] = (uint8_t)RTUhandle.PMB_registerMemPool[P_Addr + i];         //低字节
                Tx_Buf[4 + i * 2] = (uint8_t)(RTUhandle.PMB_registerMemPool[P_Addr + i] >> 8);  //高字节
            }

            P_ByteNum += 3;
            crc = RTU_CRC16(Tx_Buf, P_ByteNum);
            Tx_Buf[P_ByteNum++] = (uint8_t)crc;	                  /* crc 低字节 */
            Tx_Buf[P_ByteNum++] = (uint8_t)(crc >> 8);		      /* crc 高字节 */
            UART_Tx(Tx_Buf, P_ByteNum);
        }
        else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x03);     //数值错误回应
    }
    else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x02);     //地址错误回应
}

/**
  * 函数功能: 写单个线圈
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void RTU_RSP_05(void)
{
    uint16_t P_Addr, A_Leng, B_Leng;
    uint8_t Tx_Buf[10] = { 0 };

    P_Addr = ((RTUhandle.pRxBuff[2] << 8) | RTUhandle.pRxBuff[3]);      //寄存器地址

    if (P_Addr < BUFF_MAX)
    {
        if (((RTUhandle.pRxBuff[4] == 0xff) || (RTUhandle.pRxBuff[4] == 0x00)) && (RTUhandle.pRxBuff[5] == 0x00) && (RTUhandle.RxCount == 8))
        {
            A_Leng = P_Addr / 8; //按照数组位置存储，计算起始位置
            B_Leng = P_Addr % 8; //计算位
            if (RTUhandle.pRxBuff[4] == 0xff)  //设置线圈
						{
                /* 直接将值赋予相应的地址中 */
                RTUhandle.pMB_coilMemPool[A_Leng] |= 1 << B_Leng;
						}
            else
						{
                /* 直接将值赋予相应的地址中 */
                RTUhandle.pMB_coilMemPool[A_Leng] &= ~(1 << B_Leng);
						}
            Tx_Buf[0] = RTUhandle.pRxBuff[0];		 /* 从站地址 */
            Tx_Buf[1] = RTUhandle.pRxBuff[1];       /* 功能码   */
            Tx_Buf[2] = RTUhandle.pRxBuff[2];
            Tx_Buf[3] = RTUhandle.pRxBuff[3];
            Tx_Buf[4] = RTUhandle.pRxBuff[4];
            Tx_Buf[5] = RTUhandle.pRxBuff[5];
            Tx_Buf[6] = RTUhandle.pRxBuff[6];       /* crc 低字节 */
            Tx_Buf[7] = RTUhandle.pRxBuff[7];       /* crc 高字节 */
            UART_Tx(Tx_Buf, 8);
        }
        else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x03);     //数值错误回应
    }
    else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x02);     //地址错误回应
}

/**
  * 函数功能: 写单个寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void RTU_RSP_06(void)
{
    uint16_t P_Addr;
    uint8_t Tx_Buf[10] = { 0 };

    P_Addr = ((RTUhandle.pRxBuff[2] << 8) | RTUhandle.pRxBuff[3]);      //寄存器地址

    if (P_Addr < BUFF_MAX)
    {
        if(RTUhandle.RxCount == 8)
        {
            RTUhandle.PMB_registerMemPool[P_Addr]  = Rx_Buf[4];             //数据写入
            RTUhandle.PMB_registerMemPool[P_Addr] |= Rx_Buf[5] << 8;
            Tx_Buf[0] = RTUhandle.pRxBuff[0];		 /* 从站地址 */
            Tx_Buf[1] = RTUhandle.pRxBuff[1];       /* 功能码   */
            Tx_Buf[2] = RTUhandle.pRxBuff[2];
            Tx_Buf[3] = RTUhandle.pRxBuff[3];
            Tx_Buf[4] = RTUhandle.pRxBuff[4];
            Tx_Buf[5] = RTUhandle.pRxBuff[5];
            Tx_Buf[6] = RTUhandle.pRxBuff[6];       /* crc 低字节 */
            Tx_Buf[7] = RTUhandle.pRxBuff[7];       /* crc 高字节 */
            UART_Tx(Tx_Buf, 8);
        }
        else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x03);     //数值错误回应
    }
    else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x02);     //地址错误回应
}

/**
  * 函数功能: 写多个线圈
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void RTU_RSP_15(void)
{
    uint16_t P_Addr, P_RegNum, A_Leng, B_Leng;
    uint8_t Tx_Buf[10] = { 0 };
    uint8_t i, x;
    uint8_t data[35] = { 0 };
    uint16_t crc = 0;

    P_Addr = ((RTUhandle.pRxBuff[2] << 8) | RTUhandle.pRxBuff[3]);      //寄存器地址
    P_RegNum = ((RTUhandle.pRxBuff[4] << 8) | RTUhandle.pRxBuff[5]);   //寄存器数量

    if (P_Addr < BUFF_MAX)
    {
        if (((P_RegNum + P_Addr <= BUFF_MAX) && P_RegNum <= 30) && (RTUhandle.pRxBuff[6] == RTUhandle.RxCount - 9))
        {
            A_Leng = P_Addr / 8; //按照数组位置存储，计算起始位置
            B_Leng = P_Addr % 8; //计算位

            for (x = 0; x < Rx_Buf[6]; x++)  //根据字节数写入线圈
            {
                data[x] = Rx_Buf[7 + x];

                for (i = 0; i < (8 - B_Leng); i++)//设置线圈（首地址开始）
                {
                    if ((Rx_Buf[7 + x]) & 0x01)  //判断位是否有效
										{
                        RTUhandle.pMB_coilMemPool[A_Leng + x] |= (1 << (i % 8)) << B_Leng; //从偏移地址开始赋值，偏移地址为B_Leng
										}
                    else
										{
                        RTUhandle.pMB_coilMemPool[A_Leng + x] &= ~((1 << (i % 8)) << B_Leng);
										}
                    Rx_Buf[7 + x] >>= 1;				//继续判断下一位
                }

                for (i = 0; i < B_Leng; i++)//设置线圈（剩余位设置）
                {
                    if ((data[x] >> (8 - B_Leng)) & 0x01)  //根据前面判断剩下的数据继续处理
										{
                        RTUhandle.pMB_coilMemPool[A_Leng + x + 1] |= 1 << (i % 8);  //剩余位赋值置位
										}
                    else
										{
                        RTUhandle.pMB_coilMemPool[A_Leng + x + 1] &= ~(1 << (i % 8));  //剩余位赋值置位
										}
                    data[x] >>= 1;
                }
            }
            Tx_Buf[0] = RTUhandle.pRxBuff[0];		 /* 从站地址 */
            Tx_Buf[1] = RTUhandle.pRxBuff[1];        /* 功能码   */
            Tx_Buf[2] = RTUhandle.pRxBuff[2];        /* 功能码   */
            Tx_Buf[3] = RTUhandle.pRxBuff[3];        /* 功能码   */
            Tx_Buf[4] = RTUhandle.pRxBuff[4];        /* 功能码   */
            Tx_Buf[5] = RTUhandle.pRxBuff[5];        /* 功能码   */
            crc = RTU_CRC16(Tx_Buf, 6);
            Tx_Buf[6] = (uint8_t)crc;	                  /* crc 低字节 */
            Tx_Buf[7] = (uint8_t)(crc >> 8);		      /* crc 高字节 */
            UART_Tx(Tx_Buf, 8);
        }
        else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x03);     //数值错误回应
    }
    else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x02);     //地址错误回应
}

/**
  * 函数功能: 写多个寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void RTU_RSP_16(void)
{
    uint16_t P_Addr, P_RegNum;
    uint8_t Tx_Buf[10] = { 0 };
    uint16_t crc = 0;

    P_Addr = ((RTUhandle.pRxBuff[2] << 8) | RTUhandle.pRxBuff[3]);      //寄存器地址
    P_RegNum = ((RTUhandle.pRxBuff[4] << 8) | RTUhandle.pRxBuff[5]);   //寄存器数量

    if (P_Addr < BUFF_MAX)
    {
        if (((P_RegNum + P_Addr <= BUFF_MAX) && P_RegNum <= 30) && (RTUhandle.pRxBuff[6] == RTUhandle.RxCount - 9))
        {
            for (int i = 0; i < P_RegNum; i++)
            {
                RTUhandle.PMB_registerMemPool[P_Addr + i] = Rx_Buf[7 + i * 2];      //低字节
                RTUhandle.PMB_registerMemPool[P_Addr + i] |= Rx_Buf[8 + i * 2] << 8;  //高字节
            }

            Tx_Buf[0] = RTUhandle.pRxBuff[0];		 /* 从站地址 */
            Tx_Buf[1] = RTUhandle.pRxBuff[1];        /* 功能码   */
            Tx_Buf[2] = RTUhandle.pRxBuff[2];        /* 地址高位   */
            Tx_Buf[3] = RTUhandle.pRxBuff[3];        /* 地址低位   */
            Tx_Buf[4] = RTUhandle.pRxBuff[4];        /* 数量高位   */
            Tx_Buf[5] = RTUhandle.pRxBuff[5];        /* 数量低位   */
            crc = RTU_CRC16(Tx_Buf, 6);
            Tx_Buf[6] = (uint8_t)crc;	                  /* crc 低字节 */
            Tx_Buf[7] = (uint8_t)(crc >> 8);		      /* crc 高字节 */
            UART_Tx(Tx_Buf, 8);
        }
        else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x03);     //数值错误回应
    }
    else RTU_Exception_RSP(RTUhandle.pRxBuff[1], 0x02);     //地址错误回应
}

/**
  * 函数功能: Modbus CRC16 校验计算函数
  * 输入参数: pushMsg:待计算的数据首地址,usDataLen:数据长度
  * 返 回 值: CRC16 计算结果
  * 说    明: 计算结果是高位在前,需要转换才能发送
  */
uint16_t RTU_CRC16(uint8_t* _pushMsg, uint8_t _usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint16_t uIndex;

    while (_usDataLen--)
    {
        uIndex = uchCRCLo ^ *_pushMsg++;
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }

    return (uchCRCHi << 8 | uchCRCLo);
}

/**
  * 函数功能: 异常响应
  * 输入参数: _FunCode :发送异常的功能码,_ExCode:异常码
  * 返 回 值: 无
  * 说    明: 当通信数据帧发生异常时,发送异常响应
  */
void RTU_Exception_RSP(uint8_t _FunCode, uint8_t _ExCode)
{
    uint8_t Tx_Buf[7];
    uint8_t TxCount = 0;
    uint16_t crc = 0;

    Tx_Buf[TxCount++] = MB_SLAVEADDR;		    /* 从站地址 */
    Tx_Buf[TxCount++] = _FunCode | 0x80;		  /* 功能码 + 0x80*/
    Tx_Buf[TxCount++] = _ExCode;	          /* 异常码*/
    crc = RTU_CRC16(Tx_Buf, TxCount);
    Tx_Buf[TxCount++] = (uint8_t)crc;	          /* crc 低字节 */
    Tx_Buf[TxCount++] = (uint8_t)(crc >> 8);		      /* crc 高字节 */
    UART_Tx(Tx_Buf, TxCount);
}

/**********************************************************************************************//**
 * @fn	void RTU_IRQCallback(void)
 *
 * @brief  串口接收中断，因为是不定长的接收中断，所以不能笼统的写在接收中断的回调里面
 *
 * @return	无 N/A
 **************************************************************************************************/
void RTU_IRQCallback(void)
{
	    if(__HAL_UART_GET_FLAG(RTUhandle.pRTU_Usart, UART_FLAG_RXNE) != RESET)
    {
        HAL_UART_IRQHandler(&huart2);
        if(RTUhandle.receiveFlagSts == MB_MSG_IDLE)RTUhandle.receiveFlagSts = MB_MSG_RXING;
			  RTUhandle.pRxBuff[RTUhandle.RxCount++] = RTUhandle.tmpReceiveBuf;
			  RTUhandle.tmpReceiveBuf = 0;
    }

    if(__HAL_UART_GET_FLAG(RTUhandle.pRTU_Usart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(RTUhandle.pRTU_Usart);        //清除串口空闲中断标志位
        RTUhandle.receiveFlagSts = MB_MSG_COM;
    }

    HAL_UART_Receive_IT(RTUhandle.pRTU_Usart, &RTUhandle.tmpReceiveBuf, 1);
}



/********************************End of File************************************/

