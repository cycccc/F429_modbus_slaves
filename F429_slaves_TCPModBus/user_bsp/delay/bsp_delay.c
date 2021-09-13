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
 * 文件名称: bsp_delay.c
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

#include "./bsp_delay.h"
#include "../typedef/bsp_typedef.h"

//DWT高速计数器延时方案---------//
/**
 * dwt_delay_init
 * @brief  DWT高数计数器初始化，只开启了计数
 * @param
 *   @arg   None
 * @data    2021-04-17
 * @retval  None
 */
void dwt_delay_init(void) {
    DEM_CR         |=  DEM_CR_TRCENA;
    DWT_CYCCNT      = 0u;
    DWT_CR         |= DWT_CR_CYCCNTENA;
}

/**
 * dwt_delay_us
 * @brief   以1us为单位，经行延时
 * @param
 *   @arg   usec: [输入] 需要延时的秒数
 * @data    2021-04-17
 * @retval  None
 */
void dwt_delay_us(uint32_t usec) {
    uint32_t startts, endts, ts;
    startts = DWT_CYCCNT;

    ts =  usec * (HAL_RCC_GetHCLKFreq() / (1000 * 1000));
    endts = startts + ts;
    if(endts > startts) {
        while(DWT_CYCCNT < endts);
    } else {
        while(DWT_CYCCNT > endts);
        while(DWT_CYCCNT < endts);
    }
}
//------------------------------//

//TIM外部定时器方案-------------//
/**
 * HAL_Delay_us
 * @brief
 * @param
 *   @arg   at24cxx_x :[输入] 芯片数据定义句柄
 *   @arg   Delay_us: [输入] 需要延时的时间
 * @data    2021-04-17
 * @retval  None
 */
void tim_delay_us(TIM_HandleTypeDef* HTIM, uint32_t Delay_us) {
    __HAL_TIM_SET_COUNTER(HTIM, 0);           //将CNT值设置为0

    HAL_TIM_Base_Start(HTIM);                 //启动定时器
    while(__HAL_TIM_GET_COUNTER(HTIM) < Delay_us); //查询计数器的计数值判断
    HAL_TIM_Base_Stop(HTIM);//频繁开启时，可以注释掉这句
}
//------------------------------//

/********************************End of File************************************/

