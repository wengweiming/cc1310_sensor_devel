/*
 * delay.c
 *
 *  Created on: 2017年9月29日
 *      Author: jgh-ninghy
 */
#include "delay.h"
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Types.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include "Board.h"

GPTimerCC26XX_Params delayTimerParams;
GPTimerCC26XX_Handle delayTimerHandle;

uint8_t delayTimeoutFlag = 0;   // 延时时间到达标志
Types_FreqHz freq;  // 保存系统工作频率
GPTimerCC26XX_Value usloadValue;
GPTimerCC26XX_Value msloadValue;

/**
 * 定时器回调，置延时完成标志
 */
//void delayTimerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
//{
//    delayTimeoutFlag = 1;
//}

/**
 * us延时，使用空循环实现
 */
void Delay_shortUS(uint32_t usDelay)
{
    CPUdelay(8*usDelay);
//    uint32_t i = 0;
//
//    UInt key = Task_disable();
//    Hwi_disable();
//    for (i=0; i<3*usDelay; ++i);
//    Hwi_enable();
//    Task_restore(key);
}

/**
 * us延时，使用 定时器实现，阻塞型
 * 该延时会有大约 10us 误差，延时时间 <100us 建议使用 Delay_shortUS()
 */
//void Delay_us(uint32_t usDelay)
//{
//    GPTimerCC26XX_setLoadValue(delayTimerHandle, usloadValue*usDelay);    // 设置定时器装载值
//    GPTimerCC26XX_start(delayTimerHandle);
//    while(!delayTimeoutFlag);   // 等待定时时间到达
//    delayTimeoutFlag = 0;
//}

/**
 * ms延时，使用 定时器实现，阻塞型
 */
//void Delay_ms(uint32_t mSDelay)
//{
//    GPTimerCC26XX_setLoadValue(delayTimerHandle, msloadValue*mSDelay);    // 设置定时器装载值
//    GPTimerCC26XX_start(delayTimerHandle);
//    while(!delayTimeoutFlag);   // 等待定时时间到达
//    delayTimeoutFlag = 0;
//}
/*
 * 延时初始化，配置 GPTIMER
 */
//void Delay_init(void)
//{
//    GPTimerCC26XX_Params_init(&delayTimerParams);
//    delayTimerParams.mode = GPT_MODE_ONESHOT_UP;    // 单次向上计数模式
//    delayTimerParams.width = GPT_CONFIG_32BIT;  // 32Bit
//    delayTimerHandle = GPTimerCC26XX_open(Board_GPTIMER0A, &delayTimerParams);
//    GPTimerCC26XX_registerInterrupt(delayTimerHandle, delayTimerCallback, GPT_INT_TIMEOUT); // 注册中断，用于返回定时完成标志
//    BIOS_getCpuFreq(&freq); // 获取系统工作频率，用于计算定时时间
//    usloadValue = freq.lo / 1000000 - 1;  // 47 , 1us
//    msloadValue = freq.lo / 1000 - 1;  // 4799 , 1ms
//}

