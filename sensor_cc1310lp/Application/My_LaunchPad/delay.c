/*
 * delay.c
 *
 *  Created on: 2017��9��29��
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

uint8_t delayTimeoutFlag = 0;   // ��ʱʱ�䵽���־
Types_FreqHz freq;  // ����ϵͳ����Ƶ��
GPTimerCC26XX_Value usloadValue;
GPTimerCC26XX_Value msloadValue;

/**
 * ��ʱ���ص�������ʱ��ɱ�־
 */
//void delayTimerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
//{
//    delayTimeoutFlag = 1;
//}

/**
 * us��ʱ��ʹ�ÿ�ѭ��ʵ��
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
 * us��ʱ��ʹ�� ��ʱ��ʵ�֣�������
 * ����ʱ���д�Լ 10us ����ʱʱ�� <100us ����ʹ�� Delay_shortUS()
 */
//void Delay_us(uint32_t usDelay)
//{
//    GPTimerCC26XX_setLoadValue(delayTimerHandle, usloadValue*usDelay);    // ���ö�ʱ��װ��ֵ
//    GPTimerCC26XX_start(delayTimerHandle);
//    while(!delayTimeoutFlag);   // �ȴ���ʱʱ�䵽��
//    delayTimeoutFlag = 0;
//}

/**
 * ms��ʱ��ʹ�� ��ʱ��ʵ�֣�������
 */
//void Delay_ms(uint32_t mSDelay)
//{
//    GPTimerCC26XX_setLoadValue(delayTimerHandle, msloadValue*mSDelay);    // ���ö�ʱ��װ��ֵ
//    GPTimerCC26XX_start(delayTimerHandle);
//    while(!delayTimeoutFlag);   // �ȴ���ʱʱ�䵽��
//    delayTimeoutFlag = 0;
//}
/*
 * ��ʱ��ʼ�������� GPTIMER
 */
//void Delay_init(void)
//{
//    GPTimerCC26XX_Params_init(&delayTimerParams);
//    delayTimerParams.mode = GPT_MODE_ONESHOT_UP;    // �������ϼ���ģʽ
//    delayTimerParams.width = GPT_CONFIG_32BIT;  // 32Bit
//    delayTimerHandle = GPTimerCC26XX_open(Board_GPTIMER0A, &delayTimerParams);
//    GPTimerCC26XX_registerInterrupt(delayTimerHandle, delayTimerCallback, GPT_INT_TIMEOUT); // ע���жϣ����ڷ��ض�ʱ��ɱ�־
//    BIOS_getCpuFreq(&freq); // ��ȡϵͳ����Ƶ�ʣ����ڼ��㶨ʱʱ��
//    usloadValue = freq.lo / 1000000 - 1;  // 47 , 1us
//    msloadValue = freq.lo / 1000 - 1;  // 4799 , 1ms
//}

