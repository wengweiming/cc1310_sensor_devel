/*
 * sensor_mhz14a.c
 *
 *  Created on: 2017��10��9��
 *      Author: jgh-ninghy
 */

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <xdc/runtime/Types.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/sysbios/BIOS.h>

#include "sensor_mhz14a.h"
#include "Board.h"

// ��ʼ����Ϊ�����ش���
static PIN_Config mhz14aInputPinTable[] =
{
    Board_MHZ14A | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,
    PIN_TERMINATE
};
// PIN ���ò���
PIN_State mhz14aPinState;
PIN_Handle mhz14aPinHandle;
// GPTimer ���ò���
GPTimerCC26XX_Params dutyTimerParams;
GPTimerCC26XX_Handle dutyTimerHandle;

// ˽�б���
uint8_t dutyMeasureFirst = 0;   // ��һ�β�����־
uint8_t dutyMeasureEnd = 0;    // ����������־
uint32_t timerBuf[2];       // �ֱ��¼ ������ �� �½��� �ж�ʱ ��ʱ�� ��ֵ
uint8_t timerBufIndex = 0;  // ��¼������־
uint32_t dutyCycle;     // �洢�ߵ�ƽʱ��

/**
 * ��ʱ������ʽ�ص�����
 */
/*
void mhz14aCaptureCallbackFxn(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    timerBuf[timerBufIndex] = GPTimerCC26XX_getValue(handle);

    if (dutyMeasureFirst) {
        dutyMeasureFirst = 0;
        timerBufIndex++;
        GPTimerCC26XX_setCaptureEdge(handle, GPTimerCC26XX_NEG_EDGE);
    } else {
        if (timerBuf[timerBufIndex] > timerBuf[timerBufIndex-1]) {
            dutyCycle = (timerBuf[timerBufIndex] - timerBuf[timerBufIndex-1] + 1) / 48000;
        } else {
            dutyCycle = (timerBuf[timerBufIndex] + 0xFFFFFF - timerBuf[timerBufIndex-1] + 1) / 48000;
        }

        dutyMeasureFirst = 1;
        timerBufIndex = 0;
        dutyMeasureEnd = 1;
        GPTimerCC26XX_setCaptureEdge(handle, GPTimerCC26XX_POS_EDGE);
    }
}*/

/**
 * IO �жϷ�ʽ�ص�����
 */
void mhz14aPinCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    if (dutyMeasureFirst) {
        // �������ж�
        GPTimerCC26XX_start(dutyTimerHandle);   // ������ʱ��
        timerBuf[timerBufIndex] = GPTimerCC26XX_getValue(dutyTimerHandle);  // ��ȡ��ǰ��ʱ��ֵ
        PIN_setInterrupt(handle, Board_MHZ14A | PIN_IRQ_NEGEDGE); // ��Ϊ�½��ش���
        dutyMeasureFirst = 0;
        timerBufIndex++;
    } else {
        // �½����ж�
        GPTimerCC26XX_stop(dutyTimerHandle);    // �رն�ʱ��
        timerBuf[timerBufIndex] = GPTimerCC26XX_getValue(dutyTimerHandle);  // ��ȡ��ʱ��ֵ
        // ����ߵ�ƽʱ��
        // �жϵڶ��μ�¼ֵ�Ƿ��� ��ʱ����� ֮��
        if (timerBuf[timerBufIndex] > timerBuf[timerBufIndex-1]) {
            dutyCycle = (timerBuf[timerBufIndex] - timerBuf[timerBufIndex-1] + 1) / 48000;  // dutyCycle ��λΪ ms
        } else {
            dutyCycle = (timerBuf[timerBufIndex] + 47999999 - timerBuf[timerBufIndex-1] + 1) / 48000;
        }
        // ��ʼ����ز������ȴ���һ�θߵ�ƽ
        dutyMeasureFirst = 1;
        timerBufIndex = 0;
        dutyMeasureEnd = 1;
        PIN_setInterrupt(handle, Board_MHZ14A | PIN_IRQ_POSEDGE); // ��Ϊ�����ش���
    }
}

/**
 * ������̼������ ��ʼ��
 * �ṩ�� IO �ж��Լ� ��ʱ������ ���ַ�ʽ
 * ʹ�ö�ʱ������ʽ��󲶻�ߵ�ƽ������ 349ms�������� 698ppm�����β����ã�����Ϊ�ο�ѧϰ
 */
void MHZ14A_init(void)
{
    /* IO ���жϷ�ʽ */
    // ���� PIN
    mhz14aPinHandle = PIN_open(&mhz14aPinState, mhz14aInputPinTable);
    // ע�� IO ���жϻص�
    PIN_registerIntCb(mhz14aPinHandle, mhz14aPinCallbackFxn);
    // ���� GPTimer
    GPTimerCC26XX_Params_init(&dutyTimerParams);
    dutyTimerParams.mode = GPT_MODE_PERIODIC_UP;    // ѭ������
    dutyTimerParams.width = GPT_CONFIG_32BIT;
    dutyTimerHandle = GPTimerCC26XX_open(Board_GPTIMER2A, &dutyTimerParams);
    GPTimerCC26XX_setLoadValue(dutyTimerHandle, 47999999); // װ��ֵ = 1s����֤ÿ����ʱ���� >1s����������������� 1��
    // ��λ��һ�β�����־
    dutyMeasureFirst = 1;

    /* ��ʱ������ʽ �����ڸ÷�ʽ��ʱ��Ϊ 24bit ���������ߵ�ƽʱ�� 349ms �������㱾����;��*/
    // ���ö�ʱ��Ϊ ����ģʽ
/*    GPTimerCC26XX_Params_init(&dutyTimerParams);
    dutyTimerParams.mode = GPT_MODE_EDGE_TIME_UP;
    dutyTimerParams.width = GPT_CONFIG_16BIT;
    dutyTimerHandle = GPTimerCC26XX_open(Board_GPTIMER2A, &dutyTimerParams);

    // ע���ж�
    GPTimerCC26XX_registerInterrupt(dutyTimerHandle, mhz14aCaptureCallbackFxn, GPT_INT_CAPTURE );

    // ʹ�� PIN
    mhz14aPinHandle = PIN_open(&mhz14aPinState, mhz14aInputPinTable);
    // ���� PIN
    GPTimerCC26XX_PinMux pinMux = GPTimerCC26XX_getPinMux(dutyTimerHandle);
    PINCC26XX_setMux(mhz14aPinHandle, Board_MHZ14A, pinMux);
    GPTimerCC26XX_setCaptureEdge(dutyTimerHandle, GPTimerCC26XX_POS_EDGE);
    // ���ö�ʱ��װ��ֵ
    GPTimerCC26XX_setLoadValue(dutyTimerHandle, 0xFFFFFF);

    dutyMeasureFirst = 1;

    GPTimerCC26XX_start(dutyTimerHandle);*/
}

/**
 * ��ȡ����������
 * ���ڳ�ʼ��֮��ÿ�� PWM ���ڶ��ᴥ���жϣ��õ��µ����ݣ������õ������һ�ε�����
 * ���Ż�������ʱ��Ŵ��ܶΣ�����ʱ��ر��ж�
 */
uint32_t MHZ14A_read(void)
{
    if (dutyMeasureEnd) {
        dutyMeasureEnd = 0;
        return dutyCycle*2;
    }
    return 0;
}
