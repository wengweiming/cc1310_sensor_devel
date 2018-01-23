/*
 * sensor_mhz14a.c
 *
 *  Created on: 2017年10月9日
 *      Author: jgh-ninghy
 */

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <xdc/runtime/Types.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/sysbios/BIOS.h>

#include "sensor_mhz14a.h"
#include "Board.h"

// 初始设置为上升沿触发
static PIN_Config mhz14aInputPinTable[] =
{
    Board_MHZ14A | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,
    PIN_TERMINATE
};
// PIN 配置参数
PIN_State mhz14aPinState;
PIN_Handle mhz14aPinHandle;
// GPTimer 配置参数
GPTimerCC26XX_Params dutyTimerParams;
GPTimerCC26XX_Handle dutyTimerHandle;

// 私有变量
uint8_t dutyMeasureFirst = 0;   // 第一次测量标志
uint8_t dutyMeasureEnd = 0;    // 测量结束标志
uint32_t timerBuf[2];       // 分别记录 上升沿 和 下降沿 中断时 定时器 的值
uint8_t timerBufIndex = 0;  // 记录次数标志
uint32_t dutyCycle;     // 存储高电平时间

/**
 * 定时器捕获方式回调函数
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
 * IO 中断方式回调函数
 */
void mhz14aPinCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    if (dutyMeasureFirst) {
        // 上升沿中断
        GPTimerCC26XX_start(dutyTimerHandle);   // 启动定时器
        timerBuf[timerBufIndex] = GPTimerCC26XX_getValue(dutyTimerHandle);  // 获取当前定时器值
        PIN_setInterrupt(handle, Board_MHZ14A | PIN_IRQ_NEGEDGE); // 改为下降沿触发
        dutyMeasureFirst = 0;
        timerBufIndex++;
    } else {
        // 下降沿中断
        GPTimerCC26XX_stop(dutyTimerHandle);    // 关闭定时器
        timerBuf[timerBufIndex] = GPTimerCC26XX_getValue(dutyTimerHandle);  // 读取定时器值
        // 计算高电平时间
        // 判断第二次记录值是否在 定时器溢出 之后
        if (timerBuf[timerBufIndex] > timerBuf[timerBufIndex-1]) {
            dutyCycle = (timerBuf[timerBufIndex] - timerBuf[timerBufIndex-1] + 1) / 48000;  // dutyCycle 单位为 ms
        } else {
            dutyCycle = (timerBuf[timerBufIndex] + 47999999 - timerBuf[timerBufIndex-1] + 1) / 48000;
        }
        // 初始化相关参数，等待下一次高电平
        dutyMeasureFirst = 1;
        timerBufIndex = 0;
        dutyMeasureEnd = 1;
        PIN_setInterrupt(handle, Board_MHZ14A | PIN_IRQ_POSEDGE); // 改为上升沿触发
    }
}

/**
 * 二氧化碳传感器 初始化
 * 提供了 IO 中断以及 定时器捕获 两种方式
 * 使用定时器捕获方式最大捕获高电平不超过 349ms，最大测量 698ppm，本次不采用，可做为参考学习
 */
void MHZ14A_init(void)
{
    /* IO 口中断方式 */
    // 配置 PIN
    mhz14aPinHandle = PIN_open(&mhz14aPinState, mhz14aInputPinTable);
    // 注册 IO 口中断回调
    PIN_registerIntCb(mhz14aPinHandle, mhz14aPinCallbackFxn);
    // 配置 GPTimer
    GPTimerCC26XX_Params_init(&dutyTimerParams);
    dutyTimerParams.mode = GPT_MODE_PERIODIC_UP;    // 循环计数
    dutyTimerParams.width = GPT_CONFIG_32BIT;
    dutyTimerHandle = GPTimerCC26XX_open(Board_GPTIMER2A, &dutyTimerParams);
    GPTimerCC26XX_setLoadValue(dutyTimerHandle, 47999999); // 装载值 = 1s，保证每个定时周期 >1s，否则溢出次数超过 1次
    // 置位第一次测量标志
    dutyMeasureFirst = 1;

    /* 定时器捕获方式 （由于该方式定时器为 24bit ，最大测量高电平时间 349ms ，不满足本次用途）*/
    // 设置定时器为 捕获模式
/*    GPTimerCC26XX_Params_init(&dutyTimerParams);
    dutyTimerParams.mode = GPT_MODE_EDGE_TIME_UP;
    dutyTimerParams.width = GPT_CONFIG_16BIT;
    dutyTimerHandle = GPTimerCC26XX_open(Board_GPTIMER2A, &dutyTimerParams);

    // 注册中断
    GPTimerCC26XX_registerInterrupt(dutyTimerHandle, mhz14aCaptureCallbackFxn, GPT_INT_CAPTURE );

    // 使能 PIN
    mhz14aPinHandle = PIN_open(&mhz14aPinState, mhz14aInputPinTable);
    // 复用 PIN
    GPTimerCC26XX_PinMux pinMux = GPTimerCC26XX_getPinMux(dutyTimerHandle);
    PINCC26XX_setMux(mhz14aPinHandle, Board_MHZ14A, pinMux);
    GPTimerCC26XX_setCaptureEdge(dutyTimerHandle, GPTimerCC26XX_POS_EDGE);
    // 设置定时器装载值
    GPTimerCC26XX_setLoadValue(dutyTimerHandle, 0xFFFFFF);

    dutyMeasureFirst = 1;

    GPTimerCC26XX_start(dutyTimerHandle);*/
}

/**
 * 读取传感器数据
 * 由于初始化之后，每个 PWM 周期都会触发中断，得到新的数据，这里获得的是最近一次的数据
 * 待优化：读的时候才打开总段，其他时间关闭中断
 */
uint32_t MHZ14A_read(void)
{
    if (dutyMeasureEnd) {
        dutyMeasureEnd = 0;
        return dutyCycle*2;
    }
    return 0;
}
