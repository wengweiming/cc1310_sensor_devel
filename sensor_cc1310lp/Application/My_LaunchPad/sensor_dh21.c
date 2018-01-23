/*
 * sensor_dh21.c
 *
 *  Created on: 2017年9月28日
 *      Author: jgh-ninghy
 */

#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include "board.h"
#include "sensor_dh21.h"
#include "delay.h"

// 定义数据线的操作
#define DH21_ONEWIRE_WRITE_HIGH     (PIN_setOutputValue(oneWirePinHandle, Board_DHT21, 1))
#define DH21_ONEWIRE_WRITE_LOW      (PIN_setOutputValue(oneWirePinHandle, Board_DHT21, 0))
#define DH21_ONEWIRE_READ           (PIN_getInputValue(Board_DHT21))

// 输出模式配置
static PIN_Config oneWireOutputPinTable[] =
{
    Board_DHT21 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX | PIN_INPUT_DIS,
    PIN_TERMINATE
};

// 输入模式配置
static PIN_Config oneWireInputPinTable[] =
{
    Board_DHT21 | PIN_INPUT_EN | PIN_PULLUP | PIN_GPIO_OUTPUT_DIS ,
    PIN_TERMINATE
};

static PIN_State oneWirePinState;
static PIN_Handle oneWirePinHandle;

void DH21_init(void)
{
    oneWirePinHandle = PIN_open(&oneWirePinState, oneWireOutputPinTable);
}

/**
 * 主机发送开始信号
 */
uint8_t DH21_start(void)
{
    PIN_setConfig(oneWirePinHandle, PIN_BM_ALL, *oneWireOutputPinTable);    // 配置成输出模式
    DH21_ONEWIRE_WRITE_HIGH;    // output High
    DH21_ONEWIRE_WRITE_LOW;     // 拉低总线
    Delay_shortUS(1000);      // >500us
    PIN_setConfig(oneWirePinHandle, PIN_BM_ALL, *oneWireInputPinTable); // 配置成输入模式
    Delay_shortUS(50);        // 20-40us
    // 判断 50us 之后 DH21 是否有响应
    if (!DH21_ONEWIRE_READ) {
        return 1;
    }
    return 0;
}

/**
 * read byte from DH21
 */
uint8_t DH21_readByte(void)
{
    uint8_t i;
    uint8_t dat;

    for (i=0; i<8; ++i) {
        dat <<= 1;
        while(DH21_ONEWIRE_READ);   // 清除上一个数据电平
        while(!DH21_ONEWIRE_READ);  // 1Bit 开始信号 50us
        Delay_shortUS(40);    // 26 - 28us : 表示 0
                         // 70us : 表示 1
        // 等待 40us
        if (DH21_ONEWIRE_READ) {
            dat |= 1;
        } else {
            dat |= 0;
        }
    }
    return dat;
}

/**
 * read data from DH21
 */
void DH21_read(DH21_data *dhData)
{
    dhData->readFlag = 0;
    if (DH21_start()) {
        // 启动成功
        while(!DH21_ONEWIRE_READ);     // 等待 DH21 响应信号结束， 80us
        while(DH21_ONEWIRE_READ);      // DH21 拉高总线 80us
        // 进行数据接收
        dhData->humiH = DH21_readByte();
        dhData->humiL = DH21_readByte();
        dhData->tempH = DH21_readByte();
        dhData->tempL = DH21_readByte();
        dhData->check = DH21_readByte();
        // 数据校验
        if (dhData->check == (uint8_t)(dhData->humiH + dhData->humiL +
                                       dhData->tempH + dhData->tempL)) {
            // 校验成功
            dhData->readFlag = 1;
            dhData->humiValue = (dhData->humiH << 8) + dhData->humiL;
            dhData->tempValue = (dhData->tempH << 8) + dhData->tempL;
        }
    }
}

