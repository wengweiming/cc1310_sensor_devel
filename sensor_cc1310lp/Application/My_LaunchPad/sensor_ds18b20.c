/*
 * sensor_ds18b20.c
 *
 *  Created on: 2017年10月11日
 *      Author: jgh-ninghy
 */

#include <ti/drivers/PIN.h>

#include "board.h"
#include "sensor_ds18b20.h"
#include "delay.h"

// 定义数据线的操作
#define DS18B20_WRITE_HIGH     (PIN_setOutputValue(ds18b20PinHandle, Board_DS18B20, 1))
#define DS18B20_WRITE_LOW      (PIN_setOutputValue(ds18b20PinHandle, Board_DS18B20, 0))
#define DS18B20_READ           (PIN_getInputValue(Board_DS18B20))

// 开漏输出模式配置
static PIN_Config ds18b20OutputPinTable[] =
{
    Board_DS18B20 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_OPENDRAIN | PIN_INPUT_EN,
    PIN_TERMINATE
};

static PIN_State ds18b20PinState;
static PIN_Handle ds18b20PinHandle;

void DS18B20_init(void)
{
    // 初始化 PIN
    ds18b20PinHandle = PIN_open(&ds18b20PinState, ds18b20OutputPinTable);
}

/**
 * 写入 1Bit
 */
void DS18B20_writeOneBit(uint8_t dat)
{
    DS18B20_WRITE_LOW;     // 拉低 >1us
    Delay_shortUS(2);
    PIN_setOutputValue(ds18b20PinHandle, Board_DS18B20, dat);   // 写时隙 >60us
    Delay_shortUS(70);
    DS18B20_WRITE_HIGH;    // 释放总线
    Delay_shortUS(2);    // 写入下一位数据的间隔 >1us
}

/**
 * 写入 1Byte
 */
void DS18B20_writeByte(uint8_t dat)
{
    uint8_t i;
    for (i=0; i<8; ++i) {
        DS18B20_writeOneBit(dat&0x01);
        dat >>= 1;
    }
}

/**
 * 读入 1Bit
 */
uint8_t DS18B20_readOneBit(void)
{
    uint8_t dat;

    DS18B20_WRITE_LOW;     // 拉低 >1us
    Delay_shortUS(2);
    DS18B20_WRITE_HIGH;    // 释放总线
    Delay_shortUS(5);
    // 15us 内读数据
    dat = DS18B20_READ;
    Delay_shortUS(70);       // 保证每个 读时隙 >60us
    DS18B20_WRITE_HIGH;    // 释放总线
    Delay_shortUS(2);    // 读取下一位数据的间隔 >1us

    return dat;
}

/**
 * 读取 1Byte
 */
uint8_t DS18B20_readByte(void)
{
    uint8_t dat;
    uint8_t i;

    for (i=0; i<8; ++i) {
        dat >>= 1;
        if (DS18B20_readOneBit()) {
            dat |= 0x80;    // 总线数据是从 低位 开始的
        }
    }

    return dat;
}

uint8_t DS18B20_start(void)
{
    DS18B20_WRITE_HIGH;
    Delay_shortUS(10);
    DS18B20_WRITE_LOW;     // 主机拉低 >480us
    Delay_shortUS(800);
    DS18B20_WRITE_HIGH;    // 释放总线
    Delay_shortUS(80);
    // 等待 >60us，判断从机是否响应
    if (!DS18B20_READ) {
        return 1;
    }
    return 0;
}

/**
 * 读取 DS18B20 数据
 * 该函数要求总线上只有一个 DS18B20 ，若存在多个，则需要写入其他的指令进行读写
 * 本次读取结果为上一次转换结果，因为 12bit 分辨率下，转换时间最大需要 950ms
 */
void DS18B20_read(DS18B20_Data *dsData)
{
    dsData->readFlag = 0;
    if (DS18B20_start()) {
        while (!DS18B20_READ);     // 等待从机响应信号结束
        Delay_shortUS(500);      // 用于保证 Master Rx >480us
        DS18B20_writeByte(DS18B20_SKIP_ROM);    // 总线只有一个 DS18B20 ,跳过搜索的过程
        DS18B20_writeByte(DS18B20_CONVERT_T);    // 启动转换
        if (DS18B20_start()) {
            while (!DS18B20_READ);     // 等待从机响应信号结束
            Delay_shortUS(300);      // 用于保证 Master Rx >480us
            DS18B20_writeByte(DS18B20_SKIP_ROM);
            DS18B20_writeByte(DS18B20_READ_SCRATCHPAD); // 读取上一次转换结果

            dsData->TempL = DS18B20_readByte();
            dsData->TempH = DS18B20_readByte();
            dsData->TempValue = ((dsData->TempH<<8) | dsData->TempL) / 16.0;

            dsData->readFlag = 1;
        }
    }
}
