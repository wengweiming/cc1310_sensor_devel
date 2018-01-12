/*
 * sensor_ds18b20.c
 *
 *  Created on: 2017��10��11��
 *      Author: jgh-ninghy
 */

#include <ti/drivers/PIN.h>

#include "board.h"
#include "sensor_ds18b20.h"
#include "delay.h"

// ���������ߵĲ���
#define DS18B20_WRITE_HIGH     (PIN_setOutputValue(ds18b20PinHandle, Board_DS18B20, 1))
#define DS18B20_WRITE_LOW      (PIN_setOutputValue(ds18b20PinHandle, Board_DS18B20, 0))
#define DS18B20_READ           (PIN_getInputValue(Board_DS18B20))

// ��©���ģʽ����
static PIN_Config ds18b20OutputPinTable[] =
{
    Board_DS18B20 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_OPENDRAIN | PIN_INPUT_EN,
    PIN_TERMINATE
};

static PIN_State ds18b20PinState;
static PIN_Handle ds18b20PinHandle;

void DS18B20_init(void)
{
    // ��ʼ�� PIN
    ds18b20PinHandle = PIN_open(&ds18b20PinState, ds18b20OutputPinTable);
}

/**
 * д�� 1Bit
 */
void DS18B20_writeOneBit(uint8_t dat)
{
    DS18B20_WRITE_LOW;     // ���� >1us
    Delay_shortUS(2);
    PIN_setOutputValue(ds18b20PinHandle, Board_DS18B20, dat);   // дʱ϶ >60us
    Delay_shortUS(70);
    DS18B20_WRITE_HIGH;    // �ͷ�����
    Delay_shortUS(2);    // д����һλ���ݵļ�� >1us
}

/**
 * д�� 1Byte
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
 * ���� 1Bit
 */
uint8_t DS18B20_readOneBit(void)
{
    uint8_t dat;

    DS18B20_WRITE_LOW;     // ���� >1us
    Delay_shortUS(2);
    DS18B20_WRITE_HIGH;    // �ͷ�����
    Delay_shortUS(5);
    // 15us �ڶ�����
    dat = DS18B20_READ;
    Delay_shortUS(70);       // ��֤ÿ�� ��ʱ϶ >60us
    DS18B20_WRITE_HIGH;    // �ͷ�����
    Delay_shortUS(2);    // ��ȡ��һλ���ݵļ�� >1us

    return dat;
}

/**
 * ��ȡ 1Byte
 */
uint8_t DS18B20_readByte(void)
{
    uint8_t dat;
    uint8_t i;

    for (i=0; i<8; ++i) {
        dat >>= 1;
        if (DS18B20_readOneBit()) {
            dat |= 0x80;    // ���������Ǵ� ��λ ��ʼ��
        }
    }

    return dat;
}

uint8_t DS18B20_start(void)
{
    DS18B20_WRITE_HIGH;
    Delay_shortUS(10);
    DS18B20_WRITE_LOW;     // �������� >480us
    Delay_shortUS(800);
    DS18B20_WRITE_HIGH;    // �ͷ�����
    Delay_shortUS(80);
    // �ȴ� >60us���жϴӻ��Ƿ���Ӧ
    if (!DS18B20_READ) {
        return 1;
    }
    return 0;
}

/**
 * ��ȡ DS18B20 ����
 * �ú���Ҫ��������ֻ��һ�� DS18B20 �������ڶ��������Ҫд��������ָ����ж�д
 * ���ζ�ȡ���Ϊ��һ��ת���������Ϊ 12bit �ֱ����£�ת��ʱ�������Ҫ 950ms
 */
void DS18B20_read(DS18B20_Data *dsData)
{
    dsData->readFlag = 0;
    if (DS18B20_start()) {
        while (!DS18B20_READ);     // �ȴ��ӻ���Ӧ�źŽ���
        Delay_shortUS(500);      // ���ڱ�֤ Master Rx >480us
        DS18B20_writeByte(DS18B20_SKIP_ROM);    // ����ֻ��һ�� DS18B20 ,���������Ĺ���
        DS18B20_writeByte(DS18B20_CONVERT_T);    // ����ת��
        if (DS18B20_start()) {
            while (!DS18B20_READ);     // �ȴ��ӻ���Ӧ�źŽ���
            Delay_shortUS(300);      // ���ڱ�֤ Master Rx >480us
            DS18B20_writeByte(DS18B20_SKIP_ROM);
            DS18B20_writeByte(DS18B20_READ_SCRATCHPAD); // ��ȡ��һ��ת�����

            dsData->TempL = DS18B20_readByte();
            dsData->TempH = DS18B20_readByte();
            dsData->TempValue = ((dsData->TempH<<8) | dsData->TempL) / 16.0;

            dsData->readFlag = 1;
        }
    }
}
