/*
 * sensor_dh21.c
 *
 *  Created on: 2017��9��28��
 *      Author: jgh-ninghy
 */

#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include "board.h"
#include "sensor_dh21.h"
#include "delay.h"

// ���������ߵĲ���
#define DH21_ONEWIRE_WRITE_HIGH     (PIN_setOutputValue(oneWirePinHandle, Board_DHT21, 1))
#define DH21_ONEWIRE_WRITE_LOW      (PIN_setOutputValue(oneWirePinHandle, Board_DHT21, 0))
#define DH21_ONEWIRE_READ           (PIN_getInputValue(Board_DHT21))

// ���ģʽ����
static PIN_Config oneWireOutputPinTable[] =
{
    Board_DHT21 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX | PIN_INPUT_DIS,
    PIN_TERMINATE
};

// ����ģʽ����
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
 * �������Ϳ�ʼ�ź�
 */
uint8_t DH21_start(void)
{
    PIN_setConfig(oneWirePinHandle, PIN_BM_ALL, *oneWireOutputPinTable);    // ���ó����ģʽ
    DH21_ONEWIRE_WRITE_HIGH;    // output High
    DH21_ONEWIRE_WRITE_LOW;     // ��������
    Delay_shortUS(1000);      // >500us
    PIN_setConfig(oneWirePinHandle, PIN_BM_ALL, *oneWireInputPinTable); // ���ó�����ģʽ
    Delay_shortUS(50);        // 20-40us
    // �ж� 50us ֮�� DH21 �Ƿ�����Ӧ
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
        while(DH21_ONEWIRE_READ);   // �����һ�����ݵ�ƽ
        while(!DH21_ONEWIRE_READ);  // 1Bit ��ʼ�ź� 50us
        Delay_shortUS(40);    // 26 - 28us : ��ʾ 0
                         // 70us : ��ʾ 1
        // �ȴ� 40us
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
        // �����ɹ�
        while(!DH21_ONEWIRE_READ);     // �ȴ� DH21 ��Ӧ�źŽ����� 80us
        while(DH21_ONEWIRE_READ);      // DH21 �������� 80us
        // �������ݽ���
        dhData->humiH = DH21_readByte();
        dhData->humiL = DH21_readByte();
        dhData->tempH = DH21_readByte();
        dhData->tempL = DH21_readByte();
        dhData->check = DH21_readByte();
        // ����У��
        if (dhData->check == (uint8_t)(dhData->humiH + dhData->humiL +
                                       dhData->tempH + dhData->tempL)) {
            // У��ɹ�
            dhData->readFlag = 1;
            dhData->humiValue = (dhData->humiH << 8) + dhData->humiL;
            dhData->tempValue = (dhData->tempH << 8) + dhData->tempL;
        }
    }
}

