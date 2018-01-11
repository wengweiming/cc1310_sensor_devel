/*
 * sensor_bh1750.c
 *
 *  Created on: 2017��10��13��
 *      Author: jgh-ninghy
 */

#include <ti/drivers/I2C.h>
#include "Board.h"
#include "sensor_bh1750.h"

I2C_Params bh1750I2CParams;
I2C_Handle bh1750I2CHandle;

I2C_Transaction bh1750I2CTransac;

uint8_t bh1750TransferDone = 0;

/**
 * �� BH1750 ����
 */
void BH1750_read(uint8_t *writeBuf, uint8_t writeBufSize, uint8_t *readBuf, uint8_t readBufSize)
{
    // ����ṹ����
    bh1750I2CTransac.slaveAddress = BH1750_ADDR;
    bh1750I2CTransac.writeBuf = writeBuf;
    bh1750I2CTransac.writeCount = writeBufSize;
    bh1750I2CTransac.readBuf = readBuf;
    bh1750I2CTransac.readCount = readBufSize;

    I2C_transfer(bh1750I2CHandle, &bh1750I2CTransac);
}

/**
 * BH1750 ת����ɻص�
 * @ transferStatus: ת����ɱ�־
 */
void bh1750Callback(I2C_Handle handle, I2C_Transaction *transaction, bool transferStatus)
{
    if (transferStatus) {
        // �ɹ�ת�����ѽ��յ�����
        bh1750TransferDone = 1;
    }
}

/**
 * ��ʼ������
 * ����ÿ��ת�����ȴ�� 100ms��Ϊ�˲�ռ�ý��̣�����ʹ���жϵķ�ʽ
 */
void BH1750_init(void)
{
    // ��������
    I2C_Params_init(&bh1750I2CParams);
    bh1750I2CParams.transferMode = I2C_MODE_CALLBACK;
    bh1750I2CParams.transferCallbackFxn = bh1750Callback;
    bh1750I2CParams.bitRate = I2C_400kHz;
    // �� I2C
    bh1750I2CHandle = I2C_open(Board_I2C0, &bh1750I2CParams);
    if (bh1750I2CHandle == NULL) {
        while (1);
    }
}
