/*
 * sensor_bh1750.c
 *
 *  Created on: 2017年10月13日
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
 * 读 BH1750 数据
 */
void BH1750_read(uint8_t *writeBuf, uint8_t writeBufSize, uint8_t *readBuf, uint8_t readBufSize)
{
    // 传输结构配置
    bh1750I2CTransac.slaveAddress = BH1750_ADDR;
    bh1750I2CTransac.writeBuf = writeBuf;
    bh1750I2CTransac.writeCount = writeBufSize;
    bh1750I2CTransac.readBuf = readBuf;
    bh1750I2CTransac.readCount = readBufSize;

    I2C_transfer(bh1750I2CHandle, &bh1750I2CTransac);
}

/**
 * BH1750 转换完成回调
 * @ transferStatus: 转换完成标志
 */
void bh1750Callback(I2C_Handle handle, I2C_Transaction *transaction, bool transferStatus)
{
    if (transferStatus) {
        // 成功转换并已接收到数据
        bh1750TransferDone = 1;
    }
}

/**
 * 初始化参数
 * 由于每次转换长度大概 100ms，为了不占用进程，本次使用中断的方式
 */
void BH1750_init(void)
{
    // 参数配置
    I2C_Params_init(&bh1750I2CParams);
    bh1750I2CParams.transferMode = I2C_MODE_CALLBACK;
    bh1750I2CParams.transferCallbackFxn = bh1750Callback;
    bh1750I2CParams.bitRate = I2C_400kHz;
    // 打开 I2C
    bh1750I2CHandle = I2C_open(Board_I2C0, &bh1750I2CParams);
    if (bh1750I2CHandle == NULL) {
        while (1);
    }
}
