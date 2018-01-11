/*
 * sensor_bh1750.h
 *
 *  Created on: 2017��10��13��
 *      Author: jgh-ninghy
 */

#ifndef SENSOR_BH1750_H_
#define SENSOR_BH1750_H_

#define BH1750_ADDR         0x23        // ADDR �ӵ͵�ƽ ( 010_0011)
#define ONE_H_MODE_1        0x20        // һ�� H �ֱ���ģʽ

extern uint8_t bh1750TransferDone;

void BH1750_init(void);
void BH1750_read(uint8_t *writeBuf, uint8_t writeBufSize, uint8_t *readBuf, uint8_t readBufSize);

#endif /* SENSOR_BH1750_H_ */
