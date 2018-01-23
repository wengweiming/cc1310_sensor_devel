/*
 * sensor_dh21.h
 *
 *  Created on: 2017年9月28日
 *      Author: jgh-ninghy
 */

#ifndef SENSOR_DH21_H_
#define SENSOR_DH21_H_

typedef struct DH21_data {
    uint8_t humiH;      // 湿度高位数据
    uint8_t humiL;      // 湿度低位数据
    uint8_t tempH;      // 温度高位数据
    uint8_t tempL;      // 温度低位数据
    uint8_t check;      // 校验位
    uint16_t humiValue;    // 温度数据
    uint16_t tempValue;    // 湿度数据
    uint8_t readFlag;   // 读数成功标志
}DH21_data;

void DH21_read(DH21_data *dhData);
void DH21_init(void);
void delayUs(uint32_t);
void Delay_init(void);

#endif /* SENSOR_DH21_H_ */
