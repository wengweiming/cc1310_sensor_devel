/*
 * sensor_ds18b20.h
 *
 *  Created on: 2017��10��11��
 *      Author: jgh-ninghy
 */

#ifndef SENSOR_DS18B20_H_
#define SENSOR_DS18B20_H_

// DS18B20 �Ĵ�����ַ
#define DS18B20_READ_ROM        0x33
#define DS18B20_SKIP_ROM        0xCC
#define DS18B20_CONVERT_T       0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

typedef struct DS18B20_Data {
    uint8_t TempL;      // �¶ȵ�λ
    uint8_t TempH;      // �¶ȸ�λ
    float TempValue;    // �¶ȼ�����
    uint8_t readFlag;   // ����ɱ�־
}DS18B20_Data;

void DS18B20_init(void);
void DS18B20_read(DS18B20_Data *dsData);

#endif /* SENSOR_DS18B20_H_ */
