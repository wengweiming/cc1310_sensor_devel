/*
 * sensor_dh21.h
 *
 *  Created on: 2017��9��28��
 *      Author: jgh-ninghy
 */

#ifndef SENSOR_DH21_H_
#define SENSOR_DH21_H_

typedef struct DH21_data {
    uint8_t humiH;      // ʪ�ȸ�λ����
    uint8_t humiL;      // ʪ�ȵ�λ����
    uint8_t tempH;      // �¶ȸ�λ����
    uint8_t tempL;      // �¶ȵ�λ����
    uint8_t check;      // У��λ
    uint16_t humiValue;    // �¶�����
    uint16_t tempValue;    // ʪ������
    uint8_t readFlag;   // �����ɹ���־
}DH21_data;

void DH21_read(DH21_data *dhData);
void DH21_init(void);
void delayUs(uint32_t);
void Delay_init(void);

#endif /* SENSOR_DH21_H_ */
