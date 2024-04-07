#ifndef __TOFSENSE_H__
#define __TOFSENSE_H__

#include "main.h"

#define TOF_UART_HANDLE huart2

extern float tof_distance;

void tof_init(void);
void TOF_UART_handler(void);
uint8_t Check_Sum(uint8_t *data,uint8_t len);

#endif /* __TOFSENSE_H__ */
