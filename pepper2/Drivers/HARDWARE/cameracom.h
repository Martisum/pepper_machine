#ifndef __CAMERACOM_H__
#define __CAMERACOM_H__

#include "main.h"

#define UART3_RECVBUFLEN 30
extern uint8_t Uart3RecvData[UART3_RECVBUFLEN]; 
extern uint8_t Uart3RecvDataLen;
extern int16_t openmv_pepper_x,openmv_pepper_y,openmv_pepper_dis;
extern uint8_t check_pepper_flag;

void Uart3RecvInit(void);
void Uart3Recv_IdleCallback(void);
int Uart3Recv_DealData(uint8_t *recvData, uint8_t *len);
uint8_t check_pepper(void);

#endif /* __CAMERACOM_H__ */
