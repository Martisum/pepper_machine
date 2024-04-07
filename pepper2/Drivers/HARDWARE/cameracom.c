#include "main.h"
#include "cameracom.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

#define UART3_RECVBUFLEN 30
uint8_t Uart3RecvBuf[UART3_RECVBUFLEN];
uint8_t Uart3RecvCnt;
uint8_t Uart3RecvCompleteFlag;
uint8_t Uart3RecvData[UART3_RECVBUFLEN]={0}; //ʵ�ʽ������ݵ����飨��Uart4RecvBufҲ���ǻ�����������ת�Ƴ�����
uint8_t Uart3RecvDataLen = 0; //ʵ�ʽ������ݵ�����������ݵĳ��ȣ���Uart4RecvCnt��ͬ����ʵ
extern DMA_HandleTypeDef hdma_usart3_rx;

int16_t openmv_pepper_x,openmv_pepper_y,openmv_pepper_dis;
uint8_t check_pepper_flag=0;

/* ��ʼ������ */
void Uart3RecvInit(void)
{
	/* ʹ�ܿ����ж� */
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	/* ���𴮿�DMA���� */
	HAL_UART_Receive_DMA(&huart3, Uart3RecvBuf, UART3_RECVBUFLEN);
}
 
/* �����жϻص����� */
void Uart3Recv_IdleCallback(void)
{
	/* �жϿ����жϷ��� */
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == SET)
	{
		/* ��������жϱ�־λ����ͣ����DMA���� */
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);	
		HAL_UART_DMAStop(&huart3);
		/* ���㵱ǰ����֡���յ������ݳ��� */
		Uart3RecvCnt = UART3_RECVBUFLEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		/* ֪ͨ��̨������򣬽������ */
		Uart3RecvCompleteFlag = 1;
	}
}
 
/* 
** �������ݽ��յ����ݴ����̨����
** ret: 0 - ok, -1 - ������
*/
int Uart3Recv_DealData(uint8_t *recvData, uint8_t *len)
{
	if(Uart3RecvCompleteFlag)
	{
		/* ���ݴ��� */
		strncpy((char *)recvData, (char *)Uart3RecvBuf, Uart3RecvCnt);
		*len = Uart3RecvCnt;
		
		/* �ָ���־λ����ǰ����֡���յ������ݳ��� */
		Uart3RecvCompleteFlag = 0;
		Uart3RecvCnt = 0;
		/* �ٴη�����һ�εĴ���DMA���� */
		HAL_UART_Receive_DMA(&huart3, Uart3RecvBuf, UART3_RECVBUFLEN);
		return 0;
	}
	else
	{
		return -1;
	}
}

/// @brief ��������Ƿ�����ڻ�������
/// @return ����1�����Բü�������0�������Բü�
uint8_t check_pepper(void){
	if(check_pepper_flag==1){
		check_pepper_flag=0;
		return 1;
	}
	return 0;
}
