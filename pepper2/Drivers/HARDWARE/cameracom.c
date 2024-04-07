#include "main.h"
#include "cameracom.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

#define UART3_RECVBUFLEN 30
uint8_t Uart3RecvBuf[UART3_RECVBUFLEN];
uint8_t Uart3RecvCnt;
uint8_t Uart3RecvCompleteFlag;
uint8_t Uart3RecvData[UART3_RECVBUFLEN]={0}; //实际接收数据的数组（从Uart4RecvBuf也就是缓冲区数组中转移出来）
uint8_t Uart3RecvDataLen = 0; //实际接收数据的数组接收数据的长度，和Uart4RecvCnt相同的其实
extern DMA_HandleTypeDef hdma_usart3_rx;

int16_t openmv_pepper_x,openmv_pepper_y,openmv_pepper_dis;
uint8_t check_pepper_flag=0;

/* 初始化函数 */
void Uart3RecvInit(void)
{
	/* 使能空闲中断 */
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	/* 发起串口DMA接收 */
	HAL_UART_Receive_DMA(&huart3, Uart3RecvBuf, UART3_RECVBUFLEN);
}
 
/* 空闲中断回调函数 */
void Uart3Recv_IdleCallback(void)
{
	/* 判断空闲中断发生 */
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == SET)
	{
		/* 清除空闲中断标志位，暂停串口DMA传输 */
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);	
		HAL_UART_DMAStop(&huart3);
		/* 计算当前空闲帧接收到的数据长度 */
		Uart3RecvCnt = UART3_RECVBUFLEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		/* 通知后台处理程序，接收完成 */
		Uart3RecvCompleteFlag = 1;
	}
}
 
/* 
** 串口数据接收的数据处理后台函数
** ret: 0 - ok, -1 - 无数据
*/
int Uart3Recv_DealData(uint8_t *recvData, uint8_t *len)
{
	if(Uart3RecvCompleteFlag)
	{
		/* 数据处理 */
		strncpy((char *)recvData, (char *)Uart3RecvBuf, Uart3RecvCnt);
		*len = Uart3RecvCnt;
		
		/* 恢复标志位、当前空闲帧接收到的数据长度 */
		Uart3RecvCompleteFlag = 0;
		Uart3RecvCnt = 0;
		/* 再次发起下一次的串口DMA接收 */
		HAL_UART_Receive_DMA(&huart3, Uart3RecvBuf, UART3_RECVBUFLEN);
		return 0;
	}
	else
	{
		return -1;
	}
}

/// @brief 检查辣椒是否出现在画布中央
/// @return 返回1，可以裁剪。返回0，不可以裁剪
uint8_t check_pepper(void){
	if(check_pepper_flag==1){
		check_pepper_flag=0;
		return 1;
	}
	return 0;
}
