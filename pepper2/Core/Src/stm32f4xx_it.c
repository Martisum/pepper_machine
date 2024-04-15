/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
#include "tofsense.h"
#include "motor.h"
#include "stdlib.h"
#include "string.h"
#include "oled.h"
#include "cameracom.h"
#include "math.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#define UART4_RECVBUFLEN 10
extern uint8_t Uart4RecvBuf[UART4_RECVBUFLEN];
extern uint8_t Uart4RecvCnt;
extern uint8_t Uart4RecvCompleteFlag;
void Uart4RecvInit(void);
void Uart4Recv_IdleCallback(void);
int Uart4Recv_DealData(uint8_t *recvData, uint8_t *len);

extern uint8_t wireless_test_in_flag;

uint16_t tim7_counter=0;
uint8_t coordinate_recv_cnt=0;
uint8_t alighment_cnt=0;
uint8_t strech_length=0;
uint8_t strech_retry_time=0; //������λ������ʱ��
uint8_t strech_ok_time=0; //�ɹ�ץ������ͼ���֡��
uint8_t strech_wait_flag=0; //�ȴ�ȷ��openmv�Ѿ�ʶ�������ı�־λ
uint8_t strech_goback_flag=0; //�ȴ���еצ���صı�־λ
uint8_t shear_ok_time=0;
uint8_t isPepper=0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  Uart3Recv_IdleCallback();
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
  Uart4Recv_IdleCallback();
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
  Uart5Recv_IdleCallback();
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  if(Uart3Recv_DealData(Uart3RecvData,&Uart3RecvDataLen)==0){
    Uart3RecvData[Uart3RecvDataLen]='\0';
    printf("%s\r\n",Uart3RecvData);

    if(Uart3RecvData[0]=='['){
      uint8_t i=1;
      int16_t tmp_x_pepper=0;
      int16_t tmp_y_pepper=0;
      int16_t tmp_distance=0;
      while(Uart3RecvData[i]>='0' && Uart3RecvData[i]<='9'){
        tmp_x_pepper=10*tmp_x_pepper+Uart3RecvData[i]-'0';
        i++;
      }
      if(Uart3RecvData[i]==',') i++;
      while(Uart3RecvData[i]>='0' && Uart3RecvData[i]<='9'){
        tmp_y_pepper=10*tmp_y_pepper+Uart3RecvData[i]-'0';
        i++;
      }
      if(Uart3RecvData[i]==',') i++;
      while(Uart3RecvData[i]>='0' && Uart3RecvData[i]<='9'){
        tmp_distance=10*tmp_distance+Uart3RecvData[i]-'0';
        i++;
      }
      if(Uart3RecvData[i]==']'){
        openmv_pepper_x=tmp_x_pepper;
        openmv_pepper_y=tmp_y_pepper;
        openmv_pepper_dis=tmp_distance;
        check_pepper_flag=1;
      }
    }
  }

  if(Uart5Recv_DealData(Uart5RecvData,&Uart5RecvDataLen)==0){
      //�����������ݵ�����
      if(Uart5RecvData[0]=='['){
        uint8_t i=1;
        int16_t tmp_x_pepper=0;
        int16_t tmp_y_pepper=0;
        while(Uart5RecvData[i]>='0' && Uart5RecvData[i]<='9'){
          tmp_x_pepper=10*tmp_x_pepper+Uart5RecvData[i]-'0';
          i++;
        }
        if(Uart5RecvData[i]==',') i++;
        while(Uart5RecvData[i]>='0' && Uart5RecvData[i]<='9'){
          tmp_y_pepper=10*tmp_y_pepper+Uart5RecvData[i]-'0';
          i++;
        }
        if(Uart5RecvData[i]==']'){
          x_pepper=tmp_x_pepper;
          y_pepper=tmp_y_pepper;
          if(global_state==STROLL_STATE) coordinate_recv_cnt++;
          coordinate_recv_cnt++;
        }
      }

      if(Uart5RecvData[0]=='#'){
        if(Uart5RecvData[1]=='s'
        && Uart5RecvData[2]=='t'
        && Uart5RecvData[3]=='o'
        && Uart5RecvData[4]=='p'){
          oled_show_string(0,5,"stop");
          global_state=STOP_STATE;
        }
      }
      // Uart5RecvData[Uart5RecvDataLen]='\0';
      // printf("%s\r\n",Uart5RecvData);
    }

    //״̬��
    if(global_state==STROLL_STATE){
      flexible_servo_control(5);
      //�ۼ��յ�ʮ������������Ϣ����ô�ͻ���ת״̬ΪLOCATE_STATE
      if(stroll_dir==0) set_motor_pwm(1,600);
      else set_motor_pwm(1,-600);

      if(coordinate_recv_cnt>=15){
        global_state=LOCATE_STATE;
        //״̬��ת��Ϻ���ս��ռ���
        coordinate_recv_cnt=0;
        oled_clear();
      }
    }else if(global_state==LOCATE_STATE){
      set_x_location(x_pepper,GRAPH_CENTER_X);
      //�Է��������Ľ��мƴΣ�����200��˵�������ȶ�������Խ�����һ״̬
      if(abs(x_pepper-GRAPH_CENTER_X)<=50) alighment_cnt++;
      if(alighment_cnt>=200){
        //��ն�׼��������ת����һ״̬
        alighment_cnt=0;
        global_state=SPREAD_STATE;
        cut_servo_control(0);
        strech_length=0;
        strech_retry_time=0;
        strech_ok_time=0;
        strech_wait_flag=0;
        oled_clear();

        //ֹͣ���е������λ���ж��
        set_motor_pwm(1,0);
        set_motor_pwm(2,0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angle3);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angle4);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angle1);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_angle2);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, servo_angle7);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servo_angle8);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, servo_angle5);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, servo_angle6);
      }
    }else if(global_state==SPREAD_STATE){
      //��opemnv��Ұ��δ���ֿɲü������������쳤��û�е������ޣ��������ǰ����
      uint8_t isPepper=check_pepper();
      //isPepper=0;

      if(strech_ok_time!=0){
        tim7_counter++;
        strech_wait_flag=1;
        if(tim7_counter>100){
          tim7_counter=0;
          strech_wait_flag=0;
        }
      }

      if(cur_motor_pul_cnt<95 && isPepper==0 && strech_wait_flag==0 && strech_goback_flag==0){
        strech_ok_time=0;
        tar_motor_pul_cnt=90;
        flexible_servo_control(tar_motor_pul_cnt);
      }
      //�������Ѿ��������ޣ�������δ���ֿɲü���������ص���㣬�������²ü�
      else if(cur_motor_pul_cnt>90 && isPepper==0 && strech_wait_flag==0 && strech_goback_flag==0){
        strech_ok_time=0;
        tar_motor_pul_cnt=5;
        strech_goback_flag=1;
        flexible_servo_control(tar_motor_pul_cnt);
        strech_retry_time++; //����retryʱ��
      }else if(strech_goback_flag==1 && cur_motor_pul_cnt>5){ //����������elseif���Ǵ�����˳����������
        strech_ok_time=0;
        tar_motor_pul_cnt=5;
        flexible_servo_control(tar_motor_pul_cnt);
      }else if(strech_goback_flag==1 && cur_motor_pul_cnt<=5){
        strech_ok_time=0;
        tar_motor_pul_cnt=5;
        strech_goback_flag=0;
        flexible_servo_control(tar_motor_pul_cnt);
      }
      else if(isPepper==1){
        strech_ok_time++;
        if(strech_ok_time>=3){
          //��ת״̬������״̬
          global_state=SHEAR_STATE;
          shear_ok_time=0;
          tim7_counter=0;
          strech_wait_flag=0;
          oled_clear();
        }
      }

      //����Ѿ������˴���5�εĳ��ԣ����ж�Ϊ�Ҳ������ص�STROLL_STATE
      if(strech_retry_time>200){
        //ֹͣ���е������λ���ж��
        set_motor_pwm(1,0);
        set_motor_pwm(2,0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angle3);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angle4);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angle1);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_angle2);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, servo_angle7);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servo_angle8);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, servo_angle5);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, servo_angle6);
        global_state=STROLL_STATE;
        oled_clear();
      }


    }else if(global_state==SHEAR_STATE){
      uint8_t isPepper=check_pepper();
      //ֻҪ����ͷʼ����ʾ���������Ͳ��Ͻ��м��в���
      if(isPepper==1 || tim7_counter!=0){
        shear_ok_time=0;
        tim7_counter++;
        if(tim7_counter<150){
          cut_servo_control(0);
        }else if(tim7_counter>=150 && tim7_counter<300){
          cut_servo_control(1);
        }else if(tim7_counter>=300){
          tim7_counter=0;
        }
      }else if(isPepper==0 && tim7_counter==0){
        shear_ok_time++;
        cut_servo_control(1);
        if(shear_ok_time>25){
          cut_servo_control(0);
          tim7_counter=0;
          global_state=SHRINK_STATE;
          oled_clear();
        }
      }

    }else if(global_state==SHRINK_STATE){
      flexible_servo_control(5);
      cut_servo_control(0);

      tim7_counter++;
      if(tim7_counter>200){
        tim7_counter=0;
        global_state=STROLL_STATE;
        oled_clear();
      }
    }else if(global_state==STOP_STATE){
      set_motor_pwm(1,0);
      set_motor_pwm(2,0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angle3);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angle4);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angle1);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_angle2);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, servo_angle7);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servo_angle8);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, servo_angle5);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, servo_angle6);
      global_state=NORMAL_STATE;
    }
    // if(wireless_test_in_flag==1){
    //   set_x_location(x_pepper,GRAPH_CENTER_X);
    // }
    
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ 
  
  if(huart==&TOF_UART_HANDLE){
    TOF_UART_handler();
  }
}
 
/* Debug_uart rx DMA idle*/
/* ��ʼ������ */
void Uart4RecvInit(void)
{
	/* ʹ�ܿ����ж� */
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	/* ���𴮿�DMA���� */
	HAL_UART_Receive_DMA(&huart4, Uart4RecvBuf, UART4_RECVBUFLEN);
}
 
/* �����жϻص����� */
void Uart4Recv_IdleCallback(void)
{
	/* �жϿ����жϷ��� */
	if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) == SET)
	{
		/* ��������жϱ�־λ����ͣ����DMA���� */
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);	
		HAL_UART_DMAStop(&huart4);
		/* ���㵱ǰ����֡���յ������ݳ��� */
		Uart4RecvCnt = UART4_RECVBUFLEN - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
		/* ֪ͨ��̨������򣬽������ */
		Uart4RecvCompleteFlag = 1;
	}
}
 
/* 
** �������ݽ��յ����ݴ����̨����
** ret: 0 - ok, -1 - ������
*/
int Uart4Recv_DealData(uint8_t *recvData, uint8_t *len)
{
	if(Uart4RecvCompleteFlag)
	{
		/* ���ݴ��� */
		strncpy((char *)recvData, (char *)Uart4RecvBuf, Uart4RecvCnt);
		*len = Uart4RecvCnt;
		
		/* �ָ���־λ����ǰ����֡���յ������ݳ��� */
		Uart4RecvCompleteFlag = 0;
		Uart4RecvCnt = 0;
		/* �ٴη�����һ�εĴ���DMA���� */
		HAL_UART_Receive_DMA(&huart4, Uart4RecvBuf, UART4_RECVBUFLEN);
		return 0;
	}
	else
	{
		return -1;
	}
}
/* USER CODE END 1 */
