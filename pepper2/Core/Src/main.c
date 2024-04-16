/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
#include "tofsense.h"
#include "oled.h"
#include "menu.h"
#include "font.h"
#include "motor.h"
#include "cameracom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct page p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14;
float test1 = 0, test2 = 1, test3 = 2;
int16_t test4 = 3, test5 = 4, test6 = 5;

uint8_t wireless_test_in_flag=0;

void menu_init(void);
void menu_func_test(void);
void tof_test(void);
void pwm_test(void);
void spd_test(void);
void angle_confirm(void);
void wireless_test(void);
void strech_test(void);
void execute(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART4_RECVBUFLEN 10 //数组长度上限
uint8_t Uart4RecvBuf[UART4_RECVBUFLEN]={0}; //DMA缓冲区数组
uint8_t Uart4RecvCnt = 0; //DMA实际接收长度（数据包或长或短，有个实际长度）
uint8_t Uart4RecvCompleteFlag = 0; //接收完毕标志位
uint8_t Uart4RecvData[UART4_RECVBUFLEN]={0}; //实际接收数据的数组（从Uart4RecvBuf也就是缓冲区数组中转移出来）
uint8_t Uart4RecvDataLen = 0; //实际接收数据的数组接收数据的长度，和Uart4RecvCnt相同的其实

extern void Uart4RecvInit(void);
extern void Uart4Recv_IdleCallback(void);
int Uart4Recv_DealData(uint8_t *recvData, uint8_t *len);

extern uint16_t tim7_counter;
extern uint8_t coordinate_recv_cnt;
extern uint8_t alighment_cnt;
extern uint8_t strech_length;
extern uint8_t strech_retry_time;
extern uint8_t strech_ok_time;
extern uint8_t strech_wait_flag;
extern uint8_t strech_goback_flag;
extern uint8_t shear_ok_time;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /**********系统启动**********/
  //printf("SystemWakeUp!\n");
  HAL_UART_Transmit(&huart4, (uint8_t *)"SystemWakeUp!\n", 18,1000);
  Uart4RecvInit();

  /**********初始化无线**********/
  HAL_UART_Transmit(&huart5, (uint8_t *)"WirelessOK!\n", 12, 1000);
  Uart5RecvInit();
  /**********初始化无线**********/

  /**********初始化摄像头串口**********/
  Uart3RecvInit();
  /**********初始化摄像头串口**********/

  /**********初始化电机**********/
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
  /**********初始化电机**********/

  /**********初始化舵机**********/
  spd_pid_init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angle3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angle4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angle1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_angle2);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, servo_angle7);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servo_angle8);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, servo_angle5);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, servo_angle6);
  /**********初始化舵机**********/

  /**********初始化OLED与菜单**********/
  oled_init();
  oled_clear();
  menu_init();
  /**********初始化OLED与菜单**********/

  /**********初始化TOF**********/
  tof_init();
  /**********初始化TOF**********/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if(Uart4Recv_DealData(Uart4RecvData,&Uart4RecvDataLen)==0){
      //真正处理数据的区域
    //   Uart4RecvData[Uart4RecvDataLen]='\0';
    //   printf("%s\r\n",Uart4RecvData);
    }

		MenuCmd(key_scan());
    if (navigate[cntpage]->dymantic_page)
    {
      MenuRender(0);
      HAL_Delay(100);
    }
    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void execute(void)
{
  oled_clear();
  oled_show_string(0, 0, "execute()");
  char tmp_str[25]={0};
  tim7_counter=0;

  global_state=STROLL_STATE;
  //global_state=SPREAD_STATE;
  HAL_TIM_Base_Start_IT(&htim7);

  while (1)
  {
    oled_clear();
    if(global_state==STROLL_STATE){
      oled_show_string(0, 0, "state: STROLL_STATE");

      sprintf(tmp_str,"stroll_dir:%d",stroll_dir);
      oled_show_string(0, 1, tmp_str);

      sprintf(tmp_str,"co_recv_cnt:%d",coordinate_recv_cnt);
      oled_show_string(0, 2, tmp_str);
      
    }else if(global_state==LOCATE_STATE){
      oled_show_string(0, 0, "state: LOCATE_STATE");

      sprintf(tmp_str,"x:%d",x_pepper);
      oled_show_string(0, 1, tmp_str);

      sprintf(tmp_str,"y:%d",y_pepper);
      oled_show_string(0, 2, tmp_str);

      sprintf(tmp_str,"whlx_pwm:%d",pwm1); 
      oled_show_string(0, 3, tmp_str);   

      sprintf(tmp_str,"whly_pwm:%d",pwm2); 
      oled_show_string(0, 4, tmp_str);   

      sprintf(tmp_str,"alighment_cnt:%d",alighment_cnt);
      oled_show_string(0, 5, tmp_str); 

      sprintf(tmp_str,"now_error:%d",whlx.now_error); 
      oled_show_string(0, 6, tmp_str);   
    }else if(global_state==SPREAD_STATE){
#ifdef PERFORMANCE_MODE
      oled_show_string(0, 0, "state: SPREAD_STATE");

      sprintf(tmp_str,"cur_pulcnt:%d",cur_motor_pul_cnt);
      oled_show_string(0, 1, tmp_str);
#else
      oled_show_string(0, 0, "state: SPREAD_STATE");

      sprintf(tmp_str,"cur_pulcnt:%d",cur_motor_pul_cnt);
      oled_show_string(0, 1, tmp_str);

      sprintf(tmp_str,"retry_time:%d",strech_retry_time);
      oled_show_string(0, 2, tmp_str);

      sprintf(tmp_str,"ok_time:%d",strech_ok_time);
      oled_show_string(0, 3, tmp_str);

      sprintf(tmp_str,"wait_flag:%d",strech_wait_flag);
      oled_show_string(0, 4, tmp_str);

      sprintf(tmp_str,"goback_flag:%d",strech_goback_flag);
      oled_show_string(0, 5, tmp_str);

      sprintf(tmp_str,"tim7_counter:%d",tim7_counter);
      oled_show_string(0, 6, tmp_str);

      sprintf(tmp_str,"tar_pulcnt:%d",tar_motor_pul_cnt);
      oled_show_string(0, 7, tmp_str);
#endif
    }else if(global_state==SHEAR_STATE){
      oled_show_string(0, 0, "state: SHEAR_STATE");

      sprintf(tmp_str,"shear_ok_time:%d",shear_ok_time);
      oled_show_string(0, 1, tmp_str);

      sprintf(tmp_str,"tim7_counter:%d",tim7_counter);
      oled_show_string(0, 2, tmp_str);
    }
    

    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
        HAL_TIM_Base_Stop_IT(&htim7);
        //停止所有电机，复位所有舵机
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
        MenuRender(1);
        return;
      }
    }
  }
}

void step_motor_test(){
  oled_clear();
  oled_show_string(0, 0, "step_motor_test()");
  uint8_t test_length=5;
  char tmp_str[25]={0};

  HAL_GPIO_WritePin(ENA_GPIO_Port,ENA_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_SET);
  
  
  while(1){
    flexible_servo_control(test_length);
    sprintf(tmp_str,"tst_len:%d",test_length);
    oled_show_string(0, 1, tmp_str);
    sprintf(tmp_str,"cur_pulcnt:%d",cur_motor_pul_cnt);
    oled_show_string(0, 2, tmp_str);
    sprintf(tmp_str,"tar_pulcnt:%d",tar_motor_pul_cnt);
    oled_show_string(0, 3, tmp_str);
    // HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_SET);
    // //HAL_Delay(1);
    // delay_us(250);
    // HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_RESET);
    // //HAL_Delay(1);
    // delay_us(250);

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
        HAL_GPIO_WritePin(ENA_GPIO_Port,ENA_Pin, GPIO_PIN_RESET);
        MenuRender(1);
        return;
      }
    }
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
      {
        test_length=200;
      }
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
      {
        test_length=100;
      }
    }
  }
  
}

void wireless_test(void)
{
  oled_clear();
  oled_show_string(0, 0, "wireless_test()");

  char tmp_str[25]={0};
  static uint8_t isRecvPosition=0;
  wireless_test_in_flag=1;
  HAL_TIM_Base_Start_IT(&htim7);
  while (1)
  {
    sprintf(tmp_str,"x:%d",x_pepper);
    oled_show_string(0, 1, tmp_str);

    sprintf(tmp_str,"y:%d",y_pepper);
    oled_show_string(0, 2, tmp_str);

    sprintf(tmp_str,"whlx_pwm:%d",pwm1); 
    oled_show_string(0, 3, tmp_str);   

    sprintf(tmp_str,"isnopepper:%d",no_pepper_flag);
    oled_show_string(0, 4, tmp_str);

    HAL_Delay(10);

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET){
        //设备右移
        // set_motor_pwm(1,400);
        if(isRecvPosition==0){
          //发送关闭的信息
          char send_str[]="close";
          HAL_UART_Transmit(&huart5,(const uint8_t *)send_str,sizeof(send_str),1000);
          oled_show_string(0, 2, "sendoff");
          isRecvPosition=1;
        }else if(isRecvPosition==1){
          //发送开启的信息
          char send_str[]="open";
          HAL_UART_Transmit(&huart5,(const uint8_t *)send_str,sizeof(send_str),1000);
          oled_show_string(0, 2, "send on");
          isRecvPosition=0;
        }
      }
    }

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
        MenuRender(1);
        wireless_test_in_flag=0;
        HAL_TIM_Base_Stop_IT(&htim7);
        return;
      }
    }
  }
}

void strech_test(void)
{
  oled_clear();
  oled_show_string(0, 0, "strech_test()");

  static uint8_t cutting_state=0;
  uint8_t grab_state=0;
  static uint32_t strech_length_tmp=5;
  char tmp_str[25]={0};
  HAL_GPIO_WritePin(ENA_GPIO_Port,ENA_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim7);
  while (1)
  {
    sprintf(tmp_str,"op_x:%d",openmv_pepper_x);
    oled_show_string(0, 1, tmp_str);

    sprintf(tmp_str,"op_y:%d",openmv_pepper_y);
    oled_show_string(0, 2, tmp_str);

    sprintf(tmp_str,"op_dis:%d",openmv_pepper_dis);
    oled_show_string(0, 3, tmp_str);

    sprintf(tmp_str,"len_tmp:%d",strech_length_tmp);
    oled_show_string(0, 4, tmp_str);

    sprintf(tmp_str,"cur_pulcnt:%d",cur_motor_pul_cnt);
    oled_show_string(0, 5, tmp_str);

    sprintf(tmp_str,"cutting_state:%d",cutting_state);
    oled_show_string(0, 6, tmp_str);


    // if(check_pepper()==1){
    //   oled_show_string(0,6,"ok!!!");
    // }else{
    //   oled_show_string(0,6,"no!!!");
    // }
    
    flexible_servo_control(strech_length_tmp);

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET){
        //爪子前移
        //用来测试开环前移长度
        strech_length_tmp=pmode_length;
        //flexible_servo_control(140);
        oled_show_string(0, 2, "gone");
      }
    }else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET){
        //爪子后移
        strech_length_tmp=5;
        //flexible_servo_control(0);
        oled_show_string(0, 2, "back");
      }
      
    }else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET){
        //设备夹子抓取
        grab_servo_control(grab_state);
        grab_state=!grab_state;
        // set_motor_pwm(1,-400);
        // oled_show_string(0, 2, "motor lef");
      }
    }else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET){
        //设备右移
        // set_motor_pwm(1,400);
        // oled_show_string(0, 2, "motor rgh");
      }
      
    }else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET){
        cut_servo_control(cutting_state);
        cutting_state=!cutting_state;
        //设备右移
        // set_motor_pwm(1,400);
        //oled_show_string(0, 2, "cutt");
      }
    }

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
        HAL_TIM_Base_Stop_IT(&htim7);
        MenuRender(1);
        return;
      }
    }
  }
}

void angle_confirm(void)
{
  oled_clear();
  oled_show_string(0, 0, "angle_confirm()");

  set_servo_angle(1,servo_angle1);
  set_servo_angle(2,servo_angle2);
  set_servo_angle(3,servo_angle3);
  set_servo_angle(4,servo_angle4);
  set_servo_angle(5,servo_angle5);
  set_servo_angle(6,servo_angle6);
  set_servo_angle(7,servo_angle7);
  set_servo_angle(8,servo_angle8);
  
  oled_show_string(0, 1, "ok!");

  while (1)
  {
    oled_show_int(0,1,spd1,4);
    oled_show_int(0,2,spd2,4);
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
        MenuRender(1);
        return;
      }
    }
  }
}

void spd_test(void)
{
  oled_clear();
  oled_show_string(0, 0, "spd_test()");
  
  HAL_TIM_Base_Start_IT(&htim7);
  //大于零往左走
  set_motor_pwm(1,-600);
  set_motor_pwm(2,-900);
  

  while (1)
  {
    oled_show_int(0,1,spd1,4);
    oled_show_int(0,2,spd2,4);
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
        HAL_TIM_Base_Stop_IT(&htim7);
        set_motor_pwm(1,0);
        set_motor_pwm(2,0);
        MenuRender(1);
        return;
      }
    }
  }
}

void pwm_test(void)
{
  oled_clear();
  oled_show_string(0, 0, "pwm_test()");

  while (1)
  {
    static uint16_t test_pwm_cnt=100;
    test_pwm_cnt++;
    if(test_pwm_cnt>=900) test_pwm_cnt=100;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, test_pwm_cnt);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, test_pwm_cnt);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, test_pwm_cnt);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, test_pwm_cnt);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, test_pwm_cnt);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, test_pwm_cnt);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, test_pwm_cnt);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, test_pwm_cnt);

    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
        set_motor_pwm(1,0);
        set_motor_pwm(2,0);
        MenuRender(1);
        return;
      }
    }
  }
}

void tof_test(void)
{
  oled_clear();
  oled_show_string(0, 0, "tof_test()");

  char tmp_str[25]={0};
  while (1)
  {
    sprintf(tmp_str,"tof_distance:%0.2f",tof_distance);
    oled_show_string(0, 1, tmp_str);
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
          MenuRender(1);
          return;
      }
    }

    
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
      //设备上移
      set_motor_pwm(2,900);
      oled_show_string(0, 2, "motor upn");
    }else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
      //设备下移
      set_motor_pwm(2,-900);
      oled_show_string(0, 2, "motor dwn");
    }else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
      //设备左移
      set_motor_pwm(1,-800);
      oled_show_string(0, 2, "motor lef");
    }else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
    {
      //设备右移
      set_motor_pwm(1,800);
      oled_show_string(0, 2, "motor rgh");
    }else{
      set_motor_pwm(1,0);
      set_motor_pwm(2,0);
      //oled_show_string(0, 2, "motor none");
    }
  }
}

void menu_func_test(void)
{
  oled_clear();
  oled_show_string(0, 0, "menu_func_test()");

  oled_show_string(0, 1, "hello world!");
  oled_show_int(0, 2, 123, 3);
  oled_show_uint(0, 3, 456, 3);
  oled_show_float(0, 4, 3.14, 3, 3);

  while (1)
  {

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
          MenuRender(1);
          return;
      }
    }
  }
}

void servo_rotate_test(){
  oled_clear();
  oled_show_string(0, 0, "servo_rotate_test()");

  static uint16_t servo_rotate_cnt=100;
  while (1)
  {
    servo_rotate_cnt++;
    if(servo_rotate_cnt>=900) servo_rotate_cnt=100;
    set_servo_angle(3,servo_rotate_cnt);
    oled_show_int(0,1,servo_rotate_cnt,4);
    HAL_Delay(10);

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      HAL_Delay(KEY_DelayTime);
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
      {
          MenuRender(1);
          return;
      }
    }
  }
}

void menu_init(void){
  add_subpage(&p0, "function", &p1);
  add_subpage(&p0, "pid", &p2);
  add_subpage(&p0, "servo", &p3);

  add_func(&p1, "<execute>", execute);
  add_func(&p1, "<wireless_test>", wireless_test);
  add_func(&p1, "<angle_confirm>", angle_confirm);
  add_func(&p1, "<strech_test>", strech_test);
  add_func(&p1, "<step_motor_test>", step_motor_test);
  add_func(&p1, "<tof_test>", tof_test);
  add_func(&p1, "<s_rotate_test>", servo_rotate_test);
  add_func(&p1, "<pwm_test>", pwm_test);
  add_func(&p1, "<spd_test>", spd_test);
  add_func(&p1, "<menu_func_test>", menu_func_test);
  
  
  add_value(&p2, "[stroll_time]", (int *)&stroll_time, 10, NULL);
  add_value(&p2, "[pmode_length]", (int *)&pmode_length, 5, NULL);
  add_value(&p2, "[stroll_dir]", (int *)&stroll_dir, 1, NULL);
  add_value(&p2, "[test4]", (int *)&test4, 1, NULL);

  add_value(&p3, "[s_ang1]", (int *)&servo_angle1, 20, NULL);
  add_value(&p3, "[s_ang2]", (int *)&servo_angle2, 20, NULL);
  add_value(&p3, "[s_ang3]", (int *)&servo_angle3, 20, NULL);
  add_value(&p3, "[s_ang4]", (int *)&servo_angle4, 20, NULL);
  add_value(&p3, "[s_ang5]", (int *)&servo_angle5, 20, NULL);
  add_value(&p3, "[s_ang6]", (int *)&servo_angle6, 20, NULL);
  add_value(&p3, "[s_ang7]", (int *)&servo_angle7, 20, NULL);
  add_value(&p3, "[s_ang8]", (int *)&servo_angle8, 20, NULL);
  add_value_float(&p3, "[test3]", &test3, 0.3, NULL);

	MenuInit(&p0);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
