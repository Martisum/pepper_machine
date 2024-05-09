#include "main.h"
#include "motor.h"
#include "usart.h"
#include "string.h"
#include "tim.h"
#include "oled.h"
#include "math.h"

//**********PERFORMANCE_MODE_OPENLOOP_PARAM**********//
uint8_t pmode_length=70;
//**********PERFORMANCE_MODE_OPENLOOP_PARAM**********//

//**********STROLL_PERFORMANCE_MODE_OPENLOOP_PARAM**********//
uint16_t stroll_time=100;
//**********STROLL_PERFORMANCE_MODE_OPENLOOP_PARAM**********//

//const uint16_t GRAPH_CENTER_X=1536-716;
const uint16_t GRAPH_CENTER_X=1325;
const uint16_t GRAPH_CENTER_Y=358;
const uint16_t SAGEN=400;

uint8_t global_state=0;
uint8_t stroll_dir=0; //����STROLL״̬�µ����Ĭ���ƶ�����

//motor1Ϊ���µ�� motor2�����ҵ��
uint8_t motor1_dir=0,motor2_dir=0; //pwm��������¼��

int16_t spd1=0,spd2=0;
int16_t pwm1=0,pwm2=0;
int16_t error_debug;

uint16_t cur_motor_pul_cnt=0;
uint16_t tar_motor_pul_cnt=0;

// float v_kp1=4,v_ki1=1.5,v_kd1=0;
// float v_kp2=4,v_ki2=1.5,v_kd2=0;
float v_kp1=1,v_ki1=0,v_kd1=4;
float v_kp2=10,v_ki2=0,v_kd2=20;

SPEED whlx,whly;
SPEED dir;


DataPoint H_Data[MAX_DATA_POINTS];
float H_a,H_b;

//�����ţ�OLED�����˷��򣬴��������������Ϊ1 2 3 4 5 6 7 8 
//ע�⣬��һ�����벻�Ƕ����
//�����Ŷ�Ӧ��������壬�ο�.h�궨��
//�����Ǹ���Ŷ����ʼֵ
uint16_t servo_angle1=SAGEN;
uint16_t servo_angle2=650; //�����̶����ʼֵ
uint16_t servo_angle3=SAGEN;
uint16_t servo_angle4=350;
uint16_t servo_angle5=SAGEN;
uint16_t servo_angle6=SAGEN;
uint16_t servo_angle7=SAGEN;
uint16_t servo_angle8=SAGEN;

#define UART5_RECVBUFLEN 30
uint8_t Uart5RecvBuf[UART5_RECVBUFLEN];
uint8_t Uart5RecvCnt;
uint8_t Uart5RecvCompleteFlag;
uint8_t Uart5RecvData[UART5_RECVBUFLEN]={0}; //ʵ�ʽ������ݵ����飨��Uart4RecvBufҲ���ǻ�����������ת�Ƴ�����
uint8_t Uart5RecvDataLen = 0; //ʵ�ʽ������ݵ�����������ݵĳ��ȣ���Uart4RecvCnt��ͬ����ʵ
extern DMA_HandleTypeDef hdma_uart5_rx;
int16_t x_pepper,y_pepper; //�����õ�����������
uint8_t no_pepper_flag=0; //��������־λ
uint16_t P_Height_arr[10]={280,200,200,200,200,200,200,200,200,200}; //�����߶ȴ������
uint8_t parr_p=0;
uint16_t h_aligntment_cnt=0;
int16_t simu_tarh=0;
/* ��ʼ������ */
void Uart5RecvInit(void)
{
	/* ʹ�ܿ����ж� */
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	/* ���𴮿�DMA���� */
	HAL_UART_Receive_DMA(&huart5, Uart5RecvBuf, UART5_RECVBUFLEN);
}
 
/* �����жϻص����� */
void Uart5Recv_IdleCallback(void)
{
	/* �жϿ����жϷ��� */
	if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) == SET)
	{
		/* ��������жϱ�־λ����ͣ����DMA���� */
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);	
		HAL_UART_DMAStop(&huart5);
		/* ���㵱ǰ����֡���յ������ݳ��� */
		Uart5RecvCnt = UART5_RECVBUFLEN - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
		/* ֪ͨ��̨������򣬽������ */
		Uart5RecvCompleteFlag = 1;
	}
}
 
void delay_us(uint32_t us)
{
    for(uint32_t i = 0; i < us * 16; i++)
    {
        __NOP();
    }
}

/* 
** �������ݽ��յ����ݴ����̨����
** ret: 0 - ok, -1 - ������
*/
int Uart5Recv_DealData(uint8_t *recvData, uint8_t *len)
{
	if(Uart5RecvCompleteFlag)
	{
		/* ���ݴ��� */
		strncpy((char *)recvData, (char *)Uart5RecvBuf, Uart5RecvCnt);
		*len = Uart5RecvCnt;
		
		/* �ָ���־λ����ǰ����֡���յ������ݳ��� */
		Uart5RecvCompleteFlag = 0;
		Uart5RecvCnt = 0;
		/* �ٴη�����һ�εĴ���DMA���� */
		HAL_UART_Receive_DMA(&huart5, Uart5RecvBuf, UART5_RECVBUFLEN);
		return 0;
	}
	else
	{
		return -1;
	}
}

void spd_pid_init(){
    whlx.PS=v_kp1;
    whlx.IS=v_ki1;
    whlx.DS=v_kd1;

    whly.PS=v_kp2;
    whly.IS=v_ki2;
    whly.DS=v_kd2;
}

void set_x_location(int16_t now_x,int16_t tar_x){
    whlx.set_targetS=tar_x;
    whlx.now_error=whlx.set_targetS-now_x;

    whlx.pwm_out=whlx.PS*whlx.now_error+whlx.DS*(whlx.now_error-whlx.pre_error);
    whlx.pre_error=whlx.now_error;

    if(whlx.pwm_out<0) whlx.pwm_out-=350;
    else if(whlx.pwm_out>0) whlx.pwm_out+=350;

    if(whlx.pwm_out>1000) whlx.pwm_out=1000;
    else if(whlx.pwm_out<-1000) whlx.pwm_out=-1000;
    pwm1=whlx.pwm_out;
    set_motor_pwm(1,whlx.pwm_out);
}

//����now_yΪ��ǰ���ֵ��tar_yΪ��ͼ��߶�ת��������Ŀ����ֵ
//Ŀ����ֵ�����ʱ��Ҫ��һ�����ƣ����������������
//pwmΪ����������
void set_y_location(int16_t now_y,int16_t tar_y){
    whly.set_targetS=tar_y;
    whly.now_error=whly.set_targetS-now_y;

    whly.pwm_out=whly.PS*whly.now_error+whly.DS*(whly.now_error-whly.pre_error);
    oled_show_int(0,7,whly.pwm_out,5);
    whly.pre_error=whly.now_error;

    if(whly.pwm_out<0) whly.pwm_out-=250;
    else if(whly.pwm_out>0) whly.pwm_out+=250;

    if(whly.pwm_out>1000) whly.pwm_out=1000;
    else if(whly.pwm_out<-1000) whly.pwm_out=-1000;
    pwm2=whly.pwm_out;
    set_motor_pwm(2,-whly.pwm_out);
}

//1Ϊ���ҵ�� 2Ϊ���µ����
//���Ҷ��壺������OLED���������
//pwm����˵����1�����pwm>0���������ơ���֮���������ơ�
//idΪ�����ţ�ȡ1~2��pwmȡֵ-1000~1000
//����Ҫ��dir=0ʱ�������ţ���ɫƨ�ɳ����ұ�������ֵΪ��������gpio����ͱ���������ֵ����
void set_motor_pwm(uint8_t id,int16_t pwm){
    uint8_t dir=1;
    if(pwm<0) pwm=-pwm,dir=0;
    if(pwm>1000) pwm=1000;

    if(id==1){
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,(GPIO_PinState)dir);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm);
        motor1_dir=dir;
    }
    if(id==2){
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,(GPIO_PinState)dir);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm);
        motor2_dir=dir;
    }
}

void set_servo_angle(uint8_t id,uint16_t angle){
    if(id==1){
        servo_angle1=angle;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angle1);
    }else if(id==2){
        servo_angle2=angle;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_angle2);
    }else if(id==3){
        servo_angle3=angle;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_angle3);
    }else if(id==4){
        servo_angle4=angle;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_angle4);
    }else if(id==5){
        servo_angle5=angle;
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, servo_angle5);
    }else if(id==6){
        servo_angle6=angle;
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, servo_angle6);
    }else if(id==7){
        servo_angle7=angle;
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, servo_angle7);
    }else if(id==8){
        servo_angle8=angle;
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servo_angle8);
    }
}

//static const uint16_t FLEX_BIAS=15;
void flexible_servo_control(uint16_t length){
    tar_motor_pul_cnt=length;
    if(tar_motor_pul_cnt>cur_motor_pul_cnt){
        //���÷����
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);
        for(uint8_t i=0;i<50;i++){
            //gpio��һ
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_SET);
            delay_us(10);
            //gpio����
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_RESET);
            delay_us(10);
        }
        cur_motor_pul_cnt++;
    }else if(tar_motor_pul_cnt<cur_motor_pul_cnt){
        //���÷����
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_SET);
        for(uint8_t i=0;i<50;i++){
            //gpio��һ
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_SET);
            delay_us(10);
            //gpio����
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_RESET);
            delay_us(10);
        }
        cur_motor_pul_cnt--;
    }
    
}

//state=0�ſ����� state=1�պϼ���
void cut_servo_control(uint8_t state){
    if(state==0) set_servo_angle(CUTTING_SERVO,350);
    else if (state==1) set_servo_angle(CUTTING_SERVO,600);
}

//state=0�ſ����� state=1�պϼ���
void grab_servo_control(uint8_t state){
    if(state==1) set_servo_angle(GRAB_SERVO,350);
    else if (state==0) set_servo_angle(GRAB_SERVO,750);
}

//state=0�������� state=1�������
void busket_servo_control(uint8_t state){
    if(state==0) set_servo_angle(BUSKET_SERVO,650);
    else if (state==1) set_servo_angle(BUSKET_SERVO,250);
}

// �������Իع麯��
void linearRegression(DataPoint* data, int n, float* a, float* b) {
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < n; i++) {
        sumX += data[i].x;
        sumY += data[i].y;
        sumXY += data[i].x * data[i].y;
        sumX2 += data[i].x * data[i].x;
    }
    float denominator = n * sumX2 - sumX * sumX;
    *a = (n * sumXY - sumX * sumY) / denominator;
    *b = (sumY * sumX2 - sumX * sumXY) / denominator;
}

void height_data_init(void){
    H_Data[0].x=190;H_Data[0].y=178;
    H_Data[1].x=195;H_Data[1].y=235;
    H_Data[2].x=330;H_Data[2].y=271;
    H_Data[3].x=500;H_Data[3].y=320;
    H_Data[4].x=400;H_Data[4].y=273;
    H_Data[5].x=339;H_Data[5].y=249;
    H_Data[6].x=298;H_Data[6].y=227;
    H_Data[7].x=258;H_Data[7].y=203;
    H_Data[8].x=216;H_Data[8].y=177;
    //H_Data[9].x=9;H_Data[9].y=9;
    linearRegression(H_Data, 9, &H_a, &H_b);
}

//�����������ݽ��� state=0�ر����ݽ��� state=1�������ݽ���
void start_recv_coorData(uint8_t state){
    if(state==0){
        char send_str[]="close";
        HAL_UART_Transmit(&huart5,(const uint8_t *)send_str,sizeof(send_str),1000);
    }else if(state==1){
        char send_str[]="open";
        HAL_UART_Transmit(&huart5,(const uint8_t *)send_str,sizeof(send_str),1000);
    }    
}

// #define PC_WIRELESS 1
#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
uint8_t BUFF[30];
void sent_data(int16_t A, int16_t B, int16_t C, int16_t D, int16_t E) {
    int i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt = 0;
    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0XF1;
    BUFF[_cnt++] = 0x0A;
    BUFF[_cnt++] = BYTE0(A);
    BUFF[_cnt++] = BYTE1(A);
    BUFF[_cnt++] = BYTE0(B);
    BUFF[_cnt++] = BYTE1(B);
    BUFF[_cnt++] = BYTE0(C);
    BUFF[_cnt++] = BYTE1(C);
    BUFF[_cnt++] = BYTE0(D);
    BUFF[_cnt++] = BYTE1(D);
    BUFF[_cnt++] = BYTE0(E);
    BUFF[_cnt++] = BYTE1(E);

    for (i = 0; i < BUFF[3] + 4; i++) {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    for (i = 0; i < _cnt; i++) {
        //bluetooth_ch9141_send_buff_ch1(&BUFF[i], 1);
        //wireless_uart_send_byte((uint8) BUFF[i]);
        //uart_write_byte(UART_3, BUFF[i]);
        //HAL_UART_Transmit(&huart1, &BUFF[i], sizeof(BUFF[i]), 50);
        //printf("%d", BUFF[i]);
    }
}
