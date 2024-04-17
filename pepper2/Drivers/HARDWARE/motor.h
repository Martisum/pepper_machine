#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

// const uint8_t PERFORMANCE_MODE = 1;
#define PERFORMANCE_MODE 1
//**********PERFORMANCE_MODE_OPENLOOP_PARAM**********//
extern uint8_t pmode_length;
//**********PERFORMANCE_MODE_OPENLOOP_PARAM**********//

//#define STROLL_PERFORMANCE_MODE 1
//**********STROLL_PERFORMANCE_MODE_OPENLOOP_PARAM**********//
extern uint16_t stroll_time;
//**********STROLL_PERFORMANCE_MODE_OPENLOOP_PARAM**********//extern 

extern const uint16_t SAGEN;
//���ݶ����ʵ�������������������������޸�
#define SERVO1_NAME       1
#define BUSKET_SERVO      2
#define DOWN_STRECH_SERVO 3
#define CUTTING_SERVO     4
#define GRAB_SERVO        5
#define SERVO6_NAME       6
#define SERVO7_NAME 7
#define SERVO8_NAME 8

#define NORMAL_STATE 0 //����״̬������ʹ�ò˵�ִ�и�����
#define STROLL_STATE 1 //����״̬�����ҵ����\��ת ״̬��ת������ִ�к���
#define LOCATE_STATE 2 //��λ״̬��PID��λ����׼���� ״̬��ת����λ�����ͷ���������Ϣ
#define SPREAD_STATE 3 //�����еצ��ֱ������β��״̬ ״̬��ת����׼������ʱ����5������
#define SHEAR_STATE  4 //�ü�״̬���������� ״̬��ת��openmv�Լ�����ͷ��Ұ����������
#define SHRINK_STATE 5 //������еצ������м��õ������б��������һ���������� ״̬��ת���ޣ���ʱ��ص�״̬1
#define STOP_STATE   6 //ֹͣ״̬�������λ��������㣬�������õ�Ƭ�����ָܻ�

extern const uint16_t GRAPH_CENTER_X;

extern uint8_t global_state;
extern uint8_t stroll_dir;

extern int16_t spd1,spd2;
extern int16_t pwm1,pwm2;

extern uint16_t servo_angle1;
extern uint16_t servo_angle2;
extern uint16_t servo_angle3;
extern uint16_t servo_angle4;
extern uint16_t servo_angle5;
extern uint16_t servo_angle6;
extern uint16_t servo_angle7;
extern uint16_t servo_angle8;

#define UART5_RECVBUFLEN 30
extern uint8_t Uart5RecvBuf[UART5_RECVBUFLEN];
extern uint8_t Uart5RecvCnt;
extern uint8_t Uart5RecvCompleteFlag;
extern uint8_t Uart5RecvData[UART5_RECVBUFLEN]; //ʵ�ʽ������ݵ����飨��Uart4RecvBufҲ���ǻ�����������ת�Ƴ�����
extern uint8_t Uart5RecvDataLen; //ʵ�ʽ������ݵ�����������ݵĳ��ȣ���Uart4RecvCnt��ͬ����ʵ
extern int16_t x_pepper,y_pepper;
extern uint8_t no_pepper_flag; //��������־λ

extern uint16_t cur_motor_pul_cnt;
extern uint16_t tar_motor_pul_cnt;

typedef struct {
    int16_t set_targetS;
    int16_t pre_targetS;

    int PS;
    float IS;
    float DS;

    int16_t now_error;
    int16_t sum_error;
    int16_t pre_error;
    int diff_tar;

    int16_t pwm_out;
} SPEED;

extern SPEED whlx,whly;

void spd_pid_init(void);
void set_motor_pwm(uint8_t id,int16_t pwm);
void sent_data(int16_t A, int16_t B, int16_t C, int16_t D, int16_t E);
void set_servo_angle(uint8_t id,uint16_t angle);
void Uart5RecvInit(void);
void Uart5Recv_IdleCallback(void);
int Uart5Recv_DealData(uint8_t *recvData, uint8_t *len);
void set_x_location(int16_t now_x,int16_t tar_x);
void flexible_servo_control(uint16_t length);
void cut_servo_control(uint8_t state);
void grab_servo_control(uint8_t state);
void busket_servo_control(uint8_t state);
void delay_us(uint32_t us);

#endif /* __MOTOR_H__ */
