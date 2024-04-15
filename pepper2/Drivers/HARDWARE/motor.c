#include "main.h"
#include "motor.h"
#include "usart.h"
#include "string.h"
#include "tim.h"
#include "oled.h"
#include "math.h"

//**********PERFORMANCE_MODE_OPENLOOP_PARAM**********//
uint8_t pmode_length=110;
//**********PERFORMANCE_MODE_OPENLOOP_PARAM**********//

//const uint16_t GRAPH_CENTER_X=1536-716;
const uint16_t GRAPH_CENTER_X=1790;
const uint16_t SAGEN=400;

uint8_t global_state=0;
uint8_t stroll_dir=0; //定义STROLL状态下电机的默认移动方向

//motor1为上下电机 motor2是左右电机
uint8_t motor1_dir=0,motor2_dir=0; //pwm电机方向记录器

int16_t spd1=0,spd2=0;
int16_t pwm1=0,pwm2=0;
int16_t error_debug;

uint16_t cur_motor_pul_cnt=0;
uint16_t tar_motor_pul_cnt=0;

// float v_kp1=4,v_ki1=1.5,v_kd1=0;
// float v_kp2=4,v_ki2=1.5,v_kd2=0;
float v_kp1=1,v_ki1=0,v_kd1=4;
float v_kp2=1,v_ki2=0,v_kd2=0;

SPEED whlx,whly;
SPEED dir;

//舵机编号，OLED正对人方向，从上往下数，编号为1 2 3 4 5 6 7 8 
//注意，第一组排针不是舵机！
//具体编号对应舵机的意义，参考.h宏定义
//下面是各编号舵机初始值
uint16_t servo_angle1=SAGEN;
uint16_t servo_angle2=SAGEN;
uint16_t servo_angle3=SAGEN;
uint16_t servo_angle4=SAGEN;
uint16_t servo_angle5=SAGEN;
uint16_t servo_angle6=SAGEN;
uint16_t servo_angle7=SAGEN;
uint16_t servo_angle8=SAGEN;

#define UART5_RECVBUFLEN 30
uint8_t Uart5RecvBuf[UART5_RECVBUFLEN];
uint8_t Uart5RecvCnt;
uint8_t Uart5RecvCompleteFlag;
uint8_t Uart5RecvData[UART5_RECVBUFLEN]={0}; //实际接收数据的数组（从Uart4RecvBuf也就是缓冲区数组中转移出来）
uint8_t Uart5RecvDataLen = 0; //实际接收数据的数组接收数据的长度，和Uart4RecvCnt相同的其实
extern DMA_HandleTypeDef hdma_uart5_rx;
int16_t x_pepper,y_pepper; //解算后得到的辣椒坐标
uint8_t no_pepper_flag=0; //无辣椒标志位
/* 初始化函数 */
void Uart5RecvInit(void)
{
	/* 使能空闲中断 */
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	/* 发起串口DMA接收 */
	HAL_UART_Receive_DMA(&huart5, Uart5RecvBuf, UART5_RECVBUFLEN);
}
 
/* 空闲中断回调函数 */
void Uart5Recv_IdleCallback(void)
{
	/* 判断空闲中断发生 */
	if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) == SET)
	{
		/* 清除空闲中断标志位，暂停串口DMA传输 */
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);	
		HAL_UART_DMAStop(&huart5);
		/* 计算当前空闲帧接收到的数据长度 */
		Uart5RecvCnt = UART5_RECVBUFLEN - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
		/* 通知后台处理程序，接收完成 */
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
** 串口数据接收的数据处理后台函数
** ret: 0 - ok, -1 - 无数据
*/
int Uart5Recv_DealData(uint8_t *recvData, uint8_t *len)
{
	if(Uart5RecvCompleteFlag)
	{
		/* 数据处理 */
		strncpy((char *)recvData, (char *)Uart5RecvBuf, Uart5RecvCnt);
		*len = Uart5RecvCnt;
		
		/* 恢复标志位、当前空闲帧接收到的数据长度 */
		Uart5RecvCompleteFlag = 0;
		Uart5RecvCnt = 0;
		/* 再次发起下一次的串口DMA接收 */
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

    if(whlx.pwm_out<0) whlx.pwm_out-=300;
    else if(whlx.pwm_out>0) whlx.pwm_out+=300;

    if(whlx.pwm_out>1000) whlx.pwm_out=1000;
    else if(whlx.pwm_out<-1000) whlx.pwm_out=-1000;
    pwm1=whlx.pwm_out;
    set_motor_pwm(1,whlx.pwm_out);
}

//这里now_y为当前测距值，tar_y为由图像高度转化而来的目标测距值
//目标测距值输入的时候要有一个限制，避免出现无限下移
void set_y_location(int16_t now_y,int16_t tar_y){
    whly.set_targetS=tar_y;
    whly.now_error=whly.set_targetS-tar_y;

    whly.pwm_out=whly.PS*whlx.now_error+whly.DS*(whly.now_error-whly.pre_error);
    whly.pre_error=whly.now_error;

    if(whly.pwm_out<0) whly.pwm_out-=350;
    else if(whly.pwm_out>0) whly.pwm_out+=350;

    if(whly.pwm_out>1000) whly.pwm_out=1000;
    else if(whly.pwm_out<-1000) whly.pwm_out=-1000;
    pwm2=whly.pwm_out;
    set_motor_pwm(1,-whly.pwm_out);
}

//1为左右电机 2为上下电机、
//左右定义：人正对OLED方向的左右
//pwm极性说明：1电机，pwm>0，机器右移。反之，机器左移。
//id为电机编号，取1~2。pwm取值-1000~1000
//极性要求：dir=0时，电机横放，黑色屁股朝左，且编码器数值为正，调整gpio输出和编码器正负值即可
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
        //设置方向脚
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_SET);
        for(uint8_t i=0;i<50;i++){
            //gpio置一
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_SET);
            delay_us(10);
            //gpio置零
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_RESET);
            delay_us(10);
        }
        cur_motor_pul_cnt++;
    }else if(tar_motor_pul_cnt<cur_motor_pul_cnt){
        //设置方向脚
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);
        for(uint8_t i=0;i<50;i++){
            //gpio置一
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_SET);
            delay_us(10);
            //gpio置零
            HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin, GPIO_PIN_RESET);
            delay_us(10);
        }
        cur_motor_pul_cnt--;
    }
    
}

//state=0张开剪刀 state=1闭合剪刀
void cut_servo_control(uint8_t state){
    if(state==0) set_servo_angle(CUTTING_SERVO,350);
    else if (state==1) set_servo_angle(CUTTING_SERVO,800);
}

//state=0张开夹子 state=1闭合夹子
void grab_servo_control(uint8_t state){
    if(state==0) set_servo_angle(GRAB_SERVO,350);
    else if (state==1) set_servo_angle(GRAB_SERVO,800);
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
