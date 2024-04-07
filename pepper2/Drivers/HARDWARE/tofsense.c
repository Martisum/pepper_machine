#include "main.h"
#include "tofsense.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

uint8_t tof_rx_data=0;
int16_t tof_rx_pack[20]={0};
uint8_t tof_send_pack[8]={0x57,0x10,0xFF,0xFF,0x00,0xFF,0xFF,0x63};

uint8_t query_state=0; 
//0:空闲，此时还没有任何数据可供解算，如需解算，则使用查询函数查询 
//1:忙线，此时的所有查询将被阻拦 
//2:就绪，此时可以利用函数解算距离
float tof_distance=0;

void tof_init(void){
    HAL_UART_Receive_IT(&TOF_UART_HANDLE,(uint8_t *)&tof_rx_data,1);
}

void TOF_UART_handler(void){
    static uint8_t tof_rx_loc=0;
    static uint32_t tof_distance_reg=0;

    if(tof_rx_loc==0 && tof_rx_data==0x57){
        tof_rx_pack[tof_rx_loc]=tof_rx_data;
        tof_rx_loc=1;                                        
    }else if(tof_rx_loc==1 && tof_rx_data==0x00){
        tof_rx_pack[tof_rx_loc]=tof_rx_data;
        tof_rx_loc=2;
    }else if(tof_rx_loc==2 && tof_rx_data==0xff){
        tof_rx_pack[tof_rx_loc]=tof_rx_data;
        tof_rx_loc=3;
    }else if(tof_rx_loc==3 && tof_rx_data==0x00){
        tof_rx_pack[tof_rx_loc]=tof_rx_data;
        tof_rx_loc=4;
    }else if(tof_rx_loc>=4 && tof_rx_loc<16){
        tof_rx_pack[tof_rx_loc]=tof_rx_data;
        tof_rx_loc++;
    }

    if(tof_rx_loc>=16){
        tof_distance_reg=0;
        tof_distance_reg+=tof_rx_pack[8];
        tof_distance_reg+=(tof_rx_pack[9]<<8);
        tof_distance_reg+=(tof_rx_pack[10]<<16);
        tof_distance=((float)(tof_distance_reg))/1;

        //printf("dis:%.2f\n",tof_distance);
        
        tof_rx_loc=0;
        memset(tof_rx_pack,0,sizeof(tof_rx_pack));
    }
    
    
    HAL_UART_Receive_IT(&TOF_UART_HANDLE,(uint8_t *)&tof_rx_data,1);
}

uint8_t Check_Sum(uint8_t *data,uint8_t len){
    uint16_t sum=0;
    for(uint8_t i=0;i<len;i++){
        sum+=data[i];
    }
    return sum & 0xFF;
}
