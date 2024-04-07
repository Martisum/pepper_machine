#ifndef __MENU_H
#define __MENU_H
#include "main.h"

struct item                                     
{
    enum
    {
        subpage=1,                      
        value,
        switc,
        func,
        value_float,
        value_int32,
        title
    }type;

    int32_t dlt;                         
    float dlt_float;
    void *addr;                             
    void (*chf);                             

    char name[20];
};
struct huan
{
  int16_t  number;
};
struct pd
{
  float p;
  float p_out;
  float d;
  float d_out;
  float cs;
  float cs_out;
  float sd;
};

struct page
{
    uint8_t rpos;                          
    uint8_t pos;                           
    uint8_t count;
    uint8_t dymantic_page;                

    struct item itemlist[35];
};

//
/*************************************************************************************************/
//按键已经定义过了
// #define KEY_1       D0                      //up
// #define KEY_2       D3                     //down
// #define KEY_3       D4                     //add
// #define KEY_4       D1                     //sub
// #define KEY_5       B12                     //enter
// #define KEY_6       D11                     //leave
/***************************************************************************************************/
#define KEY_DelayTime   100                     //按键扫描延时时间
extern uint32_t cntpage;//页数
extern struct page *navigate[32];//指针

uint8_t key_scan(void);
void add_subpage(struct page * tg, char * name, struct page * v);
void add_value(struct page * tg, char * name, int * v, int16_t dt, void (*changedCallBack)());
void add_value_uint8(struct page * tg, char * name, uint8_t * v, uint8_t dt, void (*changedCallBack)());
void add_value_float(struct page * tg, char * name, float * v, float dt, void (*changedCallBack)());
void add_switc(struct page * tg, char * name, int16_t * v, void (*operate)());
void add_func(struct page * tg, char * name, void (*v)());
void add_value_int32(struct page * tg, char * name, int32_t * v, int32_t dt, void (*changedCallBack)());
void add_title(struct page * tg, char * name, char length);
void MenuRender(char full_update);
void OLED_BeginUpdate(void);
void MenuCmd(char cmd);
void key_init(void);//按键初始化
void MenuInit(struct page *mainpage);
void OLED_EndUpdate(void);
#endif
