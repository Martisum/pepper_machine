#ifndef __OLED_H
#define __OLED_H

#include "main.h"
//====================================================硬件 SPI 驱动====================================================
//硬件SPI驱动已经由CUBE初始化完毕
//下面所有的宏定义均在main.h中实现
/*
#define OLED_SPI_SPEED                  (42 * 1000 * 1000)                      // 硬件 SPI 速率
#define OLED_SPI                        (SPI_1)                                 // 硬件 SPI 号
#define OLED_D0_PIN                     (SPI1_SCK_PA5 )                    // 硬件 SPI SCK 引脚
#define OLED_D1_PIN                     (SPI1_MOSI_PA7)                    // 硬件 SPI MOSI 引脚
#define OLED_RES_PIN                    (A3 )                                   // 液晶复位引脚定义
#define OLED_DC_PIN                     (A4 )                                   // 液晶命令位引脚定义
#define OLED_CS_PIN                     (GND )                                   // CS 片选引脚
*/
//====================================================硬件 SPI 驱动====================================================

#define OLED_BRIGHTNESS                 (0x7f)                                  // 设置OLED亮度 越大越亮 范围0-0XFF
#define OLED_DEFAULT_DISPLAY_DIR        (OLED_CROSSWISE)                        // 默认的显示方向
#define OLED_DEFAULT_DISPLAY_FONT       (OLED_6X8_FONT )                        // 默认的字体模式

//#define gpio_low(x)             ((GPIO_TypeDef*)gpio_group[(x>>5)])->BCR   = (uint16)(1 << (x & 0x0F))

typedef enum
{
    OLED_CROSSWISE                      = 0,                                    // 横屏模式
    OLED_CROSSWISE_180                  = 1,                                    // 横屏模式  旋转180
}oled_dir_enum;

typedef enum
{
    OLED_6X8_FONT                       = 0,                                    // 6x8      字体
    OLED_8X16_FONT                      = 1,                                    // 8x16     字体
    OLED_16X16_FONT                     = 2,                                    // 16x16    字体 目前不支持
}oled_font_size_enum;

#define OLED_X_MAX                      (128)
#define OLED_Y_MAX                      (64 )

void    oled_clear                      (void);
void    oled_full                       (const uint8_t color);
void    oled_set_dir                    (oled_dir_enum dir);
void    oled_set_font                   (oled_font_size_enum font);
void    oled_draw_point                 (uint16_t x, uint16_t y, const uint8_t color);

void    oled_show_string                (uint16_t x, uint16_t y, const char ch[]);
void    oled_show_int                   (uint16_t x, uint16_t y, const int32_t dat, uint8_t num);
void    oled_show_uint                  (uint16_t x, uint16_t y, const uint32_t dat, uint8_t num);
void    oled_show_float                 (uint16_t x, uint16_t y, const double dat, uint8_t num, uint8_t pointnum);

void    oled_show_binary_image          (uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height);
// void    oled_show_gray_image            (uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold);

// void    oled_show_wave                  (uint16_t x, uint16_t y, const uint16_t *image, uint16_t width, uint16_t value_max, uint16_t dis_width, uint16_t dis_value_max);
void    oled_show_chinese               (uint16_t x, uint16_t y, uint8_t size, const uint8_t *chinese_buffer, uint8_t number);
void    oled_set_coordinate             (uint8_t x, uint8_t y);
void    oled_write_data                 (const uint8_t data);
void    oled_init                       (void);

#endif
