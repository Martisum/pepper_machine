#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "oled.h"
#include "font.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define myabs(x)                        ((x) >= 0 ? (x): -(x))

static oled_dir_enum        oled_display_dir    = OLED_DEFAULT_DISPLAY_DIR;
static oled_font_size_enum  oled_display_font   = OLED_DEFAULT_DISPLAY_FONT;

/***************************************************
    I2C总线传出数据函数：
                addr  :    要写入的地址（OLED的地址一般为0x40;指令地址为0x00）
                data  :    要写入的数据
***************************************************/
void HAL_I2C_WriteByte(uint8_t addr,uint8_t data)
{
	uint8_t TxData[2] = {addr,data};
	HAL_I2C_Master_Transmit(&hi2c1,0X78,(uint8_t*)TxData,2,10);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写命令
// 参数说明     cmd             命令
// 返回参数     void
// 使用示例     oled_write_command(0xb0 + y);
// 备注信息     内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void oled_write_command (uint8_t IIC_Command)
{
	HAL_I2C_WriteByte(0x00, IIC_Command);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写8位数据
// 参数说明     data            数据
// 返回参数     void
// 使用示例     oled_write_data(color);
// 备注信息     内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
void oled_write_data (uint8_t IIC_Data)
{
    HAL_I2C_WriteByte(0x40, IIC_Data);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 清屏函数
// 参数说明     void
// 返回参数     void
// 使用示例     oled_clear();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_clear (void)
{
    uint8_t y = 0, x = 0;

    //OLED_CS(0);
    for(y = 0; 8 > y; y ++)
    {
        oled_write_command(0xb0 + y);
        oled_write_command(0x01);
        oled_write_command(0x10);
        for(x = 0; OLED_X_MAX > x; x ++)
        {
            oled_write_data(0x00); 
        }
    }
    //OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED显示坐标设置
// 参数说明     x               x轴坐标设置0-127
// 参数说明     y               y轴坐标设置0-7
// 返回参数     void
// 使用示例     oled_set_coordinate(x, y);
// 备注信息     内部使用用户无需关心
//-------------------------------------------------------------------------------------------------------------------
void oled_set_coordinate (uint8_t x, uint8_t y)
{
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // 检查一下你的显示调用的函数 自己计算一下哪里超过了屏幕显示范围
//    zf_assert(128 > x);
//    zf_assert(8 > y);

    oled_write_command(0xb0 + y);
    oled_write_command(((x & 0xf0) >> 4) | 0x10);
    oled_write_command((x & 0x0f) | 0x00);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 屏幕填充函数
// 参数说明     color           填充颜色选着(0x00 or 0xff)
// 返回参数     void
// 使用示例     oled_full(0x00);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_full (const uint8_t color)
{
    uint8_t y = 0, x = 0;

    //OLED_CS(0);
    for(y = 0; 8 > y; y ++)
    {
        oled_write_command(0xb0 + y);
        oled_write_command(0x01);
        oled_write_command(0x10);
        for(x = 0; OLED_X_MAX > x; x ++)
        {
            oled_write_data(color); 
        }
    }
    //OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置显示方向
// 参数说明     dir             显示方向  参照 zf_device_oled.h 内 oled_dir_enum 枚举体定义
// 返回参数     void
// 使用示例     oled_set_dir(OLED_CROSSWISE);
// 备注信息     这个函数只有在初始化屏幕之前调用才生效
//-------------------------------------------------------------------------------------------------------------------
void oled_set_dir (oled_dir_enum dir)
{
    oled_display_dir = dir;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置显示字体
// 参数说明     dir             显示方向  参照 zf_device_oled.h 内 oled_font_size_enum 枚举体定义
// 返回参数     void
// 使用示例     oled_set_font(OLED_8x16_FONT);
// 备注信息     字体可以随时自由设置 设置后生效 后续显示就是新的字体大小
//-------------------------------------------------------------------------------------------------------------------
void oled_set_font (oled_font_size_enum font)
{
    oled_display_font = font;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画点函数
// 参数说明     x               x 轴坐标设置 0-127
// 参数说明     y               y 轴坐标设置 0-7
// 参数说明     color           8 个点数据
// 返回参数     void
// 使用示例     oled_draw_point(0, 0, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_draw_point (uint16_t x, uint16_t y, const uint8_t color)
{
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // 检查一下你的显示调用的函数 自己计算一下哪里超过了屏幕显示范围
    // zf_assert(128 > x);
    // zf_assert(8 > y);

    //OLED_CS(0);
    oled_set_coordinate(x, y);
    oled_write_command(0xb0 + y);
    oled_write_command(((x & 0xf0) >> 4) | 0x10);
    oled_write_command((x & 0x0f) | 0x00);
    oled_write_data(color);
    //OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 显示字符串
// 参数说明     x               x 轴坐标设置 0-127
// 参数说明     y               y 轴坐标设置 0-7
// 参数说明     ch[]            字符串
// 返回参数     void
// 使用示例     oled_show_string(0, 0, "SEEKFREE");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_show_string (uint16_t x, uint16_t y, const char ch[])
{
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // 检查一下你的显示调用的函数 自己计算一下哪里超过了屏幕显示范围
    // zf_assert(128 > x);
    // zf_assert(8 > y);

    //OLED_CS(0);
    uint8_t c = 0, i = 0, j = 0;
    while ('\0' != ch[j])
    {
        switch(oled_display_font)
        {
            case OLED_6X8_FONT:
            {
                c = ch[j] - 32;
                if(x > 126)
                {
                    x = 0;
                    y ++;
                }
                oled_set_coordinate(x, y);
                for(i = 0; 6 > i; i ++)
                {
                    oled_write_data(ascii_font_6x8[c][i]);
                }
                x += 6;
                j ++;
            }break;
            case OLED_8X16_FONT:
            {
                c = ch[j] - 32;
                if(x > 120)
                {
                    x = 0;
                    y ++;
                }
                oled_set_coordinate(x, y);
                for(i = 0; i < 8; i ++)
                {
                    oled_write_data(ascii_font_8x16[c][i]);
                }

                oled_set_coordinate(x, y + 1);
                for(i = 0; i < 8; i ++)
                {
                    oled_write_data(ascii_font_8x16[c][i + 8]);
                }
                x += 8;
                j ++;
            }break;
            case OLED_16X16_FONT:
            {
                // 暂不支持
            }break;
        }
    }
    //OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [-32768,32767]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_int_to_str(data_buffer, -300);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_int_to_str (char *str, int32_t number)
{
    //zf_assert(str != NULL);
    uint8_t data_temp[16];                                                        // 缓冲区
    uint8_t bit = 0;                                                              // 数字位数
    int32_t number_temp = 0;

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 > number)                                                          // 负数
        {
            *str ++ = '-';
            number = -number;
        }
        else if(0 == number)                                                    // 或者这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            number_temp = number % 10;
            data_temp[bit ++] = myabs(number_temp);                          // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [0,65535]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_uint_to_str(data_buffer, 300);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_uint_to_str (char *str, uint32_t number)
{
    //zf_assert(str != NULL);
    int8_t data_temp[16];                                                         // 缓冲区
    uint8_t bit = 0;                                                              // 数字位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 == number)                                                         // 这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            data_temp[bit ++] = (number % 10);                                  // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数字转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 参数说明     point_bit       小数点精度
// 返回参数     void
// 使用示例     func_double_to_str(data_buffer, 3.1415, 2);                     // 结果输出 data_buffer = "3.14"
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_double_to_str (char *str, double number, uint8_t point_bit)
{
    //zf_assert(str != NULL);
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[12];                                                          // 整数字符缓冲
    int data_temp_point[9];                                                     // 小数字符缓冲
    uint8_t bit = point_bit;                                                      // 转换精度位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        while(0 != bit)
        {
            *str ++ = (myabs(data_temp[bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            bit --;
        }

        // 小数部分转为字符串
        if(point_bit != 0)
        {
            bit = 0;
            *str ++ = '.';
            if(0 == data_float)
                *str = '0';
            else
            {
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;                                                
                }
                while(0 != bit)
                {
                    *str ++ = (myabs(data_temp_point[bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    bit --;
                }
            }
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 显示32位有符号 (去除整数部分无效的0)
// 参数说明     x               x轴坐标设置 0-127
// 参数说明     y               y轴坐标设置 0-7
// 参数说明     dat             需要显示的变量 数据类型 int32
// 参数说明     num             需要显示的位数 最高10位  不包含正负号
// 返回参数     void
// 使用示例     oled_show_int(0, 0, x, 3);                      // x 可以为 int32 int16 int8 类型
// 备注信息     负数会显示一个 ‘-’号
//-------------------------------------------------------------------------------------------------------------------
void oled_show_int (uint16_t x, uint16_t y, const int32_t dat, uint8_t num)
{
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // 检查一下你的显示调用的函数 自己计算一下哪里超过了屏幕显示范围
    // zf_assert(128 > x);
    // zf_assert(8 > y);

    // zf_assert(0 < num);
    // zf_assert(10 >= num);

    int32_t dat_temp = dat;
    int32_t offset = 1;
    char data_buffer[12];

    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num + 1);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_int_to_str(data_buffer, dat_temp);
    oled_show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 显示32位无符号 (去除整数部分无效的0)
// 参数说明     x               x 轴坐标设置 0-127
// 参数说明     y               y 轴坐标设置 0-7
// 参数说明     dat             需要显示的变量 数据类型 uint32
// 参数说明     num             需要显示的位数 最高10位  不包含正负号
// 返回参数     void
// 使用示例     oled_show_uint(0, 0, x, 3);                     // x 可以为 uint32 uint16 uint8 类型
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_show_uint (uint16_t x,uint16_t y,const uint32_t dat,uint8_t num)
{
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // 检查一下你的显示调用的函数 自己计算一下哪里超过了屏幕显示范围
    // zf_assert(128 > x);
    // zf_assert(8 > y);

    // zf_assert(0 < num);
    // zf_assert(10 >= num);

    uint32_t dat_temp = dat;
    int32_t offset = 1;
    char data_buffer[12];
    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_uint_to_str(data_buffer, dat_temp);
    oled_show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 显示浮点数 (去除整数部分无效的0)
// 参数说明     x               x 轴坐标设置 0-127
// 参数说明     y               y 轴坐标设置 0-7
// 参数说明     dat             需要显示的变量 数据类型 float
// 参数说明     num             整数位显示长度   最高8位
// 参数说明     pointnum        小数位显示长度   最高6位
// 返回参数     void
// 使用示例     oled_show_float(0, 0, x, 2, 3);                 // 显示浮点数   整数显示2位   小数显示三位
// 备注信息     特别注意当发现小数部分显示的值与你写入的值不一样的时候，
//              可能是由于浮点数精度丢失问题导致的，这并不是显示函数的问题，
//              有关问题的详情，请自行百度学习   浮点数精度丢失问题。
//              负数会显示一个 ‘-’号
//-------------------------------------------------------------------------------------------------------------------
void oled_show_float (uint16_t x,uint16_t y,const double dat,uint8_t num,uint8_t pointnum)
{
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // 检查一下你的显示调用的函数 自己计算一下哪里超过了屏幕显示范围
    // zf_assert(128 > x);
    // zf_assert(8 > y);

    // zf_assert(0 < num);
    // zf_assert(8 >= num);
    // zf_assert(0 < pointnum);
    // zf_assert(6 >= pointnum);

    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17];
    memset(data_buffer, 0, 17);
    memset(data_buffer, ' ', num + pointnum + 2);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    oled_show_string(x, y, data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       汉字显示
// @param       x               横坐标 0-127
// @param       y               纵坐标 0-7
// @param       size            取模的时候设置的汉字字体大小，也就是一个汉字占用的点阵长宽为多少个点，取模的时候需要长宽是一样的。
// @param       *chinese_buffer 需要显示的汉字数组
// @param       number          需要显示多少位
// @return      void
// Sample usage:                oled_show_chinese(0, 6, 16, (const uint8 *)oled_16x16_chinese, 4);
// @Note        使用PCtoLCD2002软件取模       阴码、逐行式、顺向       16*16
//-------------------------------------------------------------------------------------------------------------------
void oled_show_chinese(uint16_t x, uint16_t y, uint8_t size,const uint8_t *chinese_buffer, uint8_t number) {
    int16_t i, j, k;

    for (i = 0; i < number; i++) {
        for (j = 0; j < (size / 8); j++) {
            oled_set_coordinate(x + i * size, y + j);
            for (k = 0; k < 16; k++) {
                oled_write_data(*chinese_buffer);
                chinese_buffer++;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       IPS114 显示二值图像 数据每八个点组成一个字节数据
// @param       x               x 轴坐标设置 0-127
// @param       y               y 轴坐标设置 0-7
// @param       *image          图像数组指针
// @param       width           图像实际宽度
// @param       height          图像实际高度
// @param       dis_width       图像显示宽度 参数范围 [0, 128]
// @param       dis_height      图像显示高度 参数范围 [0, 64]
// @return      void
// Sample usage:                oled_show_binary_image(0, 0, ov7725_image_binary[0], OV7725_W, OV7725_H, OV7725_W, OV7725_H);
//-------------------------------------------------------------------------------------------------------------------
void oled_show_binary_image(uint16_t x, uint16_t y, const uint8_t *image,uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height) {
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // 检查一下你的显示调用的函数 自己计算一下哪里超过了屏幕显示范围
    /*zf_assert(x < 128);
    zf_assert(y < 8);*/

    uint32_t i = 0, j = 0, z = 0;
    uint8_t dat;
    uint32_t width_index = 0, height_index = 0;

    dis_height = dis_height - dis_height % 8;
    dis_width = dis_width - dis_width % 8;
    for (j = 0; j < dis_height; j += 8) {
        oled_set_coordinate(x + 0, y + j / 8);
        height_index = j * height / dis_height;
        for (i = 0; i < dis_width; i += 8) {
            width_index = i * width / dis_width / 8;
            for (z = 0; z < 8; z++) {
                dat = 0;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 0) & (0x80 >> z))
                    dat |= 0x01;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 1) & (0x80 >> z))
                    dat |= 0x02;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 2) & (0x80 >> z))
                    dat |= 0x04;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 3) & (0x80 >> z))
                    dat |= 0x08;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 4) & (0x80 >> z))
                    dat |= 0x10;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 5) & (0x80 >> z))
                    dat |= 0x20;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 6) & (0x80 >> z))
                    dat |= 0x40;
                if (*(image + height_index * width / 8 + width_index
                        + width / 8 * 7) & (0x80 >> z))
                    dat |= 0x80;
                oled_write_data(dat);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED初始化函数
// 参数说明     void
// 返回参数     void
// 使用示例     oled_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void oled_init (void)
{
    __HAL_I2C_ENABLE(&hi2c1); //硬件IIC初始化

    HAL_Delay(500); //HAL延时函数
 
	oled_write_command(0xAE); //开显示
	oled_write_command(0x20);	//设置内存寻址模式
 
	oled_write_command(0x10);	//00，水平寻址模式;01，垂直寻址模式;10，页面寻址模式(重置);11，无效
	oled_write_command(0xb0);	//为页面寻址模式设置页面开始地址，0-7
	oled_write_command(0x00); //---设置低列地址
	oled_write_command(0x10); //---设置高列地址
 
	oled_write_command(0xc8);	//设置COM输出扫描方向
	oled_write_command(0x40); //--设置起始行地址
	oled_write_command(0x81); //--set contrast control register
	oled_write_command(0xff); //亮度调节 0x00~0xff
	oled_write_command(0xa1); //--设置段重新映射0到127
	oled_write_command(0xa6); //--设置正常显示
	oled_write_command(0xa8); //--设置复用比(1 ~ 64)
	oled_write_command(0x3F); //
	oled_write_command(0xa4); //0xa4,输出遵循RAM内容;0xa5,Output忽略RAM内容
	oled_write_command(0xd3); //-设置显示抵消
	oled_write_command(0x00); //-not offset
	oled_write_command(0xd5); //--设置显示时钟分频/振荡器频率
	oled_write_command(0xf0); //--设置分率
	oled_write_command(0xd9); //--设置pre-charge时期
	oled_write_command(0x22); //
	oled_write_command(0xda); //--设置com大头针硬件配置
	oled_write_command(0x12);
	oled_write_command(0xdb); //--设置vcomh
	oled_write_command(0x20); //0x20,0.77xVcc
	oled_write_command(0x8d); //--设置DC-DC
	oled_write_command(0x14); //
	oled_write_command(0xaf); //--打开oled面板

    oled_clear();                                                               // 初始清屏
}
