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
    I2C���ߴ������ݺ�����
                addr  :    Ҫд��ĵ�ַ��OLED�ĵ�ַһ��Ϊ0x40;ָ���ַΪ0x00��
                data  :    Ҫд�������
***************************************************/
void HAL_I2C_WriteByte(uint8_t addr,uint8_t data)
{
	uint8_t TxData[2] = {addr,data};
	HAL_I2C_Master_Transmit(&hi2c1,0X78,(uint8_t*)TxData,2,10);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     д����
// ����˵��     cmd             ����
// ���ز���     void
// ʹ��ʾ��     oled_write_command(0xb0 + y);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void oled_write_command (uint8_t IIC_Command)
{
	HAL_I2C_WriteByte(0x00, IIC_Command);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     д8λ����
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     oled_write_data(color);
// ��ע��Ϣ     �ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
void oled_write_data (uint8_t IIC_Data)
{
    HAL_I2C_WriteByte(0x40, IIC_Data);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     OLED ��������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     oled_clear();
// ��ע��Ϣ
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
// �������     OLED��ʾ��������
// ����˵��     x               x����������0-127
// ����˵��     y               y����������0-7
// ���ز���     void
// ʹ��ʾ��     oled_set_coordinate(x, y);
// ��ע��Ϣ     �ڲ�ʹ���û��������
//-------------------------------------------------------------------------------------------------------------------
void oled_set_coordinate (uint8_t x, uint8_t y)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
//    zf_assert(128 > x);
//    zf_assert(8 > y);

    oled_write_command(0xb0 + y);
    oled_write_command(((x & 0xf0) >> 4) | 0x10);
    oled_write_command((x & 0x0f) | 0x00);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     OLED ��Ļ��亯��
// ����˵��     color           �����ɫѡ��(0x00 or 0xff)
// ���ز���     void
// ʹ��ʾ��     oled_full(0x00);
// ��ע��Ϣ
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
// �������     ������ʾ����
// ����˵��     dir             ��ʾ����  ���� zf_device_oled.h �� oled_dir_enum ö���嶨��
// ���ز���     void
// ʹ��ʾ��     oled_set_dir(OLED_CROSSWISE);
// ��ע��Ϣ     �������ֻ���ڳ�ʼ����Ļ֮ǰ���ò���Ч
//-------------------------------------------------------------------------------------------------------------------
void oled_set_dir (oled_dir_enum dir)
{
    oled_display_dir = dir;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ����
// ����˵��     dir             ��ʾ����  ���� zf_device_oled.h �� oled_font_size_enum ö���嶨��
// ���ز���     void
// ʹ��ʾ��     oled_set_font(OLED_8x16_FONT);
// ��ע��Ϣ     ���������ʱ�������� ���ú���Ч ������ʾ�����µ������С
//-------------------------------------------------------------------------------------------------------------------
void oled_set_font (oled_font_size_enum font)
{
    oled_display_font = font;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���㺯��
// ����˵��     x               x ���������� 0-127
// ����˵��     y               y ���������� 0-7
// ����˵��     color           8 ��������
// ���ز���     void
// ʹ��ʾ��     oled_draw_point(0, 0, 1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void oled_draw_point (uint16_t x, uint16_t y, const uint8_t color)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
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
// �������     OLED ��ʾ�ַ���
// ����˵��     x               x ���������� 0-127
// ����˵��     y               y ���������� 0-7
// ����˵��     ch[]            �ַ���
// ���ز���     void
// ʹ��ʾ��     oled_show_string(0, 0, "SEEKFREE");
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void oled_show_string (uint16_t x, uint16_t y, const char ch[])
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
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
                // �ݲ�֧��
            }break;
        }
    }
    //OLED_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ת�ַ��� ���ݷ�Χ�� [-32768,32767]
// ����˵��     *str            �ַ���ָ��
// ����˵��     number          ���������
// ���ز���     void
// ʹ��ʾ��     func_int_to_str(data_buffer, -300);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void func_int_to_str (char *str, int32_t number)
{
    //zf_assert(str != NULL);
    uint8_t data_temp[16];                                                        // ������
    uint8_t bit = 0;                                                              // ����λ��
    int32_t number_temp = 0;

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 > number)                                                          // ����
        {
            *str ++ = '-';
            number = -number;
        }
        else if(0 == number)                                                    // �������Ǹ� 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // ѭ��ֱ����ֵ����
        {
            number_temp = number % 10;
            data_temp[bit ++] = myabs(number_temp);                          // ������ֵ��ȡ����
            number /= 10;                                                       // ��������ȡ�ĸ�λ��
        }
        while(0 != bit)                                                         // ��ȡ�����ָ����ݼ�����
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // �����ִӵ��������е���ȡ�� �����������ַ���
            bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ת�ַ��� ���ݷ�Χ�� [0,65535]
// ����˵��     *str            �ַ���ָ��
// ����˵��     number          ���������
// ���ز���     void
// ʹ��ʾ��     func_uint_to_str(data_buffer, 300);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void func_uint_to_str (char *str, uint32_t number)
{
    //zf_assert(str != NULL);
    int8_t data_temp[16];                                                         // ������
    uint8_t bit = 0;                                                              // ����λ��

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 == number)                                                         // ���Ǹ� 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // ѭ��ֱ����ֵ����
        {
            data_temp[bit ++] = (number % 10);                                  // ������ֵ��ȡ����
            number /= 10;                                                       // ��������ȡ�ĸ�λ��
        }
        while(0 != bit)                                                         // ��ȡ�����ָ����ݼ�����
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // �����ִӵ��������е���ȡ�� �����������ַ���
            bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ת�ַ���
// ����˵��     *str            �ַ���ָ��
// ����˵��     number          ���������
// ����˵��     point_bit       С���㾫��
// ���ز���     void
// ʹ��ʾ��     func_double_to_str(data_buffer, 3.1415, 2);                     // ������ data_buffer = "3.14"
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void func_double_to_str (char *str, double number, uint8_t point_bit)
{
    //zf_assert(str != NULL);
    int data_int = 0;                                                           // ��������
    int data_float = 0.0;                                                       // С������
    int data_temp[12];                                                          // �����ַ�����
    int data_temp_point[9];                                                     // С���ַ�����
    uint8_t bit = point_bit;                                                      // ת������λ��

    do
    {
        if(NULL == str)
        {
            break;
        }

        // ��ȡ��������
        data_int = (int)number;                                                 // ֱ��ǿ��ת��Ϊ int
        if(0 > number)                                                          // �ж�Դ�������������Ǹ���
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // ����Ǹ� 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // ��ȡС������
        number = number - data_int;                                             // ��ȥ�������ּ���
        while(bit --)
        {
            number = number * 10;                                               // ����Ҫ��С��λ����ȡ����������
        }
        data_float = (int)number;                                               // ��ȡ�ⲿ����ֵ

        // ��������תΪ�ַ���
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // ���������ֵ���д���ַ�������
            data_int /= 10;
        }while(0 != data_int);
        while(0 != bit)
        {
            *str ++ = (myabs(data_temp[bit - 1]) + 0x30);                    // �ٵ��򽫵������ֵд���ַ��� �õ�������ֵ
            bit --;
        }

        // С������תΪ�ַ���
        if(point_bit != 0)
        {
            bit = 0;
            *str ++ = '.';
            if(0 == data_float)
                *str = '0';
            else
            {
                while(0 != point_bit)                                           // �ж���Чλ��
                {
                    data_temp_point[bit ++] = data_float % 10;                  // ����д���ַ�������
                    data_float /= 10;
                    point_bit --;                                                
                }
                while(0 != bit)
                {
                    *str ++ = (myabs(data_temp_point[bit - 1]) + 0x30);      // �ٵ��򽫵������ֵд���ַ��� �õ�������ֵ
                    bit --;
                }
            }
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     OLED ��ʾ32λ�з��� (ȥ������������Ч��0)
// ����˵��     x               x���������� 0-127
// ����˵��     y               y���������� 0-7
// ����˵��     dat             ��Ҫ��ʾ�ı��� �������� int32
// ����˵��     num             ��Ҫ��ʾ��λ�� ���10λ  ������������
// ���ز���     void
// ʹ��ʾ��     oled_show_int(0, 0, x, 3);                      // x ����Ϊ int32 int16 int8 ����
// ��ע��Ϣ     ��������ʾһ�� ��-����
//-------------------------------------------------------------------------------------------------------------------
void oled_show_int (uint16_t x, uint16_t y, const int32_t dat, uint8_t num)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
    // zf_assert(128 > x);
    // zf_assert(8 > y);

    // zf_assert(0 < num);
    // zf_assert(10 >= num);

    int32_t dat_temp = dat;
    int32_t offset = 1;
    char data_buffer[12];

    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num + 1);

    // ��������������ʾ 123 ��ʾ 2 λ��Ӧ����ʾ 23
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
// �������     OLED ��ʾ32λ�޷��� (ȥ������������Ч��0)
// ����˵��     x               x ���������� 0-127
// ����˵��     y               y ���������� 0-7
// ����˵��     dat             ��Ҫ��ʾ�ı��� �������� uint32
// ����˵��     num             ��Ҫ��ʾ��λ�� ���10λ  ������������
// ���ز���     void
// ʹ��ʾ��     oled_show_uint(0, 0, x, 3);                     // x ����Ϊ uint32 uint16 uint8 ����
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void oled_show_uint (uint16_t x,uint16_t y,const uint32_t dat,uint8_t num)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
    // zf_assert(128 > x);
    // zf_assert(8 > y);

    // zf_assert(0 < num);
    // zf_assert(10 >= num);

    uint32_t dat_temp = dat;
    int32_t offset = 1;
    char data_buffer[12];
    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num);

    // ��������������ʾ 123 ��ʾ 2 λ��Ӧ����ʾ 23
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
// �������     OLED ��ʾ������ (ȥ������������Ч��0)
// ����˵��     x               x ���������� 0-127
// ����˵��     y               y ���������� 0-7
// ����˵��     dat             ��Ҫ��ʾ�ı��� �������� float
// ����˵��     num             ����λ��ʾ����   ���8λ
// ����˵��     pointnum        С��λ��ʾ����   ���6λ
// ���ز���     void
// ʹ��ʾ��     oled_show_float(0, 0, x, 2, 3);                 // ��ʾ������   ������ʾ2λ   С����ʾ��λ
// ��ע��Ϣ     �ر�ע�⵱����С��������ʾ��ֵ����д���ֵ��һ����ʱ��
//              ���������ڸ��������ȶ�ʧ���⵼�µģ��Ⲣ������ʾ���������⣬
//              �й���������飬�����аٶ�ѧϰ   ���������ȶ�ʧ���⡣
//              ��������ʾһ�� ��-����
//-------------------------------------------------------------------------------------------------------------------
void oled_show_float (uint16_t x,uint16_t y,const double dat,uint8_t num,uint8_t pointnum)
{
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
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

    // ��������������ʾ 123 ��ʾ 2 λ��Ӧ����ʾ 23
    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    oled_show_string(x, y, data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       ������ʾ
// @param       x               ������ 0-127
// @param       y               ������ 0-7
// @param       size            ȡģ��ʱ�����õĺ��������С��Ҳ����һ������ռ�õĵ��󳤿�Ϊ���ٸ��㣬ȡģ��ʱ����Ҫ������һ���ġ�
// @param       *chinese_buffer ��Ҫ��ʾ�ĺ�������
// @param       number          ��Ҫ��ʾ����λ
// @return      void
// Sample usage:                oled_show_chinese(0, 6, 16, (const uint8 *)oled_16x16_chinese, 4);
// @Note        ʹ��PCtoLCD2002���ȡģ       ���롢����ʽ��˳��       16*16
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
// @brief       IPS114 ��ʾ��ֵͼ�� ����ÿ�˸������һ���ֽ�����
// @param       x               x ���������� 0-127
// @param       y               y ���������� 0-7
// @param       *image          ͼ������ָ��
// @param       width           ͼ��ʵ�ʿ��
// @param       height          ͼ��ʵ�ʸ߶�
// @param       dis_width       ͼ����ʾ��� ������Χ [0, 128]
// @param       dis_height      ͼ����ʾ�߶� ������Χ [0, 64]
// @return      void
// Sample usage:                oled_show_binary_image(0, 0, ov7725_image_binary[0], OV7725_W, OV7725_H, OV7725_W, OV7725_H);
//-------------------------------------------------------------------------------------------------------------------
void oled_show_binary_image(uint16_t x, uint16_t y, const uint8_t *image,uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height) {
    // �������������˶�����Ϣ ������ʾ����λ��������
    // ��ôһ������Ļ��ʾ��ʱ�򳬹���Ļ�ֱ��ʷ�Χ��
    // ���һ�������ʾ���õĺ��� �Լ�����һ�����ﳬ������Ļ��ʾ��Χ
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
// �������     OLED��ʼ������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     oled_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void oled_init (void)
{
    __HAL_I2C_ENABLE(&hi2c1); //Ӳ��IIC��ʼ��

    HAL_Delay(500); //HAL��ʱ����
 
	oled_write_command(0xAE); //����ʾ
	oled_write_command(0x20);	//�����ڴ�Ѱַģʽ
 
	oled_write_command(0x10);	//00��ˮƽѰַģʽ;01����ֱѰַģʽ;10��ҳ��Ѱַģʽ(����);11����Ч
	oled_write_command(0xb0);	//Ϊҳ��Ѱַģʽ����ҳ�濪ʼ��ַ��0-7
	oled_write_command(0x00); //---���õ��е�ַ
	oled_write_command(0x10); //---���ø��е�ַ
 
	oled_write_command(0xc8);	//����COM���ɨ�跽��
	oled_write_command(0x40); //--������ʼ�е�ַ
	oled_write_command(0x81); //--set contrast control register
	oled_write_command(0xff); //���ȵ��� 0x00~0xff
	oled_write_command(0xa1); //--���ö�����ӳ��0��127
	oled_write_command(0xa6); //--����������ʾ
	oled_write_command(0xa8); //--���ø��ñ�(1 ~ 64)
	oled_write_command(0x3F); //
	oled_write_command(0xa4); //0xa4,�����ѭRAM����;0xa5,Output����RAM����
	oled_write_command(0xd3); //-������ʾ����
	oled_write_command(0x00); //-not offset
	oled_write_command(0xd5); //--������ʾʱ�ӷ�Ƶ/����Ƶ��
	oled_write_command(0xf0); //--���÷���
	oled_write_command(0xd9); //--����pre-chargeʱ��
	oled_write_command(0x22); //
	oled_write_command(0xda); //--����com��ͷ��Ӳ������
	oled_write_command(0x12);
	oled_write_command(0xdb); //--����vcomh
	oled_write_command(0x20); //0x20,0.77xVcc
	oled_write_command(0x8d); //--����DC-DC
	oled_write_command(0x14); //
	oled_write_command(0xaf); //--��oled���

    oled_clear();                                                               // ��ʼ����
}
