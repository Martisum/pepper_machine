#include "main.h"
#include "gpio.h"
#include "menu.h"
#include "oled.h"
#include "stdio.h"

uint32_t cntpage;            // ҳ��
struct page *navigate[32]; // ָ��

enum
{
  KEY_UP = 1,
  KEY_DOWN,
  KEY_ADD,
  KEY_SUB,
  KEY_ENTER,
  KEY_LEAVE,
  KEY_START
} KEY_TYPE;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg������Ӳ˵�����
//  @param      tg              ��Ӽ��ĵ�ַ
//  @param      name            ���� �13λ
//  @param          v                           ��ǰ�����һ���˵���ַ
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_subpage(struct page *tg, char *name, struct page *v)
{
  struct item it;
  uint8_t i;

  it.type = subpage;
  for (i = 0; i < 13 && name[i]; i++)
    it.name[i] = name[i];
  it.name[i] = 0;
  it.chf = 0;
  it.addr = v;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg�����ֵ����
//  @param      tg                                  ��һ���˵�
//  @param      name                            ����
//  @param          v                                           ��ֵ�Ĵ�С
//  @param          changedCallBack             ����ָ�� ִ���޸ĸ�ֵ�ĺ���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_value(struct page *tg, char *name, int *v, int16_t dt, void (*changedCallBack)())
{
  struct item it;
  uint8_t i;

  it.type = value;
  for (i = 0; i < 13 && name[i]; i++)
    it.name[i] = name[i];
  it.name[i] = 0;
  it.dlt = dt;
  it.chf = changedCallBack;
  it.addr = v;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg�����ֵ����
//  @param      tg                                  ��һ���˵�
//  @param      name                            ����
//  @param          v                                           ��ֵ�Ĵ�С
//  @param          changedCallBack             ����ָ�� ִ���޸ĸ�ֵ�ĺ���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_value_uint8(struct page *tg, char *name, uint8_t *v, uint8_t dt, void (*changedCallBack)())
{
  struct item it;
  uint8_t i;

  it.type = value;
  for (i = 0; i < 10 && name[i]; i++)
    it.name[i] = name[i];
  it.name[i] = 0;
  it.dlt = dt;
  it.chf = changedCallBack;
  it.addr = v;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg�����ֵ����
//  @param      tg                                  ��һ���˵�
//  @param      name                            ����
//  @param          v                                           ��ֵ�Ĵ�С
//  @param          changedCallBack             ����ָ�� ִ���޸ĸ�ֵ�ĺ���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_title(struct page *tg, char *name, char length)
{
  struct item it;
  uint8_t i;

  it.type = title;
  it.name[0] = ' ';
  for (i = 1; i < 6; i++)
    it.name[i] = '-';
  for (; i < 6 + length; i++)
    it.name[i] = name[i - 6];
  for (; i < 19; i++)
    it.name[i] = '-';
  it.name[i] = 0;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg����Ӹ���������
//  @param      tg                                  ��һ���˵�
//  @param      name                            ����
//  @param          v                                           ��ֵ�Ĵ�С
//  @param          changedCallBack             ����ָ�� ִ���޸ĸ�ֵ�ĺ���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_value_float(struct page *tg, char *name, float *v, float dt, void (*changedCallBack)())
{
  struct item it;
  uint8_t i;

  it.type = value_float;
  for (i = 0; i < 13 && name[i]; i++)
    it.name[i] = name[i];
  it.name[i] = 0;
  it.dlt_float = dt;
  it.chf = changedCallBack;
  it.addr = v;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg�����int32������
//  @param      tg                                  ��һ���˵�
//  @param      name                            ����
//  @param          v                                           ��ֵ�Ĵ�С
//  @param          changedCallBack             ����ָ�� ִ���޸ĸ�ֵ�ĺ���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_value_int32(struct page *tg, char *name, int32_t *v, int32_t dt, void (*changedCallBack)())
{
  struct item it;
  uint8_t i;

  it.type = value_int32;
  for (i = 0; i < 13 && name[i]; i++)
    it.name[i] = name[i];
  it.name[i] = 0;
  it.dlt = dt;
  it.chf = changedCallBack;
  it.addr = v;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg����ӿ�������
//  @param      tg                  ��һ���˵�
//  @param      name            ����
//  @param          v                           Ϊ1 0��ֵ 1Ϊon 0Ϊoff
//  @param          operate             ִ�п��ز����ĺ���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_switc(struct page *tg, char *name, int16_t *v, void (*operate)())
{
  struct item it;
  uint8_t i;

  it.type = switc;

  for (i = 0; i < 18 && name[i]; i++)
    it.name[i] = name[i];
  it.name[i] = 0;
  it.dlt = 0;
  it.chf = operate;
  it.addr = v;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��tg����Ӻ�������
//  @param      tg                  ��һ���˵�
//  @param      name            ����
//  @param          v                           ִ�иò�������������
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void add_func(struct page *tg, char *name, void (*v)())
{
  struct item it;
  uint8_t i;

  it.type = func;
  for (i = 0; i < 20 && name[i]; i++)
    it.name[i] = name[i];
  it.name[i] = 0;
  it.chf = 0;
  it.addr = v;
  tg->itemlist[(int)tg->count++] = it;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʾ���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void OLED_BeginUpdate(void)
{
  oled_clear();
  oled_show_string(0, navigate[cntpage]->pos - navigate[cntpage]->rpos, "-");
}

void OLED_EndUpdate(void)
{
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ӡ��� �˵�ɨ��
//  @param      full_update                 0 �� 1 ѡ��ȫ����ӡ����ֻˢ�±仯����
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuRender(char full_update)
{
  if (full_update)
  {
    OLED_BeginUpdate();

    int tmp = 0;
    for (int i = navigate[cntpage]->rpos; i < navigate[cntpage]->rpos + 8 && i < navigate[cntpage]->count; i++)
    {

      oled_show_string(6, i - navigate[cntpage]->rpos, navigate[cntpage]->itemlist[i].name);

      switch (navigate[cntpage]->itemlist[i].type)
      {
      case title:
        break;
      case subpage:
        break;
      case switc:
        tmp = *((int16_t *)navigate[cntpage]->itemlist[i].addr);
        if (tmp)
          oled_show_string(18 * 6, i - navigate[cntpage]->rpos, "On");
        else
          oled_show_string(18 * 6, i - navigate[cntpage]->rpos, "Off");
        break;
      case func:
        break;
      case value:
      {
        if (!navigate[cntpage]->dymantic_page)
        {
          tmp = *((int16_t *)navigate[cntpage]->itemlist[i].addr);
          oled_show_int((uint8_t)14 * 6, (uint8_t)(i - navigate[cntpage]->rpos), (int16_t)tmp, 6);
        }
        break;
      }
      case value_float:
      {
        if (!navigate[cntpage]->dymantic_page)
        {
          float temp_float = (*((float *)navigate[cntpage]->itemlist[i].addr));
          oled_show_float(14 * 6, i - navigate[cntpage]->rpos, temp_float, 2, 3);
        }
        break;
      }
      default:
        break;
      }
    }

    if (!navigate[cntpage]->dymantic_page)
      oled_show_string(0, navigate[cntpage]->pos - navigate[cntpage]->rpos, "-");

    OLED_EndUpdate();
  }
  else
  {
    if (navigate[cntpage]->dymantic_page) // ֻ���¶�̬ҳ��ֻ����value���͵������ʾ���
    {
      for (int i = navigate[cntpage]->rpos; i < navigate[cntpage]->rpos + 8 && i < navigate[cntpage]->count; i++)
      {
        switch (navigate[cntpage]->itemlist[i].type)
        {
        case value:
        {
          int16_t tmpv = *((int16_t *)navigate[cntpage]->itemlist[i].addr);
          oled_show_int(15 * 6, i - navigate[cntpage]->rpos, tmpv, 6);
          break;
        }
        case value_int32:
        {
          int32_t tmpv = *((int32_t *)navigate[cntpage]->itemlist[i].addr);
          oled_show_int(15 * 3, i - navigate[cntpage]->rpos, tmpv, 6);
          break;
        }
        case value_float:
        {
          float tmpv = *((float *)navigate[cntpage]->itemlist[i].addr);
          oled_show_float(14 * 6, i - navigate[cntpage]->rpos, tmpv, 2, 3);
          break;
        }
        default:
          break;
        }
      }
      // pit_ms_init(TIM1_PIT, 10);
    }
  }
}

void MenuInit(struct page *mainpage)
{
  navigate[0] = mainpage;

  MenuRender(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ִ�а�������
//  @param      cmd                                     �������� 0 Ϊ�޲���
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuCmd(char cmd)
{

  if (cmd)
  {
    switch (cmd)
    {
    case KEY_UP:
    {
      if (navigate[cntpage]->dymantic_page)
      {
        if (navigate[cntpage]->rpos > 0)
          navigate[cntpage]->rpos--;
      }
      else
      {
        if (navigate[cntpage]->pos > 0)
        {
          navigate[cntpage]->pos--;
          if (navigate[cntpage]->rpos > navigate[cntpage]->pos)
            navigate[cntpage]->rpos--;
        }
        else
        {
          navigate[cntpage]->pos = navigate[cntpage]->count - 1;
          if (navigate[cntpage]->count > 8)
            navigate[cntpage]->rpos = navigate[cntpage]->count - 8;
        }
      }

      break;
    }
    case KEY_DOWN:
    {
      if (navigate[cntpage]->dymantic_page)
      {
        if (navigate[cntpage]->rpos + 8 < navigate[cntpage]->count)
          navigate[cntpage]->rpos++;
      }
      else
      {
        if (navigate[cntpage]->pos < navigate[cntpage]->count - 1)
        {
          navigate[cntpage]->pos++;
          if (navigate[cntpage]->rpos < navigate[cntpage]->pos - 7)
            navigate[cntpage]->rpos++;
        }
        else
        {
          navigate[cntpage]->pos = 0;
          navigate[cntpage]->rpos = 0;
        }
      }

      break;
    }
    case KEY_ADD:
    {
      if (navigate[cntpage]->dymantic_page)
        return;

      if (navigate[cntpage]->itemlist[navigate[cntpage]->pos].type == value)
      {
        *((int16_t *)navigate[cntpage]->itemlist[navigate[cntpage]->pos].addr) += navigate[cntpage]->itemlist[navigate[cntpage]->pos].dlt;

        void (*func)() = navigate[cntpage]->itemlist[navigate[cntpage]->pos].chf;
        if (func)
          func();
      }
      if (navigate[cntpage]->itemlist[navigate[cntpage]->pos].type == value_float)
      {
        *((float *)navigate[cntpage]->itemlist[navigate[cntpage]->pos].addr) += navigate[cntpage]->itemlist[navigate[cntpage]->pos].dlt_float;

        void (*func)() = navigate[cntpage]->itemlist[navigate[cntpage]->pos].chf;
        if (func)
          func();
      }
      break;
    }
    case KEY_SUB:
    {
      if (navigate[cntpage]->dymantic_page)
        return;

      if (navigate[cntpage]->itemlist[navigate[cntpage]->pos].type == value)
      {
        *((int16_t *)navigate[cntpage]->itemlist[navigate[cntpage]->pos].addr) -= navigate[cntpage]->itemlist[navigate[cntpage]->pos].dlt;

        void (*func)() = navigate[cntpage]->itemlist[navigate[cntpage]->pos].chf;
        if (func)
          func();
      }
      if (navigate[cntpage]->itemlist[navigate[cntpage]->pos].type == value_float)
      {
        *((float *)navigate[cntpage]->itemlist[navigate[cntpage]->pos].addr) -= navigate[cntpage]->itemlist[navigate[cntpage]->pos].dlt_float;

        void (*func)() = navigate[cntpage]->itemlist[navigate[cntpage]->pos].chf;
        if (func)
          func();
      }
      break;
    }
    case KEY_ENTER:
    {
      if (navigate[cntpage]->itemlist[navigate[cntpage]->pos].type == subpage)
      {
        if (cntpage < 32)
        {
          navigate[cntpage + 1] = navigate[cntpage]->itemlist[navigate[cntpage]->pos].addr;
          cntpage++;
        }
      }
      else if (navigate[cntpage]->itemlist[navigate[cntpage]->pos].type == func)
      {
        void (*func)() = navigate[cntpage]->itemlist[navigate[cntpage]->pos].addr;
        if (func)
          func();
        MenuRender(0);
        return;
      }
      else if (navigate[cntpage]->itemlist[navigate[cntpage]->pos].type == switc)
      {
        int16_t *value = (int16_t *)navigate[cntpage]->itemlist[navigate[cntpage]->pos].addr;
        void (*func)() = navigate[cntpage]->itemlist[navigate[cntpage]->pos].chf;
        if (value)
        {
          if (*value)
            *value = 0;
          else
            *value = 1;
        }
        if (func)
          func();
      }
      break;
    }
    case KEY_LEAVE:
    {
      if (cntpage != 0)
      {
        cntpage--;
      }
      break;
    }
    case KEY_START:
    {
      //                for(uint8 k = 0;k<50;k++)
      //                {
      //                    int i = -1;
      //                    uint8_t start_buf_M[BUFSIZ]={0};
      //                    start_buf_M[++i] ='s';
      ////                    start_buf_M[++i] =dir_forkroad + '0';
      ////                    start_buf_M[++i] =dir_ku + '0';
      ////                    start_buf_M[++i] ='\0';
      //                    wireless_ch573_send_buff((uint8 *)start_buf_M, 1);
      ////                    wireless_ch573_send_uint8('s');
      ////                    wireless_ch573_send_uint8(dir_forkroad);
      ////                    wireless_ch573_send_uint8(dir_ku);
      ////                    wireless_ch573_send_uint8('e');
      //                }

      break;
    }
    default:
      break;
    }
    MenuRender(1);
  }
  else
    MenuRender(0);
}

uint8_t key_scan(void)
{
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
  {
    HAL_Delay(KEY_DelayTime);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
      printf("KEY_ENTER\n");
      return KEY_ENTER;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET)
  {
    HAL_Delay(KEY_DelayTime);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
      printf("KEY_ADD\n");
      return KEY_ADD;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
  {
    HAL_Delay(KEY_DelayTime);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
      printf("KEY_LEAVE\n");
      return KEY_LEAVE;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
  {
    HAL_Delay(KEY_DelayTime);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
      printf("KEY_UP\n");
      return KEY_UP;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
  {
    HAL_Delay(KEY_DelayTime);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      printf("KEY_LEAVE\n");
      return KEY_LEAVE;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
  {
    HAL_Delay(KEY_DelayTime);
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
      printf("KEY_DOWN\n");
      return KEY_DOWN;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
  {
    HAL_Delay(KEY_DelayTime);
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
    {
      printf("KEY_SUB\n");
      return KEY_SUB;
    }
  }
  return 0;
}
