#include "key_drv.h"

uint8_t   KeyNum = 0;
button_t  button[2] = {0};

/* 读取两个按键引脚电平到 level 成员 */
static void ReadKey(void)
{
    button[0].level = KEY1;
    button[1].level = KEY2;
}

/* 按键状态机, 判定短按/长按 */
void Key_Process(void)
{
    uint8_t index = 0;

    for (index = 0; index < 2; index++)
    {
        switch (button[index].status)
        {
            /* 状态0: 等待按下 */
            case 0:
                if (button[index].level == 0)
                {
                    button[index].scan_cnt = 0;
                    button[index].status = 1;
                }
                break;

            /* 状态1: 确认按下, 计数并判定短按/长按 */
            case 1:
                if (button[index].level == 0)
                {
                    button[index].scan_cnt++;
                    if (button[index].scan_cnt >= 50)   /* 达到阈值 -> 长按 */
                    {
                        if (index == 0)
                        {
                            KeyNum = 2;
                            button[index].status = 2;
                        }
                        if (index == 1)
                        {
                            KeyNum = 4;
                            button[index].status = 2;
                        }
                    }
                }
                else                                    /* 未达阈值即释放 -> 短按 */
                {
                    if (button[index].scan_cnt <= 50)
                    {
                        if (index == 0)
                        {
                            KeyNum = 1;
                            button[index].status = 0;
                        }
                        if (index == 1)
                        {
                            KeyNum = 3;
                            button[index].status = 0;
                        }
                    }
                }
                break;

            /* 状态2: 等待释放 */
            case 2:
                if (button[index].level == 1)
                {
                    button[index].status = 0;
                }
                break;
        }
    }
}

/* 扫描总入口: 先读电平, 再跑状态机 */
void Key_Scan(void)
{
    ReadKey();
    Key_Process();
}
