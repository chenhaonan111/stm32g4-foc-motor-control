#include "key_task.h"
#include "key_drv.h"

volatile uint16_t KeyTaskId  = 10;
volatile uint16_t KeyTaskTim = 0;

/* 由 TIM2 中断每 100us 调用一次, 软件分频到 10ms 扫描一次按键 */
void Key_Task(void)
{
    switch (KeyTaskId)
    {
        /* 状态10: 等待 10ms 到达 (100 × 100us) */
        case 10:
        {
            if (KeyTaskTim >= 100)
            {
                KeyTaskTim = 0;
                KeyTaskId = 20;
            }
        }
        break;

        /* 状态20: 执行一次按键扫描, 然后回到等待 */
        case 20:
        {
            Key_Scan();
            KeyTaskId = 10;
        }
        break;

        default:
            break;
    }
}
