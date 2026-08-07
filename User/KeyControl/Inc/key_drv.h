#ifndef __KEY_DRV_H__
#define __KEY_DRV_H__

#include "main.h"

/* 按键引脚: KEY1 -> PC9, KEY2 -> PB12, 低电平有效(按下读到0) */
#define KEY1      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)
#define KEY2      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)

typedef struct button
{
    uint8_t  level;     /* 当前电平: 0=按下, 1=释放   */
    uint8_t  status;    /* 状态机状态: 0/1/2          */
    uint16_t scan_cnt;  /* 按下持续计数器             */
} button_t;

extern uint8_t KeyNum;  /* 键值: 0=无 1=KEY1短按 2=KEY1长按 3=KEY2短按 4=KEY2长按 */

void Key_Scan(void);

#endif /* __KEY_DRV_H__ */
