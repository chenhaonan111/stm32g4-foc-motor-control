#ifndef __USART_TASK_H__
#define __USART_TASK_H__

#include "main.h"

#define CH_COUNT 4

typedef struct
{
    float   fdata[CH_COUNT];
    uint8_t u8tail[4];
} TXDATA;

void Usart_Task(void);

#endif
