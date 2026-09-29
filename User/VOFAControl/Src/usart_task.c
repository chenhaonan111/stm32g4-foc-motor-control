#include "usart_task.h"
#include "usart.h"
#include "key_drv.h"
#include "global_control.h"
#include "motor_publicdata.h"

TXDATA TxData;
volatile uint16_t UsartTaskId = 5;
volatile uint16_t UsartTaskTim = 0;

void Usart_Task(void)
{
    switch(UsartTaskId)
    {
        case 5: 
        {
            TxData.u8tail[0] = 0x00;
            TxData.u8tail[1] = 0x00;
            TxData.u8tail[2] = 0x80;
            TxData.u8tail[3] = 0x7f;
            UsartTaskId = 10;
        }
        break;        
        
        case 10:
        {
            if(UsartTaskTim >= 1)
            {
                UsartTaskTim = 0;
                UsartTaskId = 20;
            }
        }
        break;
        
        case 20:
        { 
            if (__HAL_DMA_GET_COUNTER(&hdma_usart1_tx) == 0) 
            {              
                TxData.fdata[0] = MC.Foc.Id;
                TxData.fdata[1] = MC.IdPid.Ref;
                TxData.fdata[2] = 0;
                TxData.fdata[3] = 0;

                __HAL_DMA_DISABLE(&hdma_usart1_tx);
                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxData, sizeof(TxData));
                __HAL_DMA_ENABLE(&hdma_usart1_tx);
            }                
            UsartTaskId = 10;        
        }
        break;
        
    default:
      break;            
    }
}




