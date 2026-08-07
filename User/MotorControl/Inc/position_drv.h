#ifndef __POSITION_DRV_H__
#define __POSITION_DRV_H__

#include "main.h"

typedef struct
{
    uint16_t   PosCalculateCnt;            // 位置计算计数    
    uint16_t   ElectricalValMax;           // 电角度最大值
    int32_t    ElectricalPosThis;          // 本次电角位置
    int32_t    ElectricalPosLast;          // 上次电角位置
    int32_t    ElectricalPosChange;        // 单位时间位移
    int32_t    ElectricalPosSum;           // 绝对电角位置
    int32_t    MechanicalPosRaw;           // 绝对机械位置
    int32_t    MechanicalPosSet;           // 目标机械位置
}POSITION_STRUCT;

void Calculate_Position(POSITION_STRUCT *p);

#endif

