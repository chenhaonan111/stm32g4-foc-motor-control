#ifndef __EANGLE_DRV_H
#define __EANGLE_DRV_H

#include "main.h"

/* 电角度结构体:阶段3 主要用发生器(Ts/ElectricalAngleSpdSet/ElectricalAngleSetPU);
   编码器相关成员(EncoderVal/CalibOffset 等)留给阶段4 有感闭环使用,这里先一并定义。 */
typedef struct
{
    uint8_t  Dir;                        /* 方向标志(CCW=1正转 / CW=0反转) */
    uint8_t  PolePairs;                  /* 极对数 */
    int32_t  EncoderVal;                 /* 编码器原始计数(阶段4用) */
    int32_t  EncoderValMax;              /* 编码器一圈最大计数值(阶段4用) */
    int32_t  EncoderValChange;           /* 编码器变化量(阶段4用) */
    uint16_t CalibFlag;                  /* 校准完成标志(阶段4用) */
    int32_t  CalibOffset;                /* 转子零位偏移(阶段4用) */
    float    Ts;                         /* 控制周期(秒),=1/控制频率 */
    float    ElectricalAnglePU;          /* 当前电角度标幺值(编码器算,阶段4用) */
    float    ElectricalAngleSpdSet;      /* 设定的电角速度(单位:电RPM) */
    float    ElectricalAngleSetPU;       /* 设定电角度标幺值(发生器累加输出,0~1) */
} E_ANGLE_STRUCT;

/* 电角度发生器:按设定电角速度积分累加电角度(开环强拖用) */
void Electrical_Angle_Generator(E_ANGLE_STRUCT *p);
void Calculate_Encoder_Data(E_ANGLE_STRUCT *p);

#endif
