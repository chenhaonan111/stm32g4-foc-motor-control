#ifndef __MATH_DRV_H
#define __MATH_DRV_H

#include "main.h"

/* 根据电角度标幺值(0~1 对应 0~360°)查表计算 sin/cos */
void Calculate_Sin_Cos(float angle, float *sinval, float *cosval);

/* 幅值限幅:把 *input 限制在 [min, max] */
void Amplitude_Limit(float *input, float min, float max);

#endif
