#ifndef __DEADTIME_COMP__
#define __DEADTIME_COMP__

#include "main.h"

typedef struct
{
    float Iu;               // U相电流，A
    float Iv;               // V相电流，A
    float Iw;               // W相电流，A

    float CompTicks;        // 补偿量（以PWM计数tick为单位）
    float CompGain;         // 补偿增益，工程标定用
    float ZeroBand;         // 电流过零滞环阈值，A

    int   SignU;            // U相电流极性
    int   SignV;            // V相电流极性
    int   SignW;            // W相电流极性
} DEADTIME_COMP_STRUCT;

void Deadtime_Comp_Init(DEADTIME_COMP_STRUCT *p, float comp_ticks, float zero_band);
void Deadtime_Comp_Calculate(DEADTIME_COMP_STRUCT *p, float iu, float iv, float iw);
void Deadtime_Comp_Apply(DEADTIME_COMP_STRUCT *p, uint16_t *duty_u, uint16_t *duty_v, uint16_t *duty_w, uint16_t pwm_cycle);

#endif

