#include "deadtime_comp.h"
#include "motor_publicdata.h"

void Deadtime_Comp_Init(DEADTIME_COMP_STRUCT *p, float comp_ticks, float zero_band)
{
    p->Iu = 0.0f;
    p->Iv = 0.0f;
    p->Iw = 0.0f;

    p->CompTicks = comp_ticks;
    p->CompGain = DT_COMP_GAIN_DEFAULT;
    p->ZeroBand = zero_band;

    p->SignU = 0;
    p->SignV = 0;
    p->SignW = 0;
}

static int Current_Sign(float i, float band)
{
    if (i > band)  return -1;
    if (i < -band) return 1;
    return 0;
}

void Deadtime_Comp_Calculate(DEADTIME_COMP_STRUCT *p, float iu, float iv, float iw)
{
    p->Iu = iu;
    p->Iv = iv;
    p->Iw = iw;

    p->SignU = Current_Sign(iu, p->ZeroBand);
    p->SignV = Current_Sign(iv, p->ZeroBand);
    p->SignW = Current_Sign(iw, p->ZeroBand);
}

void Deadtime_Comp_Apply(DEADTIME_COMP_STRUCT *p, uint16_t *duty_u, uint16_t *duty_v, uint16_t *duty_w, uint16_t pwm_cycle)
{
    float comp_u, comp_v, comp_w;
    float temp_u, temp_v, temp_w;

    comp_u = (float)p->SignU * p->CompTicks * p->CompGain;
    comp_v = (float)p->SignV * p->CompTicks * p->CompGain;
    comp_w = (float)p->SignW * p->CompTicks * p->CompGain;

    temp_u = (float)(*duty_u) + comp_u;
    temp_v = (float)(*duty_v) + comp_v;
    temp_w = (float)(*duty_w) + comp_w;

    // 限幅处理，防止溢出
    if (temp_u > (float)pwm_cycle) temp_u = (float)pwm_cycle;
    if (temp_u < 0.0f)             temp_u = 0.0f;

    if (temp_v > (float)pwm_cycle) temp_v = (float)pwm_cycle;
    if (temp_v < 0.0f)             temp_v = 0.0f;

    if (temp_w > (float)pwm_cycle) temp_w = (float)pwm_cycle;
    if (temp_w < 0.0f)             temp_w = 0.0f;

    *duty_u = (uint16_t)temp_u;
    *duty_v = (uint16_t)temp_v;
    *duty_w = (uint16_t)temp_w;
}
