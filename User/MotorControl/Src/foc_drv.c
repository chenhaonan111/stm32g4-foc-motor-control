#include "foc_drv.h"

/**
 * Clark 变换:三相静止坐标系(a,b,c)→ 两相静止坐标系(α,β)。阶段4 电流环用。
 * 等幅值变换,利用 Iu+Iv+Iw=0 省去 Iw:
 *   Iα = Iu
 *   Iβ = (Iu + 2*Iv) / √3
 */
void Clark_Transform(FOC_STRUCT *p)
{
    p->Ialpha = p->Iu;
    p->Ibeta  = (p->Iu * 0.57735027f) + (p->Iv * 1.1547004f);
}

/**
 * Park 变换:两相静止(α,β)→ 旋转坐标系(d,q)。阶段4 电流环用。
 *   Id =  Iα*cos + Iβ*sin
 *   Iq = -Iα*sin + Iβ*cos
 */
void Park_Transform(FOC_STRUCT *p)
{
    p->Id = (p->Ialpha * p->CosVal) + (p->Ibeta * p->SinVal);
    p->Iq = (-p->Ialpha * p->SinVal) + (p->Ibeta * p->CosVal);
}

/**
 * 反 Park 变换:旋转坐标系(d,q)电压 → 两相静止(α,β)电压。开环强拖用。
 *   Uα = Ud*cos - Uq*sin
 *   Uβ = Uq*cos + Ud*sin
 * 输入 Ud/Uq、SinVal/CosVal,输出 Ualpha/Ubeta。
 */
void IPark_Transform(FOC_STRUCT *p)
{
    p->Ualpha = p->Ud * p->CosVal - p->Uq * p->SinVal;
    p->Ubeta  = p->Uq * p->CosVal + p->Ud * p->SinVal;
}

/**
 * SVPWM(空间矢量脉宽调制):根据 Uα/Uβ 和母线电压 Ubus,计算三相占空比 DutyCycleA/B/C。
 * 算法步骤:
 *   1. 由 Uα/Uβ 判断扇区 N(1~6)
 *   2. 计算相邻矢量作用时间 T1/T2
 *   3. 过调制处理(T1+T2 不能超过 PwmLimit)
 *   4. 计算三相比较值 Ta/Tb/Tc 并按扇区映射到 A/B/C 占空比
 * PwmCycle=8500(=2×ARR),算出的 DutyCycle 范围 0~4250,正好对应 TIM1 的 ARR,可直接 SET_COMPARE。
 */
void Calculate_SVPWM(FOC_STRUCT *p)
{
    float U1, U2, U3 = 0;
    float X, Y, Z = 0;
    float T1, T2, T1Temp, T2Temp = 0;
    uint8_t A, B, C, N = 0;
    uint16_t Ta, Tb, Tc = 0;

    /* 1. 扇区判断 */
    U1 = p->Ubeta;                                  /* U1 = Uβ */
    U2 = (0.866f * p->Ualpha) - (0.5f * p->Ubeta);  /* U2 = (√3/2)Uα - (1/2)Uβ */
    U3 = (-0.866f * p->Ualpha) - (0.5f * p->Ubeta); /* U3 = -(√3/2)Uα - (1/2)Uβ */

    if (U1 > 0) { A = 1; } else { A = 0; }
    if (U2 > 0) { B = 1; } else { B = 0; }
    if (U3 > 0) { C = 1; } else { C = 0; }
    N = 4 * C + 2 * B + A;                          /* 扇区号 1~6 */

    /* 2. 相邻矢量作用时间(未归一化) */
    X = (1.732f * p->PwmCycle * p->Ubeta) / p->Ubus;
    Y = (1.5f * p->Ualpha * p->PwmCycle + 0.866f * p->Ubeta * p->PwmCycle) / p->Ubus;
    Z = (-1.5f * p->Ualpha * p->PwmCycle + 0.866f * p->Ubeta * p->PwmCycle) / p->Ubus;

    /* 3. 按扇区取 T1/T2 */
    switch (N)
    {
        case 3: { T1 = -Z; T2 =  X; } break;
        case 1: { T1 =  Z; T2 =  Y; } break;
        case 5: { T1 =  X; T2 = -Y; } break;
        case 4: { T1 = -X; T2 =  Z; } break;
        case 6: { T1 = -Y; T2 = -Z; } break;
        case 2: { T1 =  Y; T2 = -X; } break;
        default: { T1 = 0; T2 = 0; } break;
    }

    /* 4. 过调制处理(总作用时间不超过 PwmLimit) */
    T1Temp = T1;
    T2Temp = T2;
    if (T1 + T2 > p->PwmLimit)
    {
        T1 = p->PwmLimit * T1Temp / (T1Temp + T2Temp);
        T2 = p->PwmLimit * T2Temp / (T1Temp + T2Temp);
    }

    /* 5. 三相比较值 */
    Ta = (p->PwmCycle - T1 - T2) * 0.25f;
    Tb = Ta + T1 * 0.5f;
    Tc = Tb + T2 * 0.5f;

    /* 6. 按扇区映射到 A/B/C 占空比 */
    switch (N)
    {
        case 3: p->DutyCycleA = Ta; p->DutyCycleB = Tb; p->DutyCycleC = Tc; break;
        case 1: p->DutyCycleA = Tb; p->DutyCycleB = Ta; p->DutyCycleC = Tc; break;
        case 5: p->DutyCycleA = Tc; p->DutyCycleB = Ta; p->DutyCycleC = Tb; break;
        case 4: p->DutyCycleA = Tc; p->DutyCycleB = Tb; p->DutyCycleC = Ta; break;
        case 6: p->DutyCycleA = Tb; p->DutyCycleB = Tc; p->DutyCycleC = Ta; break;
        case 2: p->DutyCycleA = Ta; p->DutyCycleB = Tc; p->DutyCycleC = Tb; break;
        default:p->DutyCycleA = Ta; p->DutyCycleB = Tb; p->DutyCycleC = Tc; break;
    }
}
