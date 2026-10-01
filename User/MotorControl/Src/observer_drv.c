#include "observer_drv.h"

/*滑模观测器（SMO）核心计算*/
void SMO_Calculate(SMO_STRUCT *p)
{
    // 1. 电流预测（基于电机模型的前向欧拉离散化）
    // 公式：I_fore(k+1) = I_fore(k) + Ts * [ -Rs/Ld * I_fore(k) + (U - E_foreLPF)/Ld ]
    p->IalphaFore += p->Ts * (-p->Rs / p->Ld * p->IalphaFore + (p->Ualpha - p->EalphaForeLPF) / p->Ld);
    p->IbetaFore  += p->Ts * (-p->Rs / p->Ld * p->IbetaFore  + (p->Ubeta  - p->EbetaForeLPF)  / p->Ld);

    // 2. 滑模切换函数（饱和函数，减小抖振）
    // 边界层设定为 ±0.5（实测整定值，54µH电感下稳定余量与收敛速度的折中）
    // 当误差 > 0.5 时，输出 +Gain
    if ((p->IalphaFore - p->Ialpha) > 0.5f)
        p->EalphaFore = p->Gain;
    // 当误差 < -0.5 时，输出 -Gain
    else if ((p->IalphaFore - p->Ialpha) < -0.5f)
        p->EalphaFore = -p->Gain;
    // 在边界层内，输出 = Gain * 误差（线性区）
    else
        p->EalphaFore = p->Gain * (p->IalphaFore - p->Ialpha);

    // β轴同理
    if ((p->IbetaFore - p->Ibeta) > 0.5f)
        p->EbetaFore = p->Gain;
    else if ((p->IbetaFore - p->Ibeta) < -0.5f)
        p->EbetaFore = -p->Gain;
    else
        p->EbetaFore = p->Gain * (p->IbetaFore - p->Ibeta);

    // 3. 反电动势低通滤波（滤除高频切换噪声）
    // 一阶低通滤波：E_filt(k) = α * E_raw(k) + (1-α) * E_filt(k-1)
    p->EalphaForeLPF = p->EalphaFore * p->EabForeLPFFactor + p->EalphaForeLPF * (1 - p->EabForeLPFFactor);
    p->EbetaForeLPF  = p->EbetaFore  * p->EabForeLPFFactor + p->EbetaForeLPF  * (1 - p->EabForeLPFFactor);
}
