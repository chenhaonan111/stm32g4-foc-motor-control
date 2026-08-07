#include "observer_drv.h"

/*滑模观测器（SMO）核心计算*/
void SMO_Calculate(SMO_STRUCT *p)
{
    // 1. 电流预测（基于电机模型的前向欧拉离散化）
    // 公式：I_fore(k+1) = I_fore(k) + Ts * [ -Rs/Ld * I_fore(k) + (U - E_foreLPF)/Ld ]
    p->IalphaFore += p->Ts * (-p->Rs / p->Ld * p->IalphaFore + (p->Ualpha - p->EalphaForeLPF) / p->Ld);
    p->IbetaFore  += p->Ts * (-p->Rs / p->Ld * p->IbetaFore  + (p->Ubeta  - p->EbetaForeLPF)  / p->Ld);

    // 2. 滑模切换函数（饱和函数，减小抖振）
    // 边界层设定为 ±1.0（可根据实际电流误差范围调整）
    // 当误差 > 1.0 时，输出 +Gain
    if ((p->IalphaFore - p->Ialpha) > 1.0f)
        p->EalphaFore = p->Gain;
    // 当误差 < -1.0 时，输出 -Gain
    else if ((p->IalphaFore - p->Ialpha) < -1.0f)
        p->EalphaFore = -p->Gain;
    // 在边界层内，输出 = Gain * 误差（线性区）
    else
        p->EalphaFore = p->Gain * (p->IalphaFore - p->Ialpha);

    // β轴同理
    if ((p->IbetaFore - p->Ibeta) > 1.0f)
        p->EbetaFore = p->Gain;
    else if ((p->IbetaFore - p->Ibeta) < -1.0f)
        p->EbetaFore = -p->Gain;
    else
        p->EbetaFore = p->Gain * (p->IbetaFore - p->Ibeta);

    // 3. 反电动势低通滤波（滤除高频切换噪声）
    // 一阶低通滤波：E_filt(k) = α * E_raw(k) + (1-α) * E_filt(k-1)
    p->EalphaForeLPF = p->EalphaFore * p->EabForeLPFFactor + p->EalphaForeLPF * (1 - p->EabForeLPFFactor);
    p->EbetaForeLPF  = p->EbetaFore  * p->EabForeLPFFactor + p->EbetaForeLPF  * (1 - p->EabForeLPFFactor);
}

/*高频注入（HFI）及信号解析（用于零低速无传感器控制*/
void HFI_Calculate(HFI_STRUCT *p)
{
    /* ================= 第一部分：初始角度辨识（南北极检测） ================= */
    if (p->NSDFlag == 0)
    {
        p->NSDCount++;   // 状态计数器递增

        // 提取d轴高频电流分量：相邻两次Id采样差的一半
        p->IdHigh = (p->Id - p->IdLast) * 0.5f;
        if (p->IdHigh < 0)
        {
            p->IdHigh = -p->IdHigh;   // 取绝对值（幅值）
        }

        // 状态机：根据计数值执行不同操作
        if (p->NSDCount < 10400)
        {
            // 阶段0：等待观测器/滤波器收敛（约400次调用）
            p->IdRef = 5.0f;
        }
        else if (p->NSDCount >= 10400 && p->NSDCount < 10600)
        {
            // 阶段1：施加正向电流脉冲（+5A）
            p->IdRef = 5.0f;
        }
        else if (p->NSDCount >= 10600 && p->NSDCount < 10610)
        {
            // 阶段2：保持正向电流，同时采样高频电流幅值（累加10次）
            p->IdRef = 5.0f;
            p->NSDSum1 += p->IdHigh;
        }
        else if (p->NSDCount >= 10610 && p->NSDCount < 10810)
        {
            // 阶段3：等待电流归零
            p->IdRef = 0.0f;
        }
        else if (p->NSDCount >= 10810 && p->NSDCount < 11010)
        {
            // 阶段4：施加负向电流脉冲（-5A）
            p->IdRef = -5.0f;
        }
        else if (p->NSDCount >= 11010 && p->NSDCount < 11020)
        {
            // 阶段5：保持负向电流，采样高频电流幅值（累加10次）
            p->IdRef = -5.0f;
            p->NSDSum2 += p->IdHigh;
        }
        else if (p->NSDCount == 11020)
        {
            // 阶段6：电流归零
            p->IdRef = 0.0f;
        }
        else if (p->NSDCount == 11021)
        {
            // 阶段7：完成NSD，比较正负脉冲下的高频电流响应幅值
            p->NSDFlag = 1;       // NSD完成标志
            p->NSDCount = 0;      // 计数器复位

            // 若负向电流下的高频幅值大于正向，则NSDOut=1，否则0
            // 该结果用于确定转子N极相对于注入轴的方向
            if (p->NSDSum2 > p->NSDSum1)
            {
                p->NSDOut = 1;
            }
            else
            {
                p->NSDOut = 0;
            }
        }
    }

    /* ================= 第二部分：信号注入与解析（高频响应解调） ================= */
    // 提取d、q轴基频电流分量（相邻两次采样平均，滤除高频成分）
    p->IdBase = (p->Id + p->IdLast) * 0.5f;
    p->IqBase = (p->Iq + p->IqLast) * 0.5f;
    // 更新上次采样值
    p->IdLast = p->Id;
    p->IqLast = p->Iq;

    // 保存上一周期的高频电流分量（用于差分）
    p->IalphaHighLast = p->IalphaHigh;
    p->IbetaHighLast  = p->IbetaHigh;

    // 提取αβ轴高频电流分量：相邻两次采样差的一半
    // 由于高频注入电压频率远高于基频，相邻采样点之差主要反映高频响应
    p->IalphaHigh = (p->Ialpha - p->IalphaLast) * 0.5f;
    p->IbetaHigh  = (p->Ibeta  - p->IbetaLast)  * 0.5f;

    // 更新上次αβ电流采样值
    p->IalphaLast = p->Ialpha;
    p->IbetaLast  = p->Ibeta;

    // 交替计算高频包络信号（用于PLL解调位置误差）
    // 通过Dir标志交替改变差分方向，可消除直流偏移，增强信噪比
    if (p->Dir == 0)
    {
        // 当前周期输出 = 本次高频分量 - 上次高频分量
        p->IalphaOut = p->IalphaHigh - p->IalphaHighLast;
        p->IbetaOut  = p->IbetaHigh  - p->IbetaHighLast;
        p->Dir = 1;   // 翻转方向，下次计算相反差分
    }
    else if (p->Dir == 1)
    {
        // 输出 = 上次高频分量 - 本次高频分量
        p->IalphaOut = p->IalphaHighLast - p->IalphaHigh;
        p->IbetaOut  = p->IbetaHighLast  - p->IbetaHigh;
        p->Dir = 0;
    }
}


/*函数功能: 正交锁相环（QPLL）—— 从正交信号中提取转子角度和角速度*/
void PLL_Calculate(PLL_STRUCT *p)
{
    // 1. 计算角度误差（锁相环鉴相器）
    // 公式：ThetaErr = Dir * (cosθ_hat * Ain + sinθ_hat * Bin)
    // 当Ain、Bin为Ealpha、Ebeta时，该误差近似正比于角度差
    p->ThetaErr = p->Dir * (p->CosVal * p->Ain + p->SinVal * p->Bin);

    // 2. PI控制器（环路滤波器）
    p->PPart = p->Kp * p->ThetaErr;            // 比例项
    p->IPart += p->Ki * p->ThetaErr;           // 积分项累加
    p->WeFore = p->PPart + p->IPart;           // 观测角速度（rad/s）

    // 3. 角速度低通滤波（可选，用于平滑输出）
    p->WeForeLPF = p->WeFore * p->WeForeLPFFactor + p->WeForeLPF * (1 - p->WeForeLPFFactor);

    // 4. 积分得到观测电角度（压控振荡器）
    p->ThetaFore += p->WeFore * p->Ts;          // 角度积分

    // 5. 角度归一化到 [0, 2π)
    if (p->ThetaFore > 6.28318f)               // 2π ≈ 6.28318
    {
        p->ThetaFore -= 6.28318f;
    }
    else if (p->ThetaFore < 0)
    {
        p->ThetaFore += 6.28318f;
    }

    // 6. 角度格式转换（供外部使用）
    p->ETheta = p->ThetaFore;
    // 标幺化角度 0~1 对应 0~2π
    p->EThetaPU = p->ETheta / 6.28318f;
}
