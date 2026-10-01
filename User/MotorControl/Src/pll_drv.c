#include "pll_drv.h"
#include "math_drv.h"
#include <math.h>

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

/*函数功能: SMO反电动势专用锁相环——归一化鉴相器+低速EMF门限*/
void SMO_PLL_Calculate(SMO_PLL *p)
{
    // 1. 求反电动势幅值
    p->EMag = sqrtf(p->Ealpha * p->Ealpha + p->Ebeta * p->Ebeta);

    // 2. 归一化鉴相：误差=Dir*(-Eα*cosθ̂-Eβ*sinθ̂)/EMag，恒在[-1,1]，
    //    增益不随反电动势幅值(转速)变化；反电动势不足时(EMag<0.5V)误差置0防误锁
    if (p->EMag >= 0.5f)
    {
        p->Error = p->Dir * (-p->Ealpha * p->CosVal - p->Ebeta * p->SinVal) / p->EMag;
    }
    else
    {
        p->Error = 0.0f;
    }

    // 3. PI环路滤波器
    p->Ppart = p->Error * p->Kp;
    p->Ipart += p->Error * p->Ki;
    p->We = p->Ppart + p->Ipart;

    // 4. 观测电角速度低通滤波
    p->WeLPF = p->WeLPFFactor * p->We + (1.0f - p->WeLPFFactor) * p->WeLPF;

    // 5. 积分得到观测电角度（压控振荡器）
    p->ETheta += p->We * p->Ts;

    // 6. 角度归一化到 [0, 2π)
    if (p->ETheta > 6.28318f)
    {
        p->ETheta -= 6.28318f;
    }
    else if (p->ETheta < 0.0f)
    {
        p->ETheta += 6.28318f;
    }

    // 7. 角度格式转换（标幺值 0~1 对应 0~2π）
    p->EThetaPU = p->ETheta / 6.28318f;
}

/* ============================================================================
 * HFI专用标量误差锁相环
 * 输入Error(∝sin(2Δθ))，输出机械角速度OutWm/机械角度OutThetaM
 * ==========================================================================*/
void HFI_PLL_Init(HFI_PLL_STRUCT *pll){
    double temp0 = ((double)pll->T) * ((double)pll->Ki);
    pll->go.PiNum[0] = (float)((2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.PiNum[1] = (float)((-2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.IntegCoeff = (float)(((double)pll->T) / 2.0);

    // 初始化为零
    pll->is_position_mode = 0; // 默认非位置环mode
    pll->go.ErrPrev = 0.0f;
    pll->go.WmPrev = 0.0f;
    pll->go.OutWm = 0.0f;
    pll->go.OutThetaM = 0.0f;
    pll->go.Error = 0.0f;
}

void HFI_PLL_Loop(HFI_PLL_STRUCT *pll){
    // 1.计算PI控制器(增量式, 输出OutWm)
    pll->go.OutWm += pll->go.PiNum[0] * pll->go.Error +
                    pll->go.PiNum[1] * pll->go.ErrPrev;

    // 2.计算积分器(梯形积分, 输出OutThetaM)
    pll->go.OutThetaM += pll->go.IntegCoeff * (pll->go.OutWm + pll->go.WmPrev);
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutThetaM = Value_normalize(pll->go.OutThetaM);
    }

    // 3.更新历史值(ErrPrev=e[k-1], WmPrev=ωm[k-1])
    pll->go.ErrPrev = pll->go.Error;
    pll->go.WmPrev = pll->go.OutWm;
}

/* ============================================================================
 * 编码器角度跟踪锁相环：输入Error(编码器机械角与估计角之差)，
 * 输出机械角速度OutWm/机械角度OutThetaM
 * ==========================================================================*/
void ENC_PLL_Init(ENC_PLL_STRUCT *pll)
{
    double temp0 = ((double)pll->T) * ((double)pll->Ki);
    pll->go.PiNum[0] = (float)((2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.PiNum[1] = (float)((-2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.IntegCoeff = (float)(((double)pll->T) / 2.0);

    // 初始化为零
    pll->is_position_mode = 0; // 默认非位置环mode
    pll->go.ErrPrev = 0.0f;
    pll->go.WmPrev = 0.0f;
    pll->go.OutWm = 0.0f;
    pll->go.OutThetaM = 0.0f;
    pll->go.Error = 0.0f;
}

void ENC_PLL_Loop(ENC_PLL_STRUCT *pll){
    // 1.计算PI控制器(增量式, 输出OutWm)
    pll->go.OutWm += pll->go.PiNum[0] * pll->go.Error +
                    pll->go.PiNum[1] * pll->go.ErrPrev;

    // 2.计算积分器(梯形积分, 输出OutThetaM)
    pll->go.OutThetaM += pll->go.IntegCoeff * (pll->go.OutWm + pll->go.WmPrev);
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutThetaM = Value_normalize(pll->go.OutThetaM);
    }

    // 3.更新历史值(ErrPrev=e[k-1], WmPrev=ωm[k-1])
    pll->go.ErrPrev = pll->go.Error;
    pll->go.WmPrev = pll->go.OutWm;
}
