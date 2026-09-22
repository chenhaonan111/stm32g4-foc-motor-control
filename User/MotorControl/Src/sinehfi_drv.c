#include "sinehfi_drv.h"
#include "math_drv.h"

/* ============================================================================
 * 极性辨识（NSD）时序参数（20kHz电流环拍数）
 * 时序：等待PLL收敛(IdRef=0仅注入) → +脉冲斜坡/稳流/采样 → 归零
 *       → -脉冲斜坡/稳流/采样 → 判定
 * ============================================================================*/
#define NSD_WAIT_END        10400u  /* 等待PLL收敛阶段结束（0.52s，期间IdRef=0仅注入） */
#define NSD_RAMP_COUNT      400u    /* 辨识脉冲电流斜坡拍数（20ms，0→±NsdCurrent） */
#define NSD_SETTLE_COUNT    200u    /* 脉冲稳流拍数（10ms，等待电流环跟踪到位） */
#define NSD_SAMPLE_COUNT    2000u   /* 脉冲采样累加拍数（100ms≈109个注入周期） */
#define NSD_ZERO_COUNT      200u    /* 正负脉冲间电流归零拍数（10ms） */

#define NSD_RAMP1_END       (NSD_WAIT_END + NSD_RAMP_COUNT)      /* +脉冲斜坡结束 */
#define NSD_SETTLE1_END     (NSD_RAMP1_END + NSD_SETTLE_COUNT)   /* +脉冲稳流结束 */
#define NSD_SAMPLE1_END     (NSD_SETTLE1_END + NSD_SAMPLE_COUNT) /* +脉冲采样结束 */
#define NSD_ZERO_END        (NSD_SAMPLE1_END + NSD_ZERO_COUNT)   /* 归零阶段结束 */
#define NSD_RAMP2_END       (NSD_ZERO_END + NSD_RAMP_COUNT)      /* -脉冲斜坡结束 */
#define NSD_SETTLE2_END     (NSD_RAMP2_END + NSD_SETTLE_COUNT)   /* -脉冲稳流结束 */
#define NSD_SAMPLE2_END     (NSD_SETTLE2_END + NSD_SAMPLE_COUNT) /* -脉冲采样结束 */

static void HFI_Sin_Cos(float angle, float *sinval, float *cosval)
{
    // 0.15915494f = 1/(2π)。驱动层不包含motor_publicdata.h（避免循环包含），故用字面常量
    Calculate_Sin_Cos(Value_normalize(angle) * 0.15915494f, sinval, cosval);
}

/*高频注入（HFI）初始化——双线性变换离散化计算滤波器系数并清零历史状态*/
void HFI_Init(HFI_STRUCT *p)
{
    double temp0, temp1, temp2, den1, den2;
    double p_temp1, p_temp2, den;
    uint8_t i;

    // ====== 1. 陷波滤波器系数（双线性变换离散化） ======
    temp0 = ((double)p->T) * ((double)p->Wo) * ((double)p->K1);
    temp1 = ((double)p->T) * ((double)p->Wo) * ((double)p->K2);
    temp2 = ((double)p->T) * ((double)p->Wo) * ((double)p->T) * ((double)p->Wo);
    den1 = 4.0 + 4.0 * temp0 + temp2;
    den2 = 4.0 + 8.0 * temp0 + 4.0 * temp2;

    p->Go.S1Num[0] = (float)((4.0 + 4.0 * temp1 + temp2) / den1);
    p->Go.S1Num[1] = (float)((-8.0 + 2.0 * temp2) / den1);
    p->Go.S1Num[2] = (float)((4.0 - 4.0 * temp1 + temp2) / den1);
    p->Go.S1Den    = (float)((4.0 - 4.0 * temp0 + temp2) / den1);

    p->Go.S2Num[0] = (float)((4.0 + 8.0 * temp1 + 4.0 * temp2) / den2);
    p->Go.S2Num[1] = (float)((-8.0 + 8.0 * temp2) / den2);
    p->Go.S2Num[2] = (float)((4.0 - 8.0 * temp1 + 4.0 * temp2) / den2);
    p->Go.S2Den    = (float)((4.0 - 8.0 * temp0 + 4.0 * temp2) / den2);

    // ====== 2. 带通滤波器系数（双线性变换离散化，中心频率Wo） ======
    p_temp1 = ((double)p->T) * ((double)p->Wo) * (((double)p->Zeta) * 4.0);
    p_temp2 = ((double)p->T) * ((double)p->Wo) * ((double)p->T) * ((double)p->Wo);
    den = p_temp2 + p_temp1 + 4.0;

    p->Go.PNum    = (float)(p_temp1 / den);
    p->Go.PDen[0] = (float)((-8.0 + 2.0 * p_temp2) / den);
    p->Go.PDen[1] = (float)((p_temp2 - p_temp1 + 4.0) / den);

    // ====== 3. 清零全部滤波器历史状态 ======
    // 注：源库此处带通历史清零误写到了data0（实际带通状态在data1），
    //     本工程统一全清零，首次运行行为一致且重复初始化更稳健
    for (i = 0; i < 2; i++)
    {
        p->Data0.InputX[i] = 0.0f;  p->Data0.OutputX[i] = 0.0f;
        p->Data0.InputY[i] = 0.0f;  p->Data0.OutputY[i] = 0.0f;
        p->Data1.InputX[i] = 0.0f;  p->Data1.OutputX[i] = 0.0f;
        p->Data1.InputY[i] = 0.0f;  p->Data1.OutputY[i] = 0.0f;
    }

    // ====== 4. 清零输入输出与注入信号 ======
    p->Go.InputId = 0.0f;   p->Go.InputIq = 0.0f;
    p->Go.OutputId = 0.0f;  p->Go.OutputIq = 0.0f;
    p->Go.InputQ = 0.0f;    p->Go.OutputQ = 0.0f;
    p->Go.OutputUin = 0.0f;
    p->Go.Angle = 0.0f;
    p->Go.Sine[0] = 0.0f;   p->Go.Sine[1] = 0.0f;

    p->Re = 0.0f;
    p->ReCtrl = 0.0f;
    p->SpeedLPF = 0.0f;

    /* 清零极性辨识（NSD）状态（上电后自动启动一次NSD） */
    p->NSDFlag = 0;
    p->NSDOut = 0;
    p->NSDCount = 0;
    p->NSDSum1 = 0.0f;
    p->NSDSum2 = 0.0f;
    p->NSDIdRef = 0.0f;
    p->IdHigh = 0.0f;
    p->IdLast = 0.0f;
}

void HFI_Calculate(HFI_STRUCT *p)
{
    float Cosine;
    float TPNF_in;

    // ====== 1. 带通滤波，提取估计Q轴电流中的高频分量 ======
    // 差分方程：y[k] = PNum*(x[k]-x[k-2]) - PDen[0]*y[k-1] - PDen[1]*y[k-2]
    p->Go.High = p->Go.PNum * (p->Go.InputQ - p->Data1.InputX[1]) -
                 p->Go.PDen[0] * p->Data1.OutputX[0] -
                 p->Go.PDen[1] * p->Data1.OutputX[1];

    // ====== 2. 更新带通滤波器历史值 ======
    p->Data1.InputX[1] = p->Data1.InputX[0];
    p->Data1.InputX[0] = p->Go.InputQ;
    p->Data1.OutputX[1] = p->Data1.OutputX[0];
    p->Data1.OutputX[0] = p->Go.High;

    // ====== 3. 同频相乘解调（乘上拍注入正弦，含解调增益h） ======
    TPNF_in = p->Go.High * p->h * p->Go.Sine[1];

    // ====== 4. 陷波滤波，滤除二倍频分量（陷波点2Wo） ======
    p->Go.OutputQ = p->Go.S2Num[0] * TPNF_in +
                    p->Go.S2Num[1] * (p->Data1.InputY[0] - p->Data1.OutputY[0]) +
                    p->Go.S2Num[2] * p->Data1.InputY[1] -
                    p->Go.S2Den * p->Data1.OutputY[1];

    // ====== 5. 更新陷波滤波器历史值 ======
    p->Data1.InputY[1] = p->Data1.InputY[0];
    p->Data1.InputY[0] = TPNF_in;
    p->Data1.OutputY[1] = p->Data1.OutputY[0];
    p->Data1.OutputY[0] = p->Go.OutputQ;
    p->Go.Sine[1] = p->Go.Sine[0];

    // ====== 6. 刷新注入信号（相位积分+生成下一拍注入电压） ======
    // 注入电压用余弦，解调相乘用上一拍正弦（源库原设计，勿改）
    p->Go.Angle += p->Wo * p->T;
    p->Go.Angle = Value_normalize(p->Go.Angle);
    HFI_Sin_Cos(p->Go.Angle, &p->Go.Sine[0], &Cosine);
    p->Go.OutputUin = Cosine * p->Uh;
}

void HFI_Current_Filter(HFI_STRUCT *p)
{
    // ====== 1. DQ轴电流陷波差分方程 ======
    p->Go.OutputId = p->Go.S1Num[0] * p->Go.InputId +
                     p->Go.S1Num[1] * (p->Data0.InputX[0] - p->Data0.OutputX[0]) +
                     p->Go.S1Num[2] * p->Data0.InputX[1] -
                     p->Go.S1Den * p->Data0.OutputX[1];

    p->Go.OutputIq = p->Go.S1Num[0] * p->Go.InputIq +
                     p->Go.S1Num[1] * (p->Data0.InputY[0] - p->Data0.OutputY[0]) +
                     p->Go.S1Num[2] * p->Data0.InputY[1] -
                     p->Go.S1Den * p->Data0.OutputY[1];

    // ====== 2. 更新陷波滤波器历史值 ======
    p->Data0.InputX[1] = p->Data0.InputX[0];
    p->Data0.InputX[0] = p->Go.InputId;
    p->Data0.OutputX[1] = p->Data0.OutputX[0];
    p->Data0.OutputX[0] = p->Go.OutputId;

    p->Data0.InputY[1] = p->Data0.InputY[0];
    p->Data0.InputY[0] = p->Go.InputIq;
    p->Data0.OutputY[1] = p->Data0.OutputY[0];
    p->Data0.OutputY[0] = p->Go.OutputIq;
}

/* ============================================================================
 * 极性辨识（NSD）：消除sin(2Δθ)解调固有的180°极性模糊
 * 原理：估算角收敛到θ或θ+π（二义）后，先后施加±NsdCurrent的d轴电流脉冲，
 *       用|ΔId|/2的窗口累加度量d轴高频电流响应幅值（∝Uh/(Wo*Ld)）。
 *       脉冲电流与转子N极同向时磁路饱和更深、Ld更小、高频响应更大：
 *       NSDSum2 > NSDSum1 => 估计d轴指向S极 => NSDOut=1，调用方对角度+π
 * 时序（重构）：先等待PLL仅凭注入信号收敛（IdRef=0，若PLL未锁定时施加
 *       电流阶跃，暂态经带通耦合进解调误差会把PLL踢到θ+π收敛点，
 *       随后的d轴电流会把转子拖动180°电角）；之后脉冲电流均斜坡进入，
 *       消除阶跃暂态
 * 输入：p->Go.InputId为本拍原始d轴电流（调用方在Park变换并赋值后调用本函数）
 * ==========================================================================*/
void HFI_NSD_Calculate(HFI_STRUCT *p)
{
    if (p->NSDFlag != 0)
    {
        return;
    }

    p->NSDCount++;

    /* d轴高频电流幅值：|Id-IdLast|/2（同SQHFI，稳态直流分量经差分自动消除） */
    p->IdHigh = (p->Go.InputId - p->IdLast) * 0.5f;
    p->IdLast = p->Go.InputId;
    if (p->IdHigh < 0.0f)
    {
        p->IdHigh = -p->IdHigh;
    }

    if (p->NSDCount < NSD_WAIT_END)
    {
        /* 阶段1：等待PLL收敛（IdRef=0，仅注入信号，转子不受力） */
        p->NSDIdRef = 0.0f;
    }
    else if (p->NSDCount < NSD_RAMP1_END)
    {
        /* 阶段2：+脉冲电流斜坡建立（0→+NsdCurrent） */
        p->NSDIdRef = p->NsdCurrent * (float)(p->NSDCount - NSD_WAIT_END) / (float)NSD_RAMP_COUNT;
    }
    else if (p->NSDCount < NSD_SETTLE1_END)
    {
        /* 阶段3：+脉冲稳流（电流环跟踪到位后再开采样窗） */
        p->NSDIdRef = p->NsdCurrent;
    }
    else if (p->NSDCount < NSD_SAMPLE1_END)
    {
        /* 阶段4：+脉冲期间累加高频响应幅值 */
        p->NSDIdRef = p->NsdCurrent;
        p->NSDSum1 += p->IdHigh;
    }
    else if (p->NSDCount < NSD_ZERO_END)
    {
        /* 阶段5：电流归零（PLL已锁定，衰减暂态被环路吸收） */
        p->NSDIdRef = 0.0f;
    }
    else if (p->NSDCount < NSD_RAMP2_END)
    {
        /* 阶段6：-脉冲电流斜坡建立（0→-NsdCurrent） */
        p->NSDIdRef = -p->NsdCurrent * (float)(p->NSDCount - NSD_ZERO_END) / (float)NSD_RAMP_COUNT;
    }
    else if (p->NSDCount < NSD_SETTLE2_END)
    {
        /* 阶段7：-脉冲稳流 */
        p->NSDIdRef = -p->NsdCurrent;
    }
    else if (p->NSDCount < NSD_SAMPLE2_END)
    {
        /* 阶段8：-脉冲期间累加高频响应幅值 */
        p->NSDIdRef = -p->NsdCurrent;
        p->NSDSum2 += p->IdHigh;
    }
    else
    {
        /* 阶段9：辨识完成，比较判定极性 */
        p->NSDIdRef = 0.0f;
        p->NSDFlag = 1;
        p->NSDOut = (p->NSDSum2 > p->NSDSum1) ? 1 : 0;
        p->NSDCount = 0;
    }
}
