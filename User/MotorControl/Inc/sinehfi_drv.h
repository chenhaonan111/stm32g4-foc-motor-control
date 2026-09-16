#ifndef __SINEHFI_DRV_H__
#define __SINEHFI_DRV_H__

#include "main.h"
#include "pll_drv.h"
#include "butter_filter.h"

typedef struct
{
    float Angle;            // 注入信号相位角（rad，[0,2π)循环累加）
    float Sine[2];          // 注入正弦历史值 Sine[0]本拍 Sine[1]上拍（解调用Sine[1]）
    float High;             // 仅经带通滤波后的高频电流分量

    float InputId;          // (输入)原始D轴电流信号（Park变换后，未滤波）
    float InputIq;          // (输入)原始Q轴电流信号
    float OutputId;         // (输出)陷波滤波后的基频D轴电流（供电流环）
    float OutputIq;         // (输出)陷波滤波后的基频Q轴电流

    float InputQ;           // (输入)估计Q轴电流（由Ialpha/Ibeta和上拍估计电角度投影得到）
    float OutputQ;          // (输出)解调后的位置误差信号（∝sin(2Δθ)，送PLL误差）
    float OutputUin;        // (输出)D轴高频注入电压叠加量（V）

    float S1Num[3];         // (中间量)陷波滤波器1（DQ电流陷波，陷波点Wo）分子系数
    float S1Den;            // (中间量)陷波滤波器1分母系数(z^-2项)
    float S2Num[3];         // (中间量)陷波滤波器2（解调信号陷波，陷波点2Wo）分子系数
    float S2Den;            // (中间量)陷波滤波器2分母系数(z^-2项)

    float PNum;             // (中间量)带通滤波器（中心Wo）分子系数
    float PDen[2];          // (中间量)带通滤波器分母系数(z^-1,z^-2项)
}HFI_GO_STRUCT;

/* 二阶IIR滤波器历史状态 */
typedef struct
{
    float InputX[2];        // x通道滤波器历史输入值 [0]上拍 [1]上上拍
    float OutputX[2];       // x通道滤波器历史输出值

    float InputY[2];        // y通道滤波器历史输入值
    float OutputY[2];       // y通道滤波器历史输出值
}HFI_IIR_STRUCT;

/* 高频注入（HFI）总结构体 */
typedef struct
{
    HFI_GO_STRUCT   Go;         // 高频注入运算数据
    HFI_IIR_STRUCT Data0;      // 陷波滤波器1历史状态（X通道对应Id，Y通道对应Iq）
    HFI_IIR_STRUCT Data1;      // 带通滤波器（X通道）与陷波滤波器2（Y通道）历史状态

    float T;                    // 离散周期（s），等于TS
    float Wo;                   // 注入电压角频率（rad/s）
    float h;                    // 高频解调增益
    float Uh;                   // 注入电压幅值（V）

    float K1;                   // 陷波滤波器分母阻尼比（越大陷波越宽越浅）
    float K2;                   // 陷波滤波器分子阻尼比（0=理想深陷波）
    float Zeta;                 // 带通滤波器阻尼比（越小选频性越强）

    HFI_PLL_STRUCT Pll;         // 标量误差锁相环（跟踪机械角/机械角速度，电角度=OutThetaM×极对数）
    float Re;                   // 【估计电角度】PLL输出机械角×极对数。投影、Park/IPark、
                                //   注入三坐标系统一用此角，保证解调基准与注入轴一致
                                //   （勿拆分为双角度，见调用处注释）
    float ReCtrl;               // 【控制用角】恒等于Re，保留字段仅供调试通道/Watch观测
    float SpeedLPF;             // 观测机械角速度滤波值（rad/s，= SpeedLpf.Output）
    BUTTER_LPF_STRUCT SpeedLpf; // 观测速度二阶巴特沃斯低通（Wc=100rad/s≈15.9Hz）
    float SpeedMax;             // 速度给定限幅（电rpm，HFI仅低速域有效）
    float DebugIdBias;          // （参数设计）调试模式Id偏置电流(A)，0=关闭，磁饱和凸极性检测用
    float NsdCurrent;           // （参数设计）NSD极性辨识d轴脉冲电流幅值(A)，
                                //   须按电机磁饱和深度整定（可用DebugIdBias自检模式实测）

    /* 极性辨识（NSD）：消除sin(2Δθ)解调固有的180°极性模糊 */
    uint8_t  NSDFlag;           // (标志)极性辨识完成标志：0=进行中 1=已完成
    uint8_t  NSDOut;            // (输出)辨识结果：1=估计d轴指向S极，需电角度+π修正
    uint16_t NSDCount;          // (数据)阶段计数器（20kHz电流环计数）
    float    NSDSum1;           // (数据)+IdRef脉冲期间d轴高频电流幅值累加
    float    NSDSum2;           // (数据)-IdRef脉冲期间d轴高频电流幅值累加
    float    NSDIdRef;          // (输出)极性辨识期间d轴电流给定(A)
    float    IdHigh;            // (数据)d轴高频电流幅值（|ΔId|/2，正比Uh/(Wo*Ld)）
    float    IdLast;            // (数据)上拍d轴电流
}HFI_STRUCT;

void HFI_Init(HFI_STRUCT *p);
void HFI_Calculate(HFI_STRUCT *p);
void HFI_Current_Filter(HFI_STRUCT *p);
void HFI_NSD_Calculate(HFI_STRUCT *p);

#endif

