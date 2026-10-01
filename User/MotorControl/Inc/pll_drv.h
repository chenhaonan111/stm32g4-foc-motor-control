#ifndef __PLL_DRV_H__
#define __PLL_DRV_H__

#include "main.h"

typedef struct
{
    int8_t   Dir;                  //反电动势计算方向
    float    Ts;                   //调用周期

    float    Ain;                  //输入A
    float    Bin;                  //输入B

    float    ThetaErr;             //观测角度误差
    float    ThetaFore;            //观测角度 单位：弧度
    float    ThetaCompensate;      //补偿后的观测角度 单位：弧度
    float    ETheta;               //补偿后的观测角度 0-3999
    float    EThetaPU;

    float    SinVal;               //正弦值
    float    CosVal;               //余弦值

    float    Kp;                   //锁相环KP
    float    Ki;                   //锁相环KI
    float    PPart;                //积分项
    float    IPart;                //积分项

    float    WeFore;               //观测电角速度（rad/s）
    float    WeForeLPF;            //观测电角速度滤波值
    float    WeForeLPFFactor;      //观测电角速度滤波系数
}PLL_STRUCT;

void PLL_Calculate(PLL_STRUCT *p);

/* ============================================================================
 * SMO反电动势专用锁相环（归一化鉴相器）
 * 与上方PLL_STRUCT(原始叉积鉴相、误差幅值∝反电动势幅值)的区别：
 *   1. 鉴相器先求反电动势幅值EMag并归一化，误差恒在[-1,1]，增益不随转速变化；
 *   2. EMag<0.5V(低速反电动势不足)时误差强制置0，防止低速噪声误锁；
 *   3. 鉴相符号约定：正转速Dir=+1（与T_Shaped_Acc_Dec的SpeedOutDir配套，
 *      与PLL_STRUCT的Dir=-1约定相反，不可混用）
 * ==========================================================================*/
typedef struct
{
    int8_t   Dir;                  //速度方向（归一化鉴相符号约定：正转速+1）
    float    Ts;                   //调用周期

    float    Ealpha;               //输入：滤波后α轴反电动势
    float    Ebeta;                //输入：滤波后β轴反电动势

    float    EMag;                 //反电动势幅值
    float    Error;                //归一化鉴相误差[-1,1]
    float    SinVal;               //正弦值
    float    CosVal;               //余弦值

    float    Kp;                   //锁相环KP
    float    Ki;                   //锁相环KI
    float    Ppart;                //比例项
    float    Ipart;                //积分项

    float    We;                   //观测电角速度（rad/s）
    float    WeLPF;                //观测电角速度滤波值
    float    WeLPFFactor;          //观测电角速度滤波系数

    float    ETheta;               //观测角度 单位：弧度
    float    EThetaPU;             //观测角度标幺值0~1
}SMO_PLL;

void SMO_PLL_Calculate(SMO_PLL *p);

/* ============================================================================
 * HFI专用标量误差锁相环
 * 与上方PLL_STRUCT(正交信号输入型QPLL)的区别：
 *   输入go.Error为已解算好的标量误差信号(∝sin(2Δθ)，正弦HFI解调输出)，
 *   内部不含鉴相器，仅为PI(Tustin离散)+积分器；
 *   跟踪机械角度/机械角速度，电角度=OutThetaM×极对数
 * ==========================================================================*/
typedef struct{
    float ErrPrev;          // (数据)上一拍误差 e[k-1]
    float WmPrev;           // (数据)上一拍角速度输出 ωm[k-1]
    float PiNum[2];         // (中间量)Tustin离散PI分子系数(双线性变换离散)
    float IntegCoeff;       // (中间量)梯形积分系数(T/2)

    float Error;            // (输入数据)标量误差信号(正弦HFI解调输出OutputQ)
    float OutWm;            // (输出数据)机械角速度输出(rad/s)
    float OutThetaM;        // (输出数据)机械角度输出(rad)
}HFI_PLL_GO_STRUCT;

typedef struct{
    HFI_PLL_GO_STRUCT go;   // (结构体)锁相环运算数据

    float T;                // (系统时钟)运算离散周期

    float Kp;               // (参数设计)Kp比例项增益
    float Ki;               // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式(1=OutThetaM累计不归一化; 0=每拍归一化到[0,2π))
}HFI_PLL_STRUCT;

void HFI_PLL_Init(HFI_PLL_STRUCT *pll);
void HFI_PLL_Loop(HFI_PLL_STRUCT *pll);

/* ============================================================================
 * 编码器角度跟踪锁相环
 * 与上方HFI_PLL的区别：跟踪对象为编码器机械角(经零偏/方向校正后)，
 * 鉴相器在外部完成(编码器机械角-估计角,回卷[-π,π)后送入go.Error)，
 * 内部为PI(Tustin离散)+梯形积分，跟踪机械角度/机械角速度
 * ==========================================================================*/
typedef struct{
    float ErrPrev;          // (数据)上一拍误差 e[k-1]
    float WmPrev;           // (数据)上一拍角速度输出 ωm[k-1]
    float PiNum[2];         // (中间量)Tustin离散PI分子系数 [0]=Kp+Ki*T/2, [1]=-Kp+Ki*T/2
    float IntegCoeff;       // (中间量)梯形积分系数(T/2)

    float Error;            // (输入数据)角度误差输入(rad, 编码器: θm-OutThetaM, 已归一到[-π,π))
    float OutWm;            // (输出数据)机械角速度输出(rad/s)
    float OutThetaM;        // (输出数据)机械角度输出(rad), ×PolePairs得电角度
}PLL_GO_STRUCT;

typedef struct{
    PLL_GO_STRUCT go;       // (结构体)锁相环运算数据

    float T;                // (系统时钟)T运算离散周期

    float Kp;               // (参数设计)Kp比例项增益
    float Ki;               // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式(1=OutThetaM持续累计不归一化; 0=每拍归一化到[0,2π))
}ENC_PLL_STRUCT;

void ENC_PLL_Init(ENC_PLL_STRUCT *pll);
void ENC_PLL_Loop(ENC_PLL_STRUCT *pll);

#endif

