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
 * HFI专用标量误差锁相环
 * 与上方PLL_STRUCT(正交信号输入型QPLL)的区别：
 *   输入go.Error为已解算好的标量误差信号(∝sin(2Δθ)，正弦HFI解调输出)，
 *   内部不含鉴相器，仅为PI(Tustin离散)+积分器；
 *   跟踪机械角度/机械角速度，电角度=OutRe×极对数
 * ==========================================================================*/
typedef struct{
    float We_i;             // (数据)上拍误差历史值
    float Re_i;             // (数据)上拍角速度输出历史值
    float X_num[2];         // (中间量)PI传递函数分子系数(双线性变换离散)
    float Y_num;            // (中间量)积分器系数(T/2)

    float Error;            // (输入数据)标量误差信号(正弦HFI解调输出OutputQ)
    float OutWe;            // (输出数据)机械角速度输出(rad/s)
    float OutRe;            // (输出数据)机械角度输出(rad)
}HFI_PLL_GO_STRUCT;

typedef struct{
    HFI_PLL_GO_STRUCT go;   // (结构体)锁相环运算数据

    float T;                // (系统时钟)运算离散周期

    float Kp;               // (参数设计)Kp比例项增益
    float Ki;               // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式标志位(0=角度归一化[0,2π))
}HFI_PLL_STRUCT;

void HFI_PLL_Init(HFI_PLL_STRUCT *pll);
void HFI_PLL_Loop(HFI_PLL_STRUCT *pll);

#endif

