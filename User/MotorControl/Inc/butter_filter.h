#ifndef __BUTTER_FILTER_H__
#define __BUTTER_FILTER_H__

#include "main.h"

/* 二阶巴特沃斯低通滤波器（预畸变双线性变换离散化）
 * 命名说明：Butter = 巴特沃斯整定，LPF = 低通滤波器（避免 BPF=带通滤波器的歧义） */

typedef struct
{
    /* 运行时状态与系数 */
    float num[3];       /* 分子系数 b0,b1,b2（Init 计算生成） */
    float den[3];       /* 分母系数 a0,a1,a2（Init 计算生成） */
    float x[3];         /* 输入历史 x[k],x[k-1],x[k-2] */
    float y[3];         /* 输出历史 y[k],y[k-1],y[k-2] */
    float Input;        /* 当前输入（Calc 前赋值） */
    float Output;       /* 当前输出（Calc 后读取） */

    /* 设计参数（仅 Init 使用，运行时不参与计算） */
    float Wc;           /* 截止角频率(rad/s)，fc[Hz] = Wc/(2π) */
    float Ts;           /* 采样周期(s)，= 1/本滤波器调用频率 */
}BUTTER_LPF_STRUCT;

void Butter_LPF_Init(BUTTER_LPF_STRUCT *p);    /* Wc/Ts 赋值后调用一次 */
void Butter_LPF_Calc(BUTTER_LPF_STRUCT *p);    /* 每个滤波周期调用一次 */

#endif

