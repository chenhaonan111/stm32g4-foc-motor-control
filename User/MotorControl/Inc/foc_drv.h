#ifndef __FOC_DRV_H
#define __FOC_DRV_H

#include "main.h"

/* FOC 核心数据结构:涵盖电流、角度三角函数、dq电压、αβ电压、母线电压、SVPWM占空比 */
typedef struct
{
    /* 三相电流(A) */
    float Iu;            /* U相电流 */
    float Iv;            /* V相电流 */
    float Iw;            /* W相电流 */
    float Ialpha;        /* α轴电流 */
    float Ibeta;         /* β轴电流 */

    /* 角度三角函数与 dq 轴电流 */
    float SinVal;        /* sin(电角度) */
    float CosVal;        /* cos(电角度) */
    float Id;            /* d轴电流(励磁) */
    float Iq;            /* q轴电流(转矩) */

    /* 电流低通滤波(阶段4 电流环用) */
    float IdLPF;            /* d轴滤波值 */
    float IqLPF;            /* q轴滤波值 */
    float IdLPFFactor;      /* d轴滤波系数 */
    float IqLPFFactor;      /* q轴滤波系数 */

    /* dq 轴电压指令与 αβ 电压 */
    float Ud;            /* d轴电压 */
    float Uq;            /* q轴电压 */
    float Ualpha;        /* α轴电压 */
    float Ubeta;         /* β轴电压 */
    float Ubus;          /* 母线电压(V) */

    /* SVPWM 参数与输出占空比 */
    uint16_t PwmCycle;      /* PWM 周期(定时器计数值,=2×ARR=8500) */
    uint16_t PwmLimit;      /* 最大占空比限幅(7800) */
    uint16_t DutyCycleA;    /* A相占空比 */
    uint16_t DutyCycleB;    /* B相占空比 */
    uint16_t DutyCycleC;    /* C相占空比 */
} FOC_STRUCT;

/* 坐标变换与 SVPWM */
void Clark_Transform(FOC_STRUCT *p);   /* 三相→αβ(阶段4用) */
void Park_Transform(FOC_STRUCT *p);    /* αβ→dq(阶段4用) */
void IPark_Transform(FOC_STRUCT *p);   /* dq→αβ(开环强拖用) */
void Calculate_SVPWM(FOC_STRUCT *p);   /* αβ→三相占空比(开环强拖用) */

#endif
