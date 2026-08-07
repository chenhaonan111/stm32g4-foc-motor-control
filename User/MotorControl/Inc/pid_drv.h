#ifndef __PID_DRV_H__
#define __PID_DRV_H__

#include "main.h"

typedef struct 
{
    float Kp;           //比例系数
    float Ki;           //积分系数    
    float Kd;           //微分系数
    float Ref;          //目标值
    float Fbk;          //反馈值
    float Out;          //输出值
    float Err;          //本次误差
    float ErrLast;      //上次误差
    float AllowErr;     //允许误差
    float Integrate;    //积分项
    float OutMax;       //输出上限
    float OutMin;       //输出下限
    float KpMax;        //比例系数上限（适用于分段式或模糊PID）
    float KpMin;        //比例系数下限（适用于分段式或模糊PID）
}PID_STRUCT;

void PID_Clear(PID_STRUCT *p);
void PID_Control(PID_STRUCT *p);

#endif
