#include "pid_drv.h"

/*PID控制（抗积分饱和*/
void PID_Control(PID_STRUCT *p)
{
    p->Err = p->Ref - p->Fbk;                           // 计算当前误差

    // 抗积分饱和：只有输出未饱和时才累加积分项
    if (p->Out < p->OutMax && p->Out > p->OutMin)
    {
        p->Integrate = p->Integrate + p->Ki * p->Err;   // 积分累加（Ki已包含采样时间）
    }

    // 计算PID输出（比例+积分+微分）
    p->Out = p->Kp * p->Err +                           // 比例项
             p->Integrate +                             // 积分项
             p->Kd * (p->Err - p->ErrLast);             // 微分项

    // 输出上限幅
    if (p->Out >= p->OutMax)
    {
        p->Out = p->OutMax;
    }
    // 输出下限幅
    if (p->Out <= p->OutMin)
    {
        p->Out = p->OutMin;
    }

    p->ErrLast = p->Err;                                // 保存本次误差供下次微分使用
}

/*重置PID所有相关变量为零*/
void PID_Clear(PID_STRUCT *p)
{
    p->Ref = 0;       // 重置目标值
    p->Fbk = 0;       // 重置反馈值
    p->Out = 0;       // 重置输出值
    p->Err = 0;       // 重置误差值
    p->ErrLast = 0;   // 重置上次误差值
    p->Integrate = 0; // 重置积分累加项
}

