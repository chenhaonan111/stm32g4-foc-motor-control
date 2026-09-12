#include "sqhfi_drv.h"

/*方波高频注入（SQHFI）及信号解析（用于零低速无传感器控制*/
void SQHFI_Calculate(SQHFI_STRUCT *p)
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
