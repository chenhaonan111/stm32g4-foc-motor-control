#include "position_drv.h"

/*计算位置（基于编码器电角度累加，支持过零处理）*/
void Calculate_Position(POSITION_STRUCT *p)
{
    // 计算本次电角度变化量（本次 - 上次）
    p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast;
    // 更新上次电角度为本次值
    p->ElectricalPosLast = p->ElectricalPosThis;

    // 正向越过零点处理：若变化量 ≥ 半量程，实际应为负变化，减去一个周期
    if (p->ElectricalPosChange >= (p->ElectricalValMax * 0.5f))
    {
        p->ElectricalPosChange = p->ElectricalPosChange - p->ElectricalValMax;
    }
    // 反向越过零点处理：若变化量 ≤ -半量程，实际应为正变化，加上一个周期
    if (p->ElectricalPosChange <= (-p->ElectricalValMax * 0.5f))
    {
        p->ElectricalPosChange = p->ElectricalPosChange + p->ElectricalValMax;
    }

    // 累加得到总电角度位置
    p->ElectricalPosSum = p->ElectricalPosSum + p->ElectricalPosChange;
}



