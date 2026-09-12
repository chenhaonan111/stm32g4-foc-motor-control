#include "speed_drv.h"
#include "math.h"

/*计算反馈速度（基于电角度差*/
void Calculate_Speed(SPEED_STRUCT *p)
{
    // ------------------------------------------------------------------------
    // 1. 计算单位时间内的电角度变化量（差分）
    // ------------------------------------------------------------------------
    // ElectricalPosThis: 本次采样的电角度（标幺值，范围0~1）
    // ElectricalPosLast: 上次采样的电角度
    // 差值可能为正或负，取决于旋转方向
    p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast;
    // 保存本次角度供下次使用
    p->ElectricalPosLast = p->ElectricalPosThis;

    // ------------------------------------------------------------------------
    // 2. 处理电角度过零点跳变（修正由于角度从1回绕到0或0回绕到1引起的突变）
    // ------------------------------------------------------------------------
    // 当差值 ≥ 0.5 时，说明实际变化应该是负方向（逆时针绕了一圈），
    // 因为标幺值从0.1变到0.9，正常差值0.8，但实际角度只减少了0.2（顺时针），
    // 所以减去1.0得到-0.2，表示负方向小位移。
    if(p->ElectricalPosChange >= 0.5f)             
    {
        p->ElectricalPosChange = p->ElectricalPosChange - 1.0f;
    }
    // 当差值 ≤ -0.5 时，说明实际变化应该是正方向（顺时针绕了一圈），
    // 加1.0得到正值小位移。
    if(p->ElectricalPosChange <= -0.5f)
    {
        p->ElectricalPosChange = p->ElectricalPosChange + 1.0f;
    }

    // ------------------------------------------------------------------------
    // 3. 计算原始电角速度（单位：rpm）
    // ------------------------------------------------------------------------
    // ElectricalSpeedFactor: 速度转换系数，通常为 (1.0 / (Ts * 分频因子)) * 60
    // 公式：电角速度(rpm) = 电角度变化(标幺值) × 系数
    // 因为标幺值变化1对应360°电角度，系数将每Ts秒的变化转换为每分钟转数。
    p->ElectricalSpeedRaw = p->ElectricalPosChange * p->ElectricalSpeedFactor;

    // ------------------------------------------------------------------------
    // 4. 二阶巴特沃斯低通滤波（减小速度噪声，-40dB/dec）
    // ------------------------------------------------------------------------
    // Butter_LPF_Calc 内部执行二阶差分方程，结果写入 Output
    p->ButterLPF.Input = p->ElectricalSpeedRaw;
    Butter_LPF_Calc(&p->ButterLPF);
    p->ElectricalSpeedLPF = p->ButterLPF.Output;    // 复用 ElectricalSpeedLPF 变量
    
    // ------------------------------------------------------------------------
    // 5. 计算机械速度（单位：rpm）
    // ------------------------------------------------------------------------
    // 电角速度 = 机械角速度 × 极对数
    // 所以机械速度(rpm) = 电角速度(rpm) / 极对数
    p->MechanicalSpeed = p->ElectricalSpeedLPF / p->PolePairs;
}

/*T形加减速（梯形速度曲线规划）*/
void T_Shaped_Acc_Dec(TSHAPEDACCDEC_STRUCT *p)
{
    // ------------------------------------------------------------------------
    // 1. 计算目标速度对应的每周期位置增量（期望的步长）
    // ------------------------------------------------------------------------
    // TargetSpeed: 目标转速（rpm）
    // 除以60转换为转/秒，再乘以Ts（秒）得到每周期应移动的转数
    p->SpeedTargetIncrement = p->TargetSpeed / 60.0f * p->Ts;

    // ------------------------------------------------------------------------
    // 2. 计算每个周期能增加的速度步长（加速度对应的位置增量步长）
    // ------------------------------------------------------------------------
    // AccSpeed: 加速度（rpm/s）
    // 除以60转换为转/秒2，乘以Ts得到每个周期速度的变化量（转/秒），
    // 再乘以Ts得到每个周期位置增量的变化量（转数）。这里实际计算的是：
    // 每个周期可以额外增加的位置增量 = (AccSpeed/60) * Ts2
    p->SpeedIncrement = p->AccSpeed / 60.0f * p->Ts * p->Ts;

    // ------------------------------------------------------------------------
    // 3. 根据当前累积位置增量与目标位置增量的关系，调整加速度方向
    // ------------------------------------------------------------------------
    // SpeedChangeIncrement: 当前周期的位置增量（累积值），单位：转数
    if(p->SpeedChangeIncrement < p->SpeedTargetIncrement){
        // 当前增量小于目标增量 -> 加速阶段
        p->SpeedChangeIncrement += p->SpeedIncrement;   // 每个周期增加一个步长
        // 防止超调，如果超过目标值则钳位到目标值
        if(p->SpeedChangeIncrement >= p->SpeedTargetIncrement){
            p->SpeedChangeIncrement = p->SpeedTargetIncrement;
        }
    }else if(p->SpeedChangeIncrement > p->SpeedTargetIncrement){
        // 当前增量大于目标增量 -> 减速阶段
        p->SpeedChangeIncrement -= p->SpeedIncrement;   // 每个周期减少一个步长
        if(p->SpeedChangeIncrement <= p->SpeedTargetIncrement){
            p->SpeedChangeIncrement = p->SpeedTargetIncrement;
        }
    }
    // 如果相等，则保持匀速，不做调整

    // ------------------------------------------------------------------------
    // 4. 运动状态检测（加速、匀速、减速）
    // ------------------------------------------------------------------------
    // 计算当前周期速度变化量的增量（即加速度的符号变化）
    // fabsf(p->SpeedChangeIncrement) 取绝对值，因为方向可能正负
    // 减去上一周期的绝对值，得到变化趋势：
    //   >0 表示绝对值增大 -> 加速
    //   <0 表示绝对值减小 -> 减速
    //   =0 表示不变 -> 匀速
    p->SpeedIincrementDelta = fabsf(p->SpeedChangeIncrement) - p->SpeedIncrementLast;
    // 保存本周期绝对值，供下一周期使用
    p->SpeedIncrementLast = fabsf(p->SpeedChangeIncrement);
    // 根据差值正负确定运动状态（注意这里使用三目运算符）
    p->MotionState = p->SpeedIincrementDelta > 0 ? ACCELERATE : 
                    (p->SpeedIincrementDelta < 0 ? DECELERATE : UNIFORM);

    // ------------------------------------------------------------------------
    // 5. 确定速度输出的方向
    // ------------------------------------------------------------------------
    // SMO锁相环方向标志Dir
    p->SpeedOutDir = p->SpeedChangeIncrement > 0 ? -1 : 1;

    // ------------------------------------------------------------------------
    // 6. 计算最终输出的转速（单位：rpm）
    // ------------------------------------------------------------------------
    // SpeedChangeIncrement 单位是“转数”（位置增量），除以 Ts 得到速度（转/秒），
    // 乘以60得到转速（rpm），再乘以极对数得到电机的电气转速（供速度环使用）。
    // 注意：极对数转换是因为速度环通常使用电角速度。
    p->SpeedOut = p->SpeedChangeIncrement * 60.0f / p->Ts * p->PolePairs;
}

