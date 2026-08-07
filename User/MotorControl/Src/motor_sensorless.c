#include "motor_sensorless.h"
#include "motor_publicdata.h"
#include "math.h"

void Strong_Drag_Current_Open_Loop(void);
void Strong_Drag_Current_Close_Loop(void);
void Strong_Drag_Smo_Speed_Current_Loop(void);
void Strong_Drag_To_Observer_Cal(STRONG_DRAG_TO_OBSERVER *p);
void Hfi_Current_Close_Loop(void);
void Hfi_Speed_Current_Loop(void);
void Hfi_Smo_Speed_Current_Loop(void);
void Hfi_TO_Obsever_Cal(HFI_TO_OBSERVER *p);

/*无传感器（无感）模式主控函数*/
void Sensorless_Control()
{
    // 根据运行模式选择对应的控制策略
    switch(MC.Motor.RunMode)
    {
        // --------------------------------------------------------------------
        // 模式1: 电流开环强拖（STRONG_DRAG_CURRENT_OPEN）
        // --------------------------------------------------------------------
        // 说明: 最简单的开环强制拖动，不依赖电流反馈，直接给定固定电压（Ud通常为0），
        //       通过开环电角度发生器驱动电机旋转。用于电机启动初期或低速试探。
        //       此模式下无法带载，仅用于让电机转动起来。
        case STRONG_DRAG_CURRENT_OPEN:
        {
            Strong_Drag_Current_Open_Loop();
        }
        break;

        // --------------------------------------------------------------------
        // 模式2: 电流闭环强拖（STRONG_DRAG_CURRENT_CLOSE）
        // --------------------------------------------------------------------
        // 说明: 在开环强拖基础上，加入电流闭环控制（Id环给定固定励磁电流，Iq环给定0或固定值），
        //       能够限制电流、提供一定转矩，但角度仍是开环生成。适用于需要一定力矩的低速强拖。
        case STRONG_DRAG_CURRENT_CLOSE:
        {
            Strong_Drag_Current_Close_Loop();
        }
        break;

        // --------------------------------------------------------------------
        // 模式3: 强拖 + 滑模观测器（SMO）速度电流闭环（STRONG_DRAG_SMO_SPEED_CURRENT_LOOP）
        // --------------------------------------------------------------------
        // 说明: 从开环强拖向无传感器闭环过渡的阶段。使用滑模观测器估算反电动势，
        //       通过锁相环提取角度和速度，并实现速度环和电流环闭环。
        //       内部包含开环到闭环的平滑切换逻辑，适用于电机有一定转速后切入闭环。
        case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:
        {
            Strong_Drag_Smo_Speed_Current_Loop();
        }
        break;

        // --------------------------------------------------------------------
        // 模式4: 高频注入（HFI）电流闭环（HFI_CURRENT_CLOSE）
        // --------------------------------------------------------------------
        // 说明: 仅使用高频注入法估算转子位置（静止或极低速），实现电流闭环控制。
        //       没有速度环，需要外部手动控制电流或速度参考。注意：纯HFI不可高速运行，
        //       通常用于电机堵转或极低速调试。
        case HFI_CURRENT_CLOSE:
        {
            Hfi_Current_Close_Loop();
        }
        break;

        // --------------------------------------------------------------------
        // 模式5: 高频注入（HFI）速度电流闭环（HFI_SPEED_CURRENT_CLOSE）
        // --------------------------------------------------------------------
        // 说明: 在 HFI 基础上增加速度外环，实现速度闭环控制。但由于 HFI 本身依赖注入信号，
        //       响应带宽有限，通常只能用于较低转速（例如针对4006无刷电机限速2500rpm）。
        //       适合零速到中低速的无传感器速度控制。
        case HFI_SPEED_CURRENT_CLOSE:
        {
            Hfi_Speed_Current_Loop();
        }
        break;

        // --------------------------------------------------------------------
        // 模式6: 高频注入（HFI）+ 滑模观测器（SMO）全速域无感速度电流闭环（HFI_SMO_SPEED_CURRENT_CLOSE）
        // --------------------------------------------------------------------
        case HFI_SMO_SPEED_CURRENT_CLOSE:
        {
            Hfi_Smo_Speed_Current_Loop();
        }
        break;
    }
}

/*强拖模式 - 电流开环控制*/
void Strong_Drag_Current_Open_Loop(void)
{
    // ------------------------------------------------------------------------
    // 1. T型加减速处理：生成平滑的速度斜坡
    // ------------------------------------------------------------------------
    // 目标速度由外部设定（如电位器或通信指令）
    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);                // 计算当前周期应输出的速度（rpm）

    // ------------------------------------------------------------------------
    // 2. 设置电角速度并生成开环电角度
    // ------------------------------------------------------------------------
    // 将T型加减速输出的速度作为电角速度设定值（单位：rpm）
    MC.EAngle.ElectricalAngleSpdSet = MC.TAccDec.SpeedOut;
    // 电角度发生器：根据设定的电角速度，通过积分生成连续变化的电角度（标幺值0~1）
    Electrical_Angle_Generator(&MC.EAngle);

    // ------------------------------------------------------------------------
    // 3. 计算电角度的正弦和余弦值
    // ------------------------------------------------------------------------
    // 用于后续反Park变换，将旋转坐标系电压变换到静止坐标系
    Calculate_Sin_Cos(MC.EAngle.ElectricalAngleSetPU, &MC.Foc.SinVal, &MC.Foc.CosVal);

    // ------------------------------------------------------------------------
    // 4. 给定电压
    // ------------------------------------------------------------------------
    // 开环电压给定(Ud=0);Uq 随速度指令有无斜坡升降,
    // 避免电位器回死区、角度停转后 Uq 仍常开造成的持续直流堵转电流
    {
        static float UqRamp     = 0.0f;
        const  float UqOpenSet  = 1.5f;
        const  float UqRampStep = 0.0002f;

        float uqTarget = (MC.Speed.MechanicalSpeedSet > 0.5f ||
                          MC.Speed.MechanicalSpeedSet < -0.5f) ? UqOpenSet : 0.0f;

        if      (UqRamp < uqTarget) { UqRamp += UqRampStep; if (UqRamp > uqTarget) UqRamp = uqTarget; }
        else if (UqRamp > uqTarget) { UqRamp -= UqRampStep; if (UqRamp < uqTarget) UqRamp = uqTarget; }

        MC.Foc.Ud = 0.0f;
        MC.Foc.Uq = UqRamp;
    }
    // ------------------------------------------------------------------------
    // 5. 反Park变换：Ud/Uq -> Ualpha/Ubeta
    // ------------------------------------------------------------------------
    // 根据当前电角度的正余弦值，将Ud/Uq（旋转坐标系）转换为Ualpha/Ubeta（静止坐标系）
    // 注意：此时Uq并未显式赋值，可能默认为0或之前保留的值。实际开环强拖常给固定Uq。
    IPark_Transform(&MC.Foc);

    // ------------------------------------------------------------------------
    // 6. 更新母线电压（直流母线电压采样值）
    // ------------------------------------------------------------------------
    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // ------------------------------------------------------------------------
    // 7. SVPWM调制：生成三相逆变器的PWM占空比
    // ------------------------------------------------------------------------
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

/*强拖模式 - 电流闭环控制*/
void Strong_Drag_Current_Close_Loop(void)
{
    // ------------------------------------------------------------------------
    // 1. 设置Id环的目标参考值（励磁电流给定）
    // ------------------------------------------------------------------------
    // 强拖时通常需要建立一定的磁场，给定一个固定的Id参考值（单位：安培）
    MC.IdPid.Ref = 1.0f;                        // 设定Id目标为1A（具体值根据电机调整）

    // ------------------------------------------------------------------------
    // 2. T型加减速处理：生成平滑的速度斜坡
    // ------------------------------------------------------------------------
    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);           // 计算当前周期应输出的速度（rpm）

    // ------------------------------------------------------------------------
    // 3. 设置电角速度并生成开环电角度（注意：仍然使用开环角度发生器）
    // ------------------------------------------------------------------------
    // 将T型加减速输出的速度作为电角速度设定值（单位：rpm）
    MC.EAngle.ElectricalAngleSpdSet = MC.TAccDec.SpeedOut;
    // 电角度发生器：根据设定的电角速度积分生成电角度（开环方式）
    Electrical_Angle_Generator(&MC.EAngle);

    // ------------------------------------------------------------------------
    // 4. 计算电角度的正弦和余弦值
    // ------------------------------------------------------------------------
    Calculate_Sin_Cos(MC.EAngle.ElectricalAngleSetPU, &MC.Foc.SinVal, &MC.Foc.CosVal);

    // ------------------------------------------------------------------------
    // 5. 读取相电流实际值（反馈值）
    // ------------------------------------------------------------------------
    MC.Foc.Iu = MC.Sample.IuReal;
    MC.Foc.Iv = MC.Sample.IvReal;

    // ------------------------------------------------------------------------
    // 6. Clark变换：三相静止坐标系 -> 两相静止坐标系（Ialpha, Ibeta）
    // ------------------------------------------------------------------------
    Clark_Transform(&MC.Foc);

    // ------------------------------------------------------------------------
    // 7. Park变换：两相静止坐标系 -> 旋转坐标系（Id, Iq）
    // ------------------------------------------------------------------------
    // 使用开环生成的当前电角度进行变换，得到实际反馈的Id、Iq
    Park_Transform(&MC.Foc);

    // ------------------------------------------------------------------------
    // 8. 对Id、Iq进行低通滤波（抑制噪声）
    // ------------------------------------------------------------------------
    MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor);
    MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor);

    // ------------------------------------------------------------------------
    // 9. 设置电流环PID的反馈值
    // ------------------------------------------------------------------------
    MC.IqPid.Fbk = MC.Foc.IqLPF;             // Iq环反馈（转矩电流）
    MC.IdPid.Fbk = MC.Foc.IdLPF;             // Id环反馈（励磁电流）

    // ------------------------------------------------------------------------
    // 10. 执行电流环PID调节
    // ------------------------------------------------------------------------
    // Id环：参考值为1A（设定值），输出Ud
    // Iq环：参考值来自哪里？注意本函数没有设置MC.IqPid.Ref，可能是外部已设定或默认为0。
    //       因此此时Iq环可能执行的是0电流控制，即仅建立励磁而不产生转矩。
    PID_Control(&MC.IqPid);                  // Iq闭环调节，输出Uq
    PID_Control(&MC.IdPid);                  // Id闭环调节，输出Ud

    // ------------------------------------------------------------------------
    // 11. 更新电压指令
    // ------------------------------------------------------------------------
    MC.Foc.Uq = MC.IqPid.Out;                // q轴电压指令
    MC.Foc.Ud = MC.IdPid.Out;                // d轴电压指令

    // ------------------------------------------------------------------------
    // 12. 反Park变换：旋转坐标系电压 -> 两相静止坐标系电压（Ualpha, Ubeta）
    // ------------------------------------------------------------------------
    IPark_Transform(&MC.Foc);

    // ------------------------------------------------------------------------
    // 13. 更新母线电压（直流母线电压采样值）
    // ------------------------------------------------------------------------
    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // ------------------------------------------------------------------------
    // 14. SVPWM调制：生成三相逆变器的PWM占空比
    // ------------------------------------------------------------------------
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

/*强拖模式到滑模观测器的速度-电流双闭环控制（无传感器过渡）*/
void Strong_Drag_Smo_Speed_Current_Loop(void)
{
    // ============================================================================
    // 1. 动态更新滑模观测器增益（基于母线电压）
    // ============================================================================
    // INV_SQRT3 = 1/√3 ≈ 0.57735，用于将电压限制在SVPWM线性调制区（最大不失真电压为母线电压/√3）
    // 观测器增益与母线电压成正比，以适应不同电压下的反电动势变化
    MC.SMO.Gain = MC.Sample.BusReal * INV_SQRT3;

    // ============================================================================
    // 2. 获取目标速度进行T型加减速处理
    // ============================================================================
    // 目标速度赋值给加减速结构体，并调用T型斜坡发生器
    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);

    // ============================================================================
    // 3. 开环电角度发生器（用于强拖阶段及初始角度）
    // ============================================================================
    // 将T型加减速输出的速度（rpm）作为电角速度设定值
    MC.EAngle.ElectricalAngleSpdSet = MC.TAccDec.SpeedOut;
    // 积分生成开环电角度（标幺值0~1）
    Electrical_Angle_Generator(&MC.EAngle);

    // ============================================================================
    // 4. 滑模观测器（SMO）计算：观测反电动势（Ealpha, Ebeta）
    // ============================================================================
    // 设置观测器参数：定子电阻、d轴电感、当前电流和电压（静止坐标系）
    MC.SMO.Rs = MC.Identify.Rs;
    MC.SMO.Ld = MC.Identify.Ld;
    MC.SMO.Ialpha = MC.Foc.Ialpha;
    MC.SMO.Ibeta  = MC.Foc.Ibeta;
    MC.SMO.Ualpha = MC.Foc.Ualpha;
    MC.SMO.Ubeta  = MC.Foc.Ubeta;
    // 执行滑模观测器运算，输出估算的反电动势（Ealpha, Ebeta）及滤波值
    SMO_Calculate(&MC.SMO);

    // ============================================================================
    // 5. 锁相环（PLL）处理：从反电动势提取电角度和角速度
    // ============================================================================
    MC.SPLL.Dir = MC.TAccDec.SpeedOutDir;            // 速度方向（用于PLL方向判断）
    MC.SPLL.Ain = MC.SMO.EalphaForeLPF;              // 滤波后的Alpha轴反电动势
    MC.SPLL.Bin = MC.SMO.EbetaForeLPF;               // 滤波后的Beta轴反电动势
    PLL_Calculate(&MC.SPLL);                         // PLL计算，输出电角度（EThetaPU）和角速度
    // 根据PLL输出的电角度计算正余弦值，用于后续Park变换
    Calculate_Sin_Cos(MC.SPLL.EThetaPU, &MC.SPLL.SinVal, &MC.SPLL.CosVal);

    // ============================================================================
    // 6. 开闭环切换逻辑计算（强拖 -> 观测器闭环）
    // ============================================================================
    // 记录开环给定速度（来自T型加减速）、闭环观测速度（来自PLL）、开环角度、闭环角度、运动状态
    MC.StrongDragToObs.OpenSpeed = MC.TAccDec.SpeedOut;
    MC.StrongDragToObs.CloseSpeed = MC.SPLL.WeForeLPF / TWO_PI * 60.0f;   // 电角速度(rad/s) -> rpm
    MC.StrongDragToObs.ThetaRef = MC.EAngle.ElectricalAngleSetPU;         // 开环参考角度
    MC.StrongDragToObs.ThetaObs = MC.SPLL.EThetaPU;                       // 闭环观测角度
    MC.StrongDragToObs.MotionState = MC.TAccDec.MotionState;              // 当前加减速状态
    // 核心切换函数：根据速度差、角度差等条件判断应处于开环还是闭环，并处理平滑过渡
    Strong_Drag_To_Observer_Cal(&MC.StrongDragToObs);
    
    MC.Speed.MechanicalSpeed = MC.StrongDragToObs.CloseSpeed / POLEPAIRS;   //用于屏幕显示转速

    // ============================================================================
    // 7. 切换异常处理和恢复机制
    // ============================================================================
    if(MC.StrongDragToObs.ErrFlag != 0)           // 发生切换错误（例如观测器失锁）
    {
        if(MC.StrongDragToObs.ErrTimes == 0)      // 第一次进入错误状态
        {
            // 保存当前目标速度，然后将目标速度和所有累积量清零（紧急停车）
            MC.StrongDragToObs.LastTargetSpeed = MC.TAccDec.TargetSpeed;
            MC.TAccDec.TargetSpeed = 0;
            MC.TAccDec.SpeedOut = 0;
            MC.TAccDec.SpeedTargetIncrement = 0;
            MC.TAccDec.SpeedChangeIncrement = 0;
            // 强制切换到开环模式（强拖）
            MC.StrongDragToObs.GeneralMode = OPEN_LOOP;
        }
        // 错误计数递增
        MC.StrongDragToObs.ErrTimes++;
        // 如果错误持续超过5000个控制周期（时间取决于Ts），则尝试恢复
        if(MC.StrongDragToObs.ErrTimes >= 5000)
        {
            MC.StrongDragToObs.ErrTimes = 0;          // 清零错误计数
            MC.StrongDragToObs.ErrFlag = 0;           // 清除错误标志
            MC.TAccDec.TargetSpeed = MC.StrongDragToObs.LastTargetSpeed; // 恢复原目标速度
        }
    }
    else
    {
        // 无错误时，若当前为闭环模式，则将电角度发生器输出替换为观测器输出的角度（闭环角度）
        if(MC.StrongDragToObs.GeneralMode == CLOSE_LOOP)
        {
            MC.EAngle.ElectricalAngleSetPU = MC.SPLL.EThetaPU;
        }
    }

    // ============================================================================
    // 8. 速度环PID控制（按分频因子执行）
    // ============================================================================
    MC.Speed.SpeedCalculateCnt++;
    if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)   // 每SPEED_DIVISION_FACTOR次执行一次
    {
        MC.Speed.SpeedCalculateCnt = 0;
        // 速度环目标值：T型加减速输出的速度（rpm）
        MC.SpdPid.Ref = MC.TAccDec.SpeedOut;
        // 速度环反馈值：PLL观测的电角速度转换为机械速度（rpm）
        MC.SpdPid.Fbk = MC.SPLL.WeForeLPF / TWO_PI * 60.0f;
        // 仅在闭环模式下执行速度环PID（开环时速度环不调节，直接使用强拖电流）
        if(MC.StrongDragToObs.GeneralMode == CLOSE_LOOP)
        {
            PID_Control(&MC.SpdPid);   // 速度环调节，输出作为Iq参考
            // 若刚刚从开环切换到闭环，需要将速度环的积分和输出初始化为开环时的电流值，
            // 避免切换时电流突变。
            if(MC.StrongDragToObs.LastGeneralMode == OPEN_LOOP)
            {
                MC.SpdPid.Integrate = MC.StrongDragToObs.OpenCurr;
                MC.SpdPid.Out = MC.StrongDragToObs.OpenCurr;
            }
        }
    }
    // 保存本次模式，供下一次判断切换边缘使用
    MC.StrongDragToObs.LastGeneralMode = MC.StrongDragToObs.GeneralMode;

    // ============================================================================
    // 9. 电流采样及Clark变换
    // ============================================================================
    MC.Foc.Iu = MC.Sample.IuReal;
    MC.Foc.Iv = MC.Sample.IvReal;
    Clark_Transform(&MC.Foc);            // Iu,Iv -> Ialpha,Ibeta

    // ============================================================================
    // 10. 选择Park变换使用的电角度（开环角度或闭环观测角度）
    // ============================================================================
    if(MC.StrongDragToObs.GeneralMode == OPEN_LOOP)
    {
        // 开环模式：使用电角度发生器产生的角度
        Calculate_Sin_Cos(MC.EAngle.ElectricalAngleSetPU, &MC.Foc.SinVal, &MC.Foc.CosVal);
    }
    else
    {
        // 闭环模式：使用PLL观测器输出的电角度
        Calculate_Sin_Cos(MC.SPLL.EThetaPU, &MC.Foc.SinVal, &MC.Foc.CosVal);
    }

    // Park变换：Ialpha,Ibeta -> Id,Iq（使用上面计算的正余弦）
    Park_Transform(&MC.Foc);

    // ============================================================================
    // 11. Id/Iq低通滤波
    // ============================================================================
    MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor);
    MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor);

    // ============================================================================
    // 12. 设置电流环的参考值（根据开环/闭环模式不同）
    // ============================================================================
    if(MC.StrongDragToObs.GeneralMode == OPEN_LOOP)
    {
        // 开环强拖模式：速度环不工作，Iq参考值为固定的强拖电流（OpenCurr），Id参考值为0
        // 如果目标速度绝对值小于10 rpm，则停止输出电流（关闭强拖）
        if(fabsf(MC.SpdPid.Ref) < 10)
        {
            MC.SpdPid.Integrate = 0;    // 清零速度环积分
            MC.SpdPid.Out = 0;
            MC.IqPid.Ref = 0;           // Iq参考为0，电机无力矩
            MC.SMO.EalphaFore = 0;
            MC.SMO.EbetaFore = 0;
            MC.SPLL.WeFore = 0;
        }
        else
        {
            MC.IqPid.Ref = MC.StrongDragToObs.OpenCurr;   // 固定强拖电流
            MC.IdPid.Ref = 0;                             // Id参考为0（不励磁）
        }
    }
    else
    {
        // 闭环模式：Iq参考来自速度环输出，Id参考为0
        MC.IqPid.Ref = MC.SpdPid.Out;
        MC.IdPid.Ref = 0;
    }

    // 设置电流环反馈值（滤波后的Id/Iq）
    MC.IqPid.Fbk = MC.Foc.IqLPF;
    MC.IdPid.Fbk = MC.Foc.IdLPF;

    // 执行电流环PID调节
    PID_Control(&MC.IqPid);            // 输出Uq
    PID_Control(&MC.IdPid);            // 输出Ud

    // 更新电压指令
    MC.Foc.Uq = MC.IqPid.Out;
    MC.Foc.Ud = MC.IdPid.Out;

    // 反Park变换：Ud,Uq -> Ualpha,Ubeta（使用与Park相同的电角度正余弦）
    IPark_Transform(&MC.Foc);

    // ------------------------------------------------------------------------
    // 13. 更新母线电压（直流母线电压采样值）
    // ------------------------------------------------------------------------
    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // ------------------------------------------------------------------------
    // 14. SVPWM调制：生成三相逆变器的PWM占空比
    // ------------------------------------------------------------------------
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

/*高频注入（HFI）电流闭环控制（无传感器低速/零速）*/
void Hfi_Current_Close_Loop(void)
{
    // ============================================================================
    // 1. 电流采样与 Clark 变换（三相静止 -> 两相静止）
    // ============================================================================
    MC.Foc.Iu = MC.Sample.IuReal;        // U相电流实际值（安培）
    MC.Foc.Iv = MC.Sample.IvReal;        // V相电流实际值
    Clark_Transform(&MC.Foc);            // 输出 Ialpha, Ibeta

    // ============================================================================
    // 2. Park 变换（两相静止 -> 旋转坐标系）
    //    使用当前锁相环输出的电角度（HPLL.EThetaPU）进行变换
    // ============================================================================
    // 根据锁相环估算的电角度计算正余弦值
    Calculate_Sin_Cos(MC.HPLL.EThetaPU, &MC.Foc.SinVal, &MC.Foc.CosVal);
    // Park变换：Ialpha,Ibeta -> Id,Iq（在估算的旋转坐标系下）
    Park_Transform(&MC.Foc);

    // ============================================================================
    // 3. 将电流值传递给高频注入模块（HFI）
    // ============================================================================
    MC.HFI.Id = MC.Foc.Id;               // d轴电流（实际反馈）
    MC.HFI.Iq = MC.Foc.Iq;               // q轴电流
    MC.HFI.Ialpha = MC.Foc.Ialpha;       // α轴电流
    MC.HFI.Ibeta  = MC.Foc.Ibeta;        // β轴电流

    // 执行高频注入算法：注入高频电压，提取负序电流分量，输出 IdRef（若需要）、
    // IalphaOut/IbetaOut（用于PLL）、NSDFlag（极性检测标志）等
    HFI_Calculate(&MC.HFI);

    // ============================================================================
    // 4. 极性检测（NSD）处理
    //    当检测到磁极方向（N/S）时，对锁相环累积角度进行π弧度（180°）修正
    // ============================================================================
    if(MC.HFI.NSDFlag == 0)              // 如果尚未完成极性检测（NSDFlag=0表示未确定）
    {
        // 将高频注入计算的 Id 参考值（用于磁极判断时的励磁电流）赋值给 Id 环目标
        MC.IdPid.Ref = MC.HFI.IdRef;
    }
    else
    {
        MC.IqPid.Ref = 0.08f;
    }
    // 如果 NSD 输出为 1，表示需要翻转估算角度（即判断出实际N极与估算方向相反）
    if(MC.HFI.NSDOut == 1)
    {
        MC.HFI.NSDOut = 0;               // 清除标志，只执行一次
        // 锁相环累积角度加上 π（180°），实现磁极校正
        MC.HPLL.ThetaFore += ONE_PI;
        // 将角度归一化到 [0, 2π) 范围
        if(MC.HPLL.ThetaFore > TWO_PI)
        {
            MC.HPLL.ThetaFore -= TWO_PI;
        }
    }

    // ============================================================================
    // 5. 锁相环（PLL）处理：从高频注入输出的电流信号中提取转子电角度和速度
    // ============================================================================
    // 根据锁相环当前角度计算正余弦（用于内部PLL计算）
    Calculate_Sin_Cos(MC.HPLL.EThetaPU, &MC.HPLL.SinVal, &MC.HPLL.CosVal);
    // 设置PLL的输入信号：Ain = Ibeta_out, Bin = -Ialpha_out（通常为高频负序电流分量）
    MC.HPLL.Ain = MC.HFI.IbetaOut;
    MC.HPLL.Bin = -MC.HFI.IalphaOut;
    // 执行PLL计算，输出电角度（EThetaPU）和电角速度（We）
    PLL_Calculate(&MC.HPLL);

    // ============================================================================
    // 6. 电流环PID控制（使用高频注入提取的基波电流分量）
    // ============================================================================
    // 注：HFI.IdBase / IqBase 是通过滤波器从总电流中分离出的基波电流（用于闭环控制）
    MC.IqPid.Fbk = MC.HFI.IqBase;        // q轴基波电流反馈
    MC.IdPid.Fbk = MC.HFI.IdBase;        // d轴基波电流反馈
    // 执行电流环PI调节，输出 Ud、Uq（基波电压指令）
    PID_Control(&MC.IqPid);
    PID_Control(&MC.IdPid);

    // ============================================================================
    // 7. 电压指令合成与反Park变换
    // ============================================================================
    MC.Foc.Uq = MC.IqPid.Out;            // q轴电压（基波）
    // d轴电压 = Id环输出 + 高频注入电压（Uin），将高频电压叠加到Ud上
    if(MC.HFI.Dir == 1){
        MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;   // +1.4V
    }else{
        MC.Foc.Ud = MC.IdPid.Out - MC.HFI.Uin;   // -1.4V
    }
    // 反Park变换：Ud/Uq -> Ualpha/Ubeta（使用锁相环估算的电角度）
    IPark_Transform(&MC.Foc);
    // ------------------------------------------------------------------------
    // 8. 更新母线电压（直流母线电压采样值）
    // ------------------------------------------------------------------------
    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // ------------------------------------------------------------------------
    // 9. SVPWM调制：生成三相逆变器的PWM占空比
    // ------------------------------------------------------------------------
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

/*高频注入（HFI）速度-电流双闭环控制（无传感器低速段*/
void Hfi_Speed_Current_Loop(void)
{
    // ============================================================================
    // 1. T型加减速处理（速度斜坡生成）
    // ============================================================================
    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);      // 输出平滑的速度参考值 SpeedOut

    // ============================================================================
    // 2. 速度环PID控制（按分频因子降频执行）
    // ============================================================================
    MC.Speed.SpeedCalculateCnt++;
    // 每 SPEED_DIVISION_FACTOR 个控制周期执行一次速度环（降低计算负担）
    if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)
    {
        MC.Speed.SpeedCalculateCnt = 0;
        // 速度环目标值：T型加减速输出的速度（rpm）
        MC.SpdPid.Ref = MC.TAccDec.SpeedOut;
        // 速度环反馈值：HPLL（高频注入锁相环）估算的电角速度（rad/s）转换为机械速度（rpm）
        MC.SpdPid.Fbk = MC.HPLL.WeForeLPF / TWO_PI * 60.0f;
        // 执行速度环PID调节，输出作为Iq电流环的参考值
        PID_Control(&MC.SpdPid);
        MC.IqPid.Ref = MC.SpdPid.Out;
    }

    // ============================================================================
    // 3. 电流采样与Clark变换
    // ============================================================================
    MC.Foc.Iu = MC.Sample.IuReal;
    MC.Foc.Iv = MC.Sample.IvReal;
    Clark_Transform(&MC.Foc);           // Iu,Iv -> Ialpha,Ibeta

    // ============================================================================
    // 4. Park变换（两相静止 -> 旋转坐标系）
    //    使用高频注入锁相环（HPLL）估算的电角度进行变换
    // ============================================================================
    Calculate_Sin_Cos(MC.HPLL.EThetaPU, &MC.Foc.SinVal, &MC.Foc.CosVal);
    Park_Transform(&MC.Foc);            // Ialpha,Ibeta -> Id,Iq（在估算坐标系下）

    // ============================================================================
    // 5. 高频注入算法计算
    // ============================================================================
    // 将电流值传递给HFI模块
    MC.HFI.Id = MC.Foc.Id;
    MC.HFI.Iq = MC.Foc.Iq;
    MC.HFI.Ialpha = MC.Foc.Ialpha;
    MC.HFI.Ibeta  = MC.Foc.Ibeta;
    // 执行高频注入核心算法：注入高频电压，提取负序电流和基波电流分量，
    // 输出 IdRef（极性检测用）、IalphaOut/IbetaOut（用于PLL）、NSDFlag、NSDOut等
    HFI_Calculate(&MC.HFI);

    // ============================================================================
    // 6. 极性检测（NSD）与角度校正
    // ============================================================================
    // 如果尚未完成极性检测（NSDFlag==0），则使用HFI计算的IdRef作为Id环目标值，
    // 施加d轴电流脉冲来判断磁极方向。
    if(MC.HFI.NSDFlag == 0)
    {
        MC.IdPid.Ref = MC.HFI.IdRef;
    }
    // 如果NSD输出为1，表示需要翻转估算角度（实际N极与估算方向相反）
    if(MC.HFI.NSDOut == 1)
    {
        MC.HFI.NSDOut = 0;              // 清除标志，只执行一次
        // 锁相环累积角度加上π（180°）进行校正
        MC.HPLL.ThetaFore += ONE_PI;
        // 角度归一化到 [0, 2π)
        if(MC.HPLL.ThetaFore > TWO_PI)
        {
            MC.HPLL.ThetaFore -= TWO_PI;
        }
    }

    // ============================================================================
    // 7. 锁相环（PLL）处理：从高频响应中提取电角度和速度
    // ============================================================================
    // 根据当前锁相环电角度计算正余弦（用于PLL内部计算）
    Calculate_Sin_Cos(MC.HPLL.EThetaPU, &MC.HPLL.SinVal, &MC.HPLL.CosVal);
    // 设置PLL输入信号：Ain = Ibeta_out, Bin = -Ialpha_out（高频负序电流分量）
    MC.HPLL.Ain = MC.HFI.IbetaOut;
    MC.HPLL.Bin = -MC.HFI.IalphaOut;
    // 执行PLL，输出估算的电角度（EThetaPU）和电角速度（We）
    PLL_Calculate(&MC.HPLL);

    // ============================================================================
    // 8. 电流环PID控制（使用HFI提取的基波电流分量）
    // ============================================================================
    MC.IqPid.Fbk = MC.HFI.IqBase;       // q轴基波电流反馈（转矩电流）
    MC.IdPid.Fbk = MC.HFI.IdBase;       // d轴基波电流反馈（励磁电流）
    PID_Control(&MC.IqPid);             // Iq环PI调节，输出Uq
    PID_Control(&MC.IdPid);             // Id环PI调节，输出Ud

    // ============================================================================
    // 9. 电压指令合成与反Park变换
    // ============================================================================
    MC.Foc.Uq = MC.IqPid.Out;           // q轴电压指令（基波）
    // d轴电压指令 = Id环输出 ± 高频注入电压（Uin），叠加方向由HFI方向标志Dir决定
    // Dir==1时加，Dir==其他时减（用于匹配注入信号的正负序）
    if(MC.HFI.Dir == 1){
        MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;
    }else{
        MC.Foc.Ud = MC.IdPid.Out - MC.HFI.Uin;
    }
    // 反Park变换：Ud/Uq -> Ualpha/Ubeta（使用HPLL估算的电角度）
    IPark_Transform(&MC.Foc);
    // ------------------------------------------------------------------------
    // 10. 更新母线电压（直流母线电压采样值）
    // ------------------------------------------------------------------------
    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // ------------------------------------------------------------------------
    // 11. SVPWM调制：生成三相逆变器的PWM占空比
    // ------------------------------------------------------------------------
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

/*高频注入（HFI）+ 滑模观测器（SMO）混合无传感器速度-电流双闭环控制*/
void Hfi_Smo_Speed_Current_Loop(void)
{
    // ============================================================================
    // 1. 动态更新滑模观测器增益（基于母线电压）
    // ============================================================================
    // INV_SQRT3 = 1/√3 ≈ 0.57735，将增益与母线电压关联，使观测器适应不同电压等级
    // 增益越大，观测器收敛越快，但可能引入抖振；此处根据母线电压线性调整。
    MC.SMO.Gain = MC.Sample.BusReal * INV_SQRT3;

    // ============================================================================
    // 2. T型加减速处理（速度斜坡生成）
    // ============================================================================
    // 目标速度已在外部设定（如电位器或通信指令），存储于 MC.Speed.MechanicalSpeedSet
    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);          // 输出平滑的速度参考值 SpeedOut (rpm)

    // ============================================================================
    // 3. 滑模观测器（SMO）运算：估算反电动势
    // ============================================================================
    // 设置观测器参数（定子电阻、d轴电感）
    MC.SMO.Rs = MC.Identify.Rs;
    MC.SMO.Ld = MC.Identify.Ld;
    // 输入当前周期的电流和电压（静止坐标系）
    MC.SMO.Ialpha = MC.Foc.Ialpha;
    MC.SMO.Ibeta  = MC.Foc.Ibeta;
    MC.SMO.Ualpha = MC.Foc.Ualpha;
    MC.SMO.Ubeta  = MC.Foc.Ubeta;
    // 执行滑模观测器计算，输出估算的反电动势 Ealpha, Ebeta 及其滤波值
    SMO_Calculate(&MC.SMO);

    // ============================================================================
    // 4. 锁相环（PLL）从 SMO 反电动势中提取电角度和角速度
    // ============================================================================
    MC.SPLL.Dir = MC.TAccDec.SpeedOutDir;   // 速度方向（用于PLL正反转判断）
    MC.SPLL.Ain = MC.SMO.EalphaForeLPF;     // 滤波后的 α 轴反电动势
    MC.SPLL.Bin = MC.SMO.EbetaForeLPF;      // 滤波后的 β 轴反电动势
    PLL_Calculate(&MC.SPLL);                // PLL计算，输出电角度 SPLL.EThetaPU 和电角速度
    // 根据 PLL 输出的电角度计算正余弦，供后续可能的使用
    Calculate_Sin_Cos(MC.SPLL.EThetaPU, &MC.SPLL.SinVal, &MC.SPLL.CosVal);

    // ============================================================================
    // 5. 高频注入（HFI）运算（仅在使能时执行）
    // ============================================================================
    if(MC.HFI.Enable == 1)
    {
        // 将电流值传递给 HFI 模块（Id/Iq 为旋转坐标系下电流，Ialpha/Ibeta 为静止坐标系）
        MC.HFI.Id = MC.Foc.Id;
        MC.HFI.Iq = MC.Foc.Iq;
        MC.HFI.Ialpha = MC.Foc.Ialpha;
        MC.HFI.Ibeta  = MC.Foc.Ibeta;
        // 执行高频注入核心算法：注入高频电压，提取负序电流和基波电流分量，
        // 输出 IdRef（极性检测用）、IalphaOut/IbetaOut（用于PLL）、NSDFlag、NSDOut等
        HFI_Calculate(&MC.HFI);

        // --- 极性检测（NSD）处理 ---
        // NSDFlag == 0 表示尚未完成极性检测（即未确定磁极方向）
        if(MC.HFI.NSDFlag == 0)
        {
            // 使用 HFI 计算的 IdRef 作为 Id 环目标值，施加 d 轴电流脉冲来判断磁极方向
            MC.IdPid.Ref = MC.HFI.IdRef;
        }
        // NSDOut == 1 表示需要翻转估算角度（实际 N 极与估算方向相差 180°）
        if(MC.HFI.NSDOut == 1)
        {
            MC.HFI.NSDOut = 0;             // 清除标志，只执行一次
            // 锁相环累积角度加上 π（180°）进行校正
            MC.HPLL.ThetaFore += ONE_PI;
            // 角度归一化到 [0, 2π)
            if(MC.HPLL.ThetaFore > TWO_PI)
            {
                MC.HPLL.ThetaFore -= TWO_PI;
            }
        }

        // --- HFI 专用锁相环（HPLL）从高频响应中提取角度和速度 ---
        // 根据当前 HPLL 电角度计算正余弦（用于 PLL 内部运算）
        Calculate_Sin_Cos(MC.HPLL.EThetaPU, &MC.HPLL.SinVal, &MC.HPLL.CosVal);
        // 设置 PLL 输入信号：Ain = Ibeta_out, Bin = -Ialpha_out（高频负序电流分量）
        MC.HPLL.Ain = MC.HFI.IbetaOut;
        MC.HPLL.Bin = -MC.HFI.IalphaOut;
        // 执行 PLL，输出估算的电角度（HPLL.EThetaPU）和电角速度（HPLL.We）
        PLL_Calculate(&MC.HPLL);
    }

    // ============================================================================
    // 6. HFI 与 SMO 观测器切换逻辑
    // ============================================================================
    // 记录两种观测器估算的速度（rpm）和角度（标幺值）
    MC.HfiToObs.HfiEleSpeed = MC.HPLL.WeForeLPF / TWO_PI * 60.0f;  // HFI 速度（rad/s → rpm）
    MC.HfiToObs.HfiTheta   = MC.HPLL.EThetaPU;                     // HFI 角度（标幺值 0~1）
    MC.HfiToObs.ObsEleSpeed = MC.SPLL.WeForeLPF / TWO_PI * 60.0f;  // SMO 速度（rpm）
    MC.HfiToObs.ObsTheta    = MC.SPLL.EThetaPU;                    // SMO 角度（标幺值）
    MC.HfiToObs.SpeedRef    = MC.TAccDec.SpeedOut;                 // 当前速度参考值
    // 根据速度区间、角度误差等条件，计算最终输出的电角度（ThetaOut）和速度（EleSpeedOut）
    Hfi_TO_Obsever_Cal(&MC.HfiToObs);

    MC.Speed.MechanicalSpeed = MC.HfiToObs.EleSpeedOut / POLEPAIRS;   //用于屏幕显示转速
    
    // ============================================================================
    // 7. 速度环PID控制（按分频因子降频执行）
    // ============================================================================
    MC.Speed.SpeedCalculateCnt++;
    // 每 SPEED_DIVISION_FACTOR 个控制周期执行一次速度环（降低计算负担）
    if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)
    {
        MC.Speed.SpeedCalculateCnt = 0;
        // 速度环目标值：T型加减速输出的速度（rpm）
        MC.SpdPid.Ref = MC.TAccDec.SpeedOut;
        // 速度环反馈值：经切换逻辑后输出的观测速度（rpm）
        MC.SpdPid.Fbk = MC.HfiToObs.EleSpeedOut;
        // 执行速度环PI调节，输出作为 Iq 电流环的参考值
        PID_Control(&MC.SpdPid);
        MC.IqPid.Ref = MC.SpdPid.Out;
    }

    // ============================================================================
    // 8. FOC 电流环：采样、坐标变换、滤波、PID、电压重构
    // ============================================================================

    // 8.1 电流采样与 Clark 变换（三相静止 -> 两相静止）
    MC.Foc.Iu = MC.Sample.IuReal;
    MC.Foc.Iv = MC.Sample.IvReal;
    Clark_Transform(&MC.Foc);               // 输出 Ialpha, Ibeta

    // 8.2 Park 变换（两相静止 -> 旋转坐标系）
    // thetafrac 函数用于取小数部分，将角度归一化到 [0,1) 范围
    Calculate_Sin_Cos(MC.HfiToObs.ThetaOut, &MC.Foc.SinVal, &MC.Foc.CosVal);
    Park_Transform(&MC.Foc);                // 输出 Id, Iq（在实际估算坐标系下）

    // 8.3 Id/Iq 低通滤波（一阶滤波，抑制采样噪声）
    MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor);
    MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor);

    // 8.4 选择电流反馈源
    if(MC.HFI.Enable == 0)
    {
        // 未使能 HFI（即纯 SMO 模式）：使用 FOC 变换后的 Id/Iq 经低通滤波后的值
        MC.IqPid.Fbk = MC.Foc.IqLPF;
        MC.IdPid.Fbk = MC.Foc.IdLPF;
    }
    else
    {
        // 使能 HFI（低速段）：使用 HFI 算法提取的基波电流分量（已滤除高频成分）
        MC.IqPid.Fbk = MC.HFI.IqBase;
        MC.IdPid.Fbk = MC.HFI.IdBase;
    }

    // 8.5 执行电流环PID调节
    PID_Control(&MC.IqPid);                 // Iq 环（转矩电流），输出 Uq
    PID_Control(&MC.IdPid);                 // Id 环（励磁电流），输出 Ud

    // 8.6 电压指令合成
    MC.Foc.Uq = MC.IqPid.Out;               // q轴电压指令（基波）
    if(MC.HFI.Enable == 1)
    {
        // 使能 HFI 时，需要将高频注入电压叠加到 Ud 上
        if(MC.HFI.Dir == 0)
        {
            MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;   // 正向叠加
        }
        else
        {
            MC.Foc.Ud = MC.IdPid.Out - MC.HFI.Uin;   // 反向叠加
        }
    }
    else
    {
        // 未使能 HFI 时，Ud 仅为 Id 环输出
        MC.Foc.Ud = MC.IdPid.Out;
    }

    // 8.7 反Park变换：旋转坐标系电压 -> 两相静止坐标系电压（Ualpha, Ubeta）
    // 使用与 Park 变换相同的电角度（ThetaOut + ThetaOffset）
    IPark_Transform(&MC.Foc);

    // ------------------------------------------------------------------------
    // 9. 更新母线电压（直流母线电压采样值）
    // ------------------------------------------------------------------------
    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // ------------------------------------------------------------------------
    // 10. SVPWM调制：生成三相逆变器的PWM占空比
    // ------------------------------------------------------------------------
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

/*强拖模式到观测器闭环模式的切换逻辑计算*/
void Strong_Drag_To_Observer_Cal(STRONG_DRAG_TO_OBSERVER *p)
{
    // ============================================================================
    // 1. 计算角度误差（ThetaErr），并处理角度过零点跳变（标幺值范围0~1）
    // ============================================================================
    p->ThetaErr = p->ThetaRef - p->ThetaObs;          // 开环参考角度 - 闭环观测角度
    // 如果误差 > 0.5，说明实际误差应为负方向（因为角度是循环的），减去1.0
    p->ThetaErr = p->ThetaErr > 0.5f ? p->ThetaErr - 1.0f : p->ThetaErr;
    // 如果误差 < -0.5，说明实际误差应为正方向，加上1.0
    p->ThetaErr = p->ThetaErr < -0.5f ? p->ThetaErr + 1.0f : p->ThetaErr;
    // 最终 ThetaErr 范围在 [-0.5, 0.5] 之间，表示角度差的标幺值（±180°电角度）

    // ============================================================================
    // 2. 计算速度误差，并进行一阶低通滤波（平滑速度误差）
    // ============================================================================
    p->SpeedErr = p->OpenSpeed - p->CloseSpeed;       // 开环给定速度 - 闭环观测速度（rpm）
    // 一阶低通滤波：Y(n) = 0.1 * X(n) + 0.9 * Y(n-1)，滤波系数0.1
    p->SpeedErrFlt += 0.1f * (p->SpeedErr - p->SpeedErrFlt);

    // ============================================================================
    // 3. 取开环速度和闭环速度的绝对值（用于比较大小，忽略方向）
    // ============================================================================
    p->EleOpenSpeedAbs = fabsf(p->OpenSpeed);
    p->EleCloseSpeedAbs = fabsf(p->CloseSpeed);

    // ============================================================================
    // 4. 更新角度误差检测计数器（CheckCnt）
    //    当角度误差绝对值 ≤ 0.05（即18°电角度以内）时，计数器递增；
    //    否则计数器递减。该计数器用于判断角度是否已收敛。
    // ============================================================================
    p->CheckCnt = fabsf(p->ThetaErr) <= 0.05f ? p->CheckCnt + 1 : p->CheckCnt - 1;

    // ============================================================================
    // 5. 如果当前处于闭环模式，累加闭环运行时间计数器（带饱和限制）
    // ============================================================================
    if(p->GeneralMode == CLOSE_LOOP)
    {
        p->CloseRunTime++;
        p->CloseRunTime = Sat(p->CloseRunTime, 0, 40000);   // 限制在0~40000之间
    }

    // ============================================================================
    // 6. 动态调整开环强拖电流（OpenCurr）的大小
    //    根据运动状态、速度区间等，增加或减小电流，以优化切换时机和平滑性。
    // ============================================================================
    // 电流每次变化的步长（基于最大开环电流的0.03%）
    float curr_change_unit = p->OpenCurrMax * 0.0003f;

    // 情况1：非减速状态 && 开环速度绝对值 ≥ 切换速度阈值（如4200rpm）&& 角度未完全收敛（CheckCnt≤30）
    //        此时逐渐减小开环电流，为闭环切入做准备（降低强拖力矩，让观测器接管）
    if(p->MotionState != DECELERATE && p->EleOpenSpeedAbs >= p->OpenToCloseSwitchSpeed && p->CheckCnt <= 30)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) - curr_change_unit), 0.08f, p->OpenCurrMax);
    }
    // 情况2：非减速状态 && 开环速度绝对值 < 切换速度阈值（即速度还较低）
    //        此时逐渐增大开环电流，确保有足够力矩加速到切换速度
    else if(p->MotionState != DECELERATE && p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) + 0.01f), 0.05f, p->OpenCurrMax);
    }

    // 情况3：减速状态 && 开环速度绝对值在 [CloseMinSpeed, OpenToCloseSwitchSpeed+5] 区间内
    //        此时略微增加开环电流（小步长），防止减速时过早失步
    if(p->MotionState == DECELERATE && p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed + 5 && p->EleOpenSpeedAbs > p->CloseMinSpeed)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) + 0.001f), 0.05f, p->OpenCurrMax);
    }
    // 情况4：减速状态 && 开环速度绝对值低于最低闭环速度阈值（如2000rpm）
    //        此时逐渐减小开环电流，避免低速下电流过大导致抖动
    else if(p->MotionState == DECELERATE && p->EleOpenSpeedAbs < p->CloseMinSpeed)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) - curr_change_unit), 0.08f, p->OpenCurrMax);
    }

    // ============================================================================
    // 7. 根据开环速度的方向，确定开环电流的符号（正反转方向）
    // ============================================================================
    if(p->OpenSpeed < 0)
    {
        p->OpenCurr = -fabsf(p->OpenCurr);   // 负速度方向，电流取负
    }

    // ============================================================================
    // 8. 切换条件判断与错误检测
    // ============================================================================

    // 条件A：当前处于闭环模式 && 闭环运行时间超过20000个周期 && 闭环速度绝对值 ≤ 最低闭环速度
    //        说明在闭环状态下速度过低，可能失步或观测器失效，触发错误标志。
    if(p->GeneralMode == CLOSE_LOOP && p->CloseRunTime > 20000 && (p->EleCloseSpeedAbs <= p->CloseMinSpeed))
    {
        p->ErrFlag = 1;           // 置位错误标志
        p->CloseRunTime = 0;      // 清零闭环运行计时
        p->ErrCnt++;              // 错误计数累加
    }
    // 条件B：角度检测计数器 > 40（即角度误差持续小于0.05超过40个周期）&& 开环速度 ≥ 切换速度阈值
    //        满足条件则切换到闭环模式（观测器接管）
    else if(p->CheckCnt > 40 && p->EleOpenSpeedAbs >= p->OpenToCloseSwitchSpeed)
    {
        p->ErrFlag = 0;           // 清除错误标志
        p->GeneralMode = CLOSE_LOOP;  // 切换到闭环
    }
    // 条件C：开环速度绝对值 < 闭环切换到开环的速度阈值（如3000rpm）
    //        此时强制切回开环模式（速度过低，观测器可能不可靠）
    else if(p->EleOpenSpeedAbs < p->CloseToOpenSwitchSpeed)
    {
        p->CloseRunTime = 0;      // 清零闭环运行时间
        p->GeneralMode = OPEN_LOOP;   // 切换到开环
    }

    // ============================================================================
    // 9. 限制 CheckCnt 的范围（0~41）
    // ============================================================================
    if(p->CheckCnt > 40)
    {
        p->CheckCnt = 41;         // 上限41，使得 >40 条件成立
    }
    else if(p->CheckCnt < 0)
    {
        p->CheckCnt = 0;          // 下限0
    }
}

/*HFI（高频注入）与 SMO（滑模观测器）切换逻辑计算*/
void Hfi_TO_Obsever_Cal(HFI_TO_OBSERVER *p)
{
    // ============================================================================
    // 1. 计算速度绝对值与参考速度绝对值
    // ============================================================================
    p->HfiEleSpeedAbs = fabsf(p->HfiEleSpeed);     // HFI 估算速度绝对值（rpm）
    p->ObsEleSpeedAbs = fabsf(p->ObsEleSpeed);     // SMO 估算速度绝对值（rpm）
    p->SpeedRefAbs = fabsf(p->SpeedRef);           // 速度参考值绝对值（rpm）

    // ============================================================================
    // 2. 计算两种观测器的角度误差（ThetaErr），并处理角度周期边界（标幺值范围0~1）
    // ============================================================================
    p->ThetaErr = p->HfiTheta - p->ObsTheta;        // HFI 角度 - SMO 角度（标幺值范围0~1）
    // 若误差超过 0.5f，减去 1 使之落在 [0, 1) 范围
    p->ThetaErr = p->ThetaErr > 0.5f ? p->ThetaErr - 1.0f : p->ThetaErr;
    // 若误差超过 -0.5f，加上 1 使之落在 [0, 1) 范围
    p->ThetaErr = p->ThetaErr < -0.5f ? p->ThetaErr + 1.0f : p->ThetaErr;

    // ============================================================================
    // 3. 更新角度误差检测计数器（CheckCnt）
    //    当角度误差绝对值 ≤ 0.05 时，计数器递增；否则递减。
    //    用于判断两个观测器的角度是否已经接近（收敛）。
    // ============================================================================
    p->CheckCnt = p->ThetaErr <= 0.05f ? p->CheckCnt + 1 : p->CheckCnt - 1;

    // ============================================================================
    // 4. 根据速度区间进行模式选择和输出
    // ============================================================================

    // ---------- 情况1：速度极低（≤ SpeedMin）或参考速度极低 ----------
    // 此时必须使用 HFI（因为 SMO 在零/低速下不可靠），强制切换到 HFI 模式
    if((p->HfiEleSpeedAbs <= p->SpeedMin || p->SpeedRefAbs <= p->SpeedMin))
    {
        MC.HFI.Enable = 1;                      // 使能 HFI
        p->EleSpeedOut = p->HfiEleSpeed;        // 输出速度 = HFI 估算速度
        p->ThetaOut = p->HfiTheta;              // 输出角度 = HFI 估算角度
        p->ObsMode = HFI;                       // 当前观测器模式记录为 HFI
        MC.HFI.Uin -= 0.0002f;                  // 缓慢减小高频注入电压幅值（降低噪声）
    }
    // ---------- 情况2：中等速度区间（介于 SpeedMin 和 SpeedMax 之间）----------
    // 此区间为过渡区，根据当前观测器模式（HFI 或 SMO）和角度误差 CheckCnt 决定输出
    else if((p->SpeedRefAbs > p->SpeedMin && p->SpeedRefAbs < p->SpeedMax) ||
            (p->HfiEleSpeedAbs > p->SpeedMin && p->HfiEleSpeedAbs < p->SpeedMax &&
             p->ObsEleSpeedAbs > p->SpeedMin && p->ObsEleSpeedAbs < p->SpeedMax))
    {
        // ----- 当前模式为 HFI（正在从 HFI 向 SMO 过渡）-----
        if(p->ObsMode == HFI)
        {
            if(p->CheckCnt >= 100)              // 角度误差持续很小（收敛），可以切换到 SMO
            {
                p->EleSpeedOut = p->ObsEleSpeed;    // 输出 SMO 速度
                p->ThetaOut = p->ObsTheta;          // 输出 SMO 角度
                MC.HFI.Uin -= 0.0002f;              // 继续减小注入电压
            }
            else if(p->CheckCnt <= 30)          // 角度误差较大，仍使用 HFI
            {
                p->EleSpeedOut = p->HfiEleSpeed;
                p->ThetaOut = p->HfiTheta;
            }
            else                                // 中间状态，保持上一周期的输出（平滑过渡）
            {
                p->EleSpeedOut = p->EleSpeedOut;
                p->ThetaOut = p->ThetaOut;
            }
        }
        // ----- 当前模式为 SMO（正在从 SMO 向 HFI 过渡，例如减速）-----
        else
        {
            if(p->CheckCnt >= 100)              // 角度误差小，切换到 HFI（准备低速运行）
            {
                p->EleSpeedOut = p->HfiEleSpeed;
                p->ThetaOut = p->HfiTheta;
                MC.HFI.Uin -= 0.02f;            // 快速减小注入电压
            }
            else if(p->CheckCnt <= 30)          // 角度误差大，保持 SMO
            {
                p->EleSpeedOut = p->ObsEleSpeed;
                p->ThetaOut = p->ObsTheta;
                MC.HFI.Uin += 0.02f;            // 增加注入电压（为切换到 HFI 做准备）
            }
            else
            {
                p->EleSpeedOut = p->EleSpeedOut;
                p->ThetaOut = p->ThetaOut;
            }
        }
    }
    // ---------- 情况3：高速区间（超过 SpeedMax）----------
    // 此时 SMO 更可靠，强制使用 SMO，并根据速度条件决定是否禁用 HFI 或调整注入电压
    else
    {
        // 如果 SMO 速度远高于上限（> SpeedMax + 3*SpeedMid），完全禁用 HFI
        if(p->ObsEleSpeedAbs > (p->SpeedMax + p->SpeedMid * 3.0f))
        {
            MC.HFI.Enable = 0;                  // 关闭 HFI，减少计算负担
        }
        // 如果 SMO 速度回落到略低于上限，重新使能 HFI 并调整注入电压
        else if(p->ObsEleSpeedAbs < (p->SpeedMax + p->SpeedMid * 2.5f))
        {
            MC.HFI.Enable = 1;
            // 计算速度变化率（用于动态调整注入电压）
            p->SpeedChangeRate = fabsf(p->EleSpeedOut) - fabsf(p->SpeedLast);
            p->SpeedLast = p->EleSpeedOut;
            // 当速度急剧下降（减速变化率绝对值 > 56 rpm/周期），增大注入电压以增强位置跟踪
            if(p->SpeedChangeRate < 0 && fabsf(p->SpeedChangeRate) > 56.0f)
            {
                MC.HFI.Uin = p->HfiInjectMagMax;    // 设为最大注入电压
            }
            // 当速度平稳或缓慢变化，逐渐减小注入电压
            else if(p->SpeedChangeRate > 0 && fabsf(p->SpeedChangeRate) < 14.0f)
            {
                MC.HFI.Uin -= 0.0002f;
            }
        }
        else
        {
            // 保持原有 HFI 使能状态
            MC.HFI.Enable = MC.HFI.Enable;
        }
        // 高速下输出 SMO 的速度和角度
        p->EleSpeedOut = p->ObsEleSpeed;
        p->ThetaOut = p->ObsTheta;
        p->ObsMode = SMO;                       // 当前模式记录为 SMO
    }

    // ============================================================================
    // 5. 限制高频注入电压幅值（Uin）在最小值和最大值之间
    // ============================================================================
    MC.HFI.Uin = Sat(MC.HFI.Uin, p->HfiInjectMagMin, p->HfiInjectMagMax);

    // ============================================================================
    // 6. 限制 CheckCnt 的范围（0 ~ 121）
    // ============================================================================
    if(p->CheckCnt > 120)
    {
        p->CheckCnt = 121;          // 上限121，使得 ≥100 条件成立
    }
    else if(p->CheckCnt <= 0)
    {
        p->CheckCnt = 0;            // 下限0
    }
}
