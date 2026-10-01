#include "motor_identify.h"
#include "motor_publicdata.h"

/**
 * 函数功能: 电机参数辨识（电阻、电感）及编码器转子对齐
 * 输入参数: 无（所有参数通过全局结构体 MC.Identify、MC.Foc、MC.Sample、MC.Encoder 传递）
 * 返回参数: 无（结果存入 MC.Identify.Rs、MC.Identify.Ls 等，最终设置 MC.Identify.EndFlag）
 * 说    明: 
 *         1. 该函数通过三个子状态自动完成电机参数辨识和编码器校准：
 *            - RESISTANCE_IDENTIFICATION：直流伏安法测量定子电阻 Rs
 *            - INDUCTANCE_IDENTIFICATION：直流阶跃法测量定子电感 Ls（假设 Ld = Lq）
 *            - ENCODER_ROTOR_ALIGN：两次定位（90°和0°）校准编码器零位，检测旋转方向
 *         2. 该函数应在 PWM 中断中周期性调用，每次执行一步状态机，直到 EndFlag 置1。
 *         3. 辨识过程中电机轴会轻微转动，需确保负载脱离。
 *         4. 电阻辨识原理：向 d 轴注入直流电流（电角度=0），测量不同电压下的稳定电流，
 *            根据欧姆定律 Rs = ΔU / ΔI。
 *         5. 电感辨识原理：施加阶跃电压，测量电流从0上升到稳定值95%的时间，
 *            对于一阶 RL 电路，L = R * t / ln(20)。
 *         6. 编码器对齐：先拉转子到90°电角度，记录编码器值；再拉到0°，计算两次差值
 *            判断方向，并得到零点偏移。
 */
void Motor_Identify(void)
{
    /* 根据当前辨识阶段执行不同操作 */
    switch (MC.Identify.State)
    {
        /* ================= 阶段1：定子电阻辨识 ================= */
        case RESISTANCE_IDENTIFICATION:
        {
            /* ----- 子状态 Flag = 0：清空参数，准备第一次电流设定 ----- */
            if (MC.Identify.Flag == 0)
            {
                MC.Foc.Uq = 0;                 // 交轴电压清零（只施加直轴电压）
                MC.Foc.Ud = 0;                 // 直轴电压从0开始递增
                MC.Identify.Count = 0;          // 计数清零（本阶段未使用）
                MC.Identify.WaitTim = 0;        // 等待计时器清零
                MC.Identify.Flag = 1;           // 进入下一个子状态
            }

            /* ----- 子状态 Flag = 1：逐步增加 Ud 直到电流达到第一个目标值（0.6倍最大电流）----- */
            if (MC.Identify.Flag == 1)
            {
                // 电流判据：current = Iu * (1.5 * Ud / Ubus)
                // θ=0 时反Park输出 Ualpha=Ud、Ubeta=0，代入SVPWM公式（foc_drv.c）
                // 得有效矢量V1(100)的作用占空比 t1 = T1/PwmCycle = 1.5*Ud/Ubus（此时T2=0），
                // 故 current = Iu * t1，物理意义是平均直流母线电流：母线仅在V1作用期间
                // 向电机供出相电流Iu，V0/V7期间电流在桥臂内环流，一拍平均即为 t1*Iu
                // （等价于总功率/Ubus = 1.5*Ud*Iu/Ubus）。
                // 注意：θ=0 时 d轴电流 Id=Iu（等幅值Clarke），此判据并非相电流，
                // 实际相电流 Iu 会大于判据目标值；Rs=ΔU/ΔI 用原始采样值，不受此影响
                float current = (MC.Sample.IuReal * MC.Foc.Ud * 1.5f) / MC.Sample.BusReal;
                if (current >= 0.6f * MC.Identify.CurMax)   // 达到目标电流的60%
                {
                    MC.Identify.Flag = 2;       // 进入等待稳定和记录阶段
                }
                else
                {
                    MC.Foc.Ud += 0.0001f;       // 每次调用增加0.0001V（约0.1mV），逐步升压
                    MC.Identify.VoltageSet[0] = MC.Foc.Ud;   // 记录第一次电压点
                }
            }

            /* ----- 子状态 Flag = 2：等待电流稳定，采样100次求平均（第一组电流）----- */
            if (MC.Identify.Flag == 2)
            {
                MC.Identify.WaitTim++;          // 计数器累加（每次函数调用约50~100us，需根据实际周期调整）
                if (MC.Identify.WaitTim > 4000) // 等待约0.2秒（假设周期50us，4000次=0.2s）
                {
                    MC.Identify.CurSum += MC.Sample.IuReal;   // 累加相电流
                }
                if (MC.Identify.WaitTim >= 4100) // 再采集100次（4100-4000=100次）
                {
                    MC.Identify.CurAverage[0] = MC.Identify.CurSum * 0.01f; // 平均电流 = 累加和 / 100
                    MC.Identify.WaitTim = 0;     // 计时器复位
                    MC.Identify.CurSum = 0;       // 累加和清零
                    MC.Identify.Flag = 3;         // 进入第二组电压/电流测量
                }
            }

            /* ----- 子状态 Flag = 3：继续升压，直到电流达到最大电流 CurMax（第二组）----- */
            if (MC.Identify.Flag == 3)
            {
                // 电流判据同Flag=1：current = Iu * t1（t1为V1(100)占空比），即平均母线电流
                float current = (MC.Sample.IuReal * MC.Foc.Ud * 1.5f) / MC.Sample.BusReal;
                if (current >= MC.Identify.CurMax)   // 达到最大电流
                {
                    MC.Identify.Flag = 4;       // 进入等待稳定和记录阶段
                }
                else
                {
                    MC.Foc.Ud += 0.0001f;       // 继续升压
                    MC.Identify.VoltageSet[1] = MC.Foc.Ud;   // 记录第二次电压点
                }
            }

            /* ----- 子状态 Flag = 4：等待稳定，采样100次求平均（第二组电流）----- */
            if (MC.Identify.Flag == 4)
            {
                MC.Identify.WaitTim++;
                if (MC.Identify.WaitTim > 4000)
                {
                    MC.Identify.CurSum += MC.Sample.IuReal;
                }
                if (MC.Identify.WaitTim >= 4100)
                {
                    MC.Identify.CurAverage[1] = MC.Identify.CurSum * 0.01f; // 平均电流
                    MC.Identify.WaitTim = 0;
                    MC.Identify.CurSum = 0;
                    MC.Identify.Flag = 5;       // 进入计算电阻阶段
                }
            }

            /* ----- 子状态 Flag = 5：计算定子电阻 Rs = ΔU / ΔI ----- */
            if (MC.Identify.Flag == 5)
            {
                // 电阻 = (第二组电压 - 第一组电压) / (第二组电流 - 第一组电流)
                MC.Identify.Rs = (MC.Identify.VoltageSet[1] - MC.Identify.VoltageSet[0])
                                 / (MC.Identify.CurAverage[1] - MC.Identify.CurAverage[0]);
                MC.Foc.Ud = 0;                  // 关断电压
                MC.Identify.Flag = 0;           // 重置标志
                MC.Identify.State = INDUCTANCE_IDENTIFICATION;  // 切换到电感辨识阶段
            }

            /* 电阻辨识过程中，始终保持电角度为0（d轴对齐） */
            MC.Foc.SinVal = 0;                  // sin(0) = 0
            MC.Foc.CosVal = 1;                  // cos(0) = 1
            IPark_Transform(&MC.Foc);           // 反Park变换，将Ud/Uq转换为Ualpha/Ubeta
        }
        break;

        /* ================= 阶段2：定子电感辨识 ================= */
        case INDUCTANCE_IDENTIFICATION:
        {
            /* ----- 子状态 Flag = 0：等待电流归零，确保初始条件 ----- */
            if (MC.Identify.Flag == 0)
            {
                MC.Foc.Uq = 0;
                MC.Foc.Ud = 0;
                // 检查相电流是否接近0（±0.05A以内）
                if (MC.Sample.IuReal >= -0.05f && MC.Sample.IuReal <= 0.05f)
                {
                    MC.Identify.Flag = 1;       // 电流归零，开始测试
                }
            }

            /* ----- 子状态 Flag = 1：施加固定电压 Ud，测量电流上升时间，计算电感 ----- */
            if (MC.Identify.Flag == 1)
            {
                MC.Foc.Ud = MC.Identify.VoltageSet[1];   // 使用电阻辨识时第二次的电压值
                MC.Identify.WaitTim++;                    // 计时器递增（单位：函数调用周期）

                // 当电流上升到平均电流的95%时，记录时间
                if (MC.Sample.IuReal >= MC.Identify.CurAverage[1] * 0.95f)
                {
                    // 电感计算公式：L = R * t / ln(1/(1-0.95)) = R * t / ln(20)
                    // ln(20) ≈ 2.9957，系数 0.334 = 1/2.9957 ≈ 0.334
                    // 0.00005 为函数调用周期（假设周期50us），WaitTim为周期数，故 t = WaitTim * 0.00005
                    MC.Identify.LsSum += MC.Identify.Rs * 0.334f * 0.00005f * MC.Identify.WaitTim;
                    MC.Identify.WaitTim = 0;              // 计时器复位
                    MC.Identify.Count++;                  // 完成一次测量，计数器+1
                    MC.Identify.Flag = 0;                 // 返回等待电流归零状态，进行下一次测量
                    MC.Foc.Ud = 0;                        // 关断电压，让电流下降
                    if (MC.Identify.Count >= 100)         // 重复测量100次，取平均值
                    {
                        MC.Identify.Flag = 2;             // 进入计算最终电感
                    }
                }
            }

            /* ----- 子状态 Flag = 2：计算平均电感，切换到编码器对齐阶段 ----- */
            if (MC.Identify.Flag == 2)
            {
                MC.Identify.Ls = MC.Identify.LsSum * 0.01f;   // 平均值 = 累加和 / 100
                MC.Identify.Ld = MC.Identify.Ls;             // 假设表贴式电机，Ld = Lq = Ls
                MC.Identify.Lq = MC.Identify.Ls;
                MC.Identify.Flag = 0;
                MC.Identify.LsSum = 0;
                MC.Identify.WaitTim = 0;
                MC.Identify.State = ENCODER_ROTOR_ALIGN;    // 进入编码器校准阶段
            }

            /* 电感辨识过程中，同样保持电角度为0 */
            MC.Foc.SinVal = 0;
            MC.Foc.CosVal = 1;
            IPark_Transform(&MC.Foc);
        }
        break;

        /* ================= 阶段3：编码器转子对齐与方向检测 ================= */
        case ENCODER_ROTOR_ALIGN:
        {
            /* ----- 子状态 CalibFlag = 0：第一次定位，将转子拉至90°电角度位置 ----- */
            if (MC.EAngle.CalibFlag == 0)
            {
                MC.Foc.Ud += 0.0001f;               // 逐步增加Ud（d轴电压）
                MC.Foc.Uq = 0;                      // q轴电压为0
                MC.Foc.SinVal = 1;                  // 设定电角度为90°：sin90°=1
                MC.Foc.CosVal = 0;                  // cos90°=0
                // 当Ud达到电阻辨识时的最大电压（即电压点VoltageSet[1]），表示电流已足够大
                if (MC.Foc.Ud >= MC.Identify.VoltageSet[1])
                {
                    MC.Foc.Ud = 0;                  // 关断电压
                    MC.EAngle.CalibFlag = 1;       // 第一次定位完成
                    MC.EAngle.EncoderValChange = MC.EAngle.EncoderVal; // 记录此时的编码器原始值
                }
            }

            /* ----- 子状态 CalibFlag = 1：第二次定位，将转子拉至0°电角度位置 ----- */
            if (MC.EAngle.CalibFlag == 1)
            {
                MC.Foc.Ud += 0.0001f;
                MC.Foc.Uq = 0;
                MC.Foc.SinVal = 0;                  // 电角度0°：sin0°=0
                MC.Foc.CosVal = 1;                  // cos0°=1
                if (MC.Foc.Ud >= MC.Identify.VoltageSet[1])
                {
                    // 计算两次定位的编码器差值，判断电机旋转方向
                    MC.EAngle.EncoderValChange = MC.EAngle.EncoderVal - MC.EAngle.EncoderValChange;
                    // 处理编码器溢出（单圈最大值PUL_MAX，半圈为PUL_MAX_HALF）
                    if (MC.EAngle.EncoderValChange < -PUL_MAX_HALF)
                    {
                        MC.EAngle.EncoderValChange += PUL_MAX;
                    }
                    else if (MC.EAngle.EncoderValChange > PUL_MAX_HALF)
                    {
                        MC.EAngle.EncoderValChange -= PUL_MAX;
                    }
                    // 方向判断
                    if (MC.EAngle.EncoderValChange > 0)
                    {
                        MC.EAngle.Dir = 1;         // 1表示反转
                    }
                    else if (MC.EAngle.EncoderValChange < 0)
                    {
                        MC.EAngle.Dir = 0;         // 0表示正转
                    }
                    else
                    {
                        // 如果差值为0，可能是编码器故障或安装问题，可置错误标志
                        // MC.Motor.RunState = MOTOR_ERROR;
                        // MC.Motor.ErrorCode = ENCODER_ERR;
                    }
                    
                    // 先对当前编码器值应用方向处理，再存为偏移量
                    if (MC.EAngle.Dir == 1)
                    {
                        MC.EAngle.CalibOffset = MC.EAngle.EncoderValMax - MC.EAngle.EncoderVal;
                    }else{
                        MC.EAngle.CalibOffset = MC.EAngle.EncoderVal; // 更新偏移（可作为零点偏移）
                    }

                    MC.EAngle.CalibFlag = 0;       // 校准完成
                    MC.Foc.Ud = 0;                  // 关断电压
                    MC.Identify.EndFlag = 1;        // 标识整个辨识流程结束
                }
            }
            IPark_Transform(&MC.Foc);               // 反Park变换
        }
        break;
    }

    /* 所有阶段共用的SVPWM输出（根据当前Foc结构体中的Ualpha/Ubeta和Ubus生成PWM） */
    MC.Foc.Ubus = MC.Sample.BusReal;                // 更新实时母线电压
    Calculate_SVPWM(&MC.Foc);                       // SVPWM调制输出
}





