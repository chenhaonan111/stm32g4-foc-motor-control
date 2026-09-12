#include "motor_system.h"
#include "motor_publicdata.h"
#include "motor_identify.h"
#include "motor_sensoruse.h"
#include "motor_sensorless.h"
#include "deadtime_comp.h"

void Motor_System_Init(void)
{
    Motor_Struct_Init();                       //结构体参数初始化
    Deadtime_Comp_Init(&MC.Dtc, DT_COMP_TICKS, 0.05f);
}

/*系统运行主函数（状态机调度）*/
void Motor_System_Run(void)
{
    /* ========== 1. ADC 校准完成后进行电压/电流采样与保护判断 ========== */
    if (MC.Sample.CalibEndFlag == 1)                     // ADC 偏移量已校准（通常在上电初始化时完成）
    {
        // 计算实际母线电压（单位：伏特）
        Calculate_Bus_Voltage(&MC.Sample);
        // 计算三相实际电流（单位：安培）
        Calculate_Phase_Current(&MC.Sample);

        // 设置 d、q 轴电流环 PID 输出限幅（防止输出电压超出母线电压能力）
        MC.IdPid.OutMax =  MC.Sample.BusReal * INV_SQRT3;
        MC.IdPid.OutMin = -MC.Sample.BusReal * INV_SQRT3;
        MC.IqPid.OutMax =  MC.Sample.BusReal * INV_SQRT3;
        MC.IqPid.OutMin = -MC.Sample.BusReal * INV_SQRT3;

        // 母线电压异常检测：正常范围 BUS_VOLTAGE_MIN~BUS_VOLTAGE_MAX（根据实际应用设定）
        if (MC.Sample.BusReal <= BUS_VOLTAGE_MIN || MC.Sample.BusReal >= BUS_VOLTAGE_MAX)
        {
            MC.Motor.RunState = MOTOR_ERROR;        // 切换到故障状态
            MC.Motor.ErrorCode = POWER_VOLT_ERR;    // 电源电压错误代码
        }

        // 过流检测：任一相电流绝对值超过 OVER_CURRENT 即触发过流保护
        if (MC.Sample.IuReal > OVER_CURRENT || MC.Sample.IuReal < -OVER_CURRENT ||
            MC.Sample.IvReal > OVER_CURRENT || MC.Sample.IvReal < -OVER_CURRENT ||
            MC.Sample.IwReal > OVER_CURRENT || MC.Sample.IwReal < -OVER_CURRENT)
        {
            MC.Motor.RunState = MOTOR_ERROR;
            MC.Motor.ErrorCode = OVER_CURRENT_ERR;   // 过流错误代码
        }
    }

    /* ========== 2. 根据系统运行状态执行对应控制 ========== */
    switch (MC.Motor.RunState)
    {
        /* ----- 状态1：ADC 偏移量校准（获取零电流/零电压基准） ----- */
        case ADC_CALIB:
        {
            Calculate_Adc_Offset(&MC.Sample);        // 累加 1024 次采样求平均
            if (MC.Sample.CalibEndFlag == 1)         // 校准完成
            {
                MC.Motor.RunState = MOTOR_IDENTIFY;  // 进入电机参数辨识阶段
            }
        }
        break;

        /* ----- 状态2：电机参数辨识（电阻、电感、编码器对齐） ----- */
        case MOTOR_IDENTIFY:
        {
            Motor_Identify();                        // 执行参数辨识状态机
            if (MC.Identify.EndFlag == 1)            // 辨识完成
            {
                if (MC.Motor.RunState != MOTOR_ERROR)
                {
                    if (MC.Motor.RunMode >= STRONG_DRAG_CURRENT_OPEN)
                    {
                        MC.Motor.RunState = MOTOR_SENSORLESS;     // 无感模式
                    }
                    else
                    {
                        MC.Motor.RunState = MOTOR_SENSORUSE;      // 有感模式
                    }
                }
            }
        }
        break;

        /* ----- 状态3：有感控制（基于编码器传感器） ----- */
        case MOTOR_SENSORUSE:
        {
            Calculate_Encoder_Data(&MC.EAngle);    // 读取编码器并计算电角度、速度
            Sensoruse_Control();                    // 执行有感 FOC 控制（电流环/速度环/位置环）
        }
        break;

        /* ----- 状态4：无感控制（基于高频注入或滑模观测器+锁相环） ----- */
        case MOTOR_SENSORLESS:
        {
            Calculate_Encoder_Data(&MC.EAngle);    // 仍然读取编码器（用于对比或调试，实际无感控制可不依赖）
            Sensorless_Control();                   // 执行无感 FOC 控制
        }
        break;

        /* ----- 状态5：故障处理（封锁 PWM 输出，关闭驱动） ----- */
        case MOTOR_ERROR:
        {
            // 将三相占空比清零
            MC.Foc.DutyCycleA = PWM_CYCLE / 2;
            MC.Foc.DutyCycleB = PWM_CYCLE / 2;
            MC.Foc.DutyCycleC = PWM_CYCLE / 2;
            // 根据硬件设计，拉低使能引脚（PB0、PA1、PA2 为驱动使能信号）
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        }
        break;

        /* ----- 状态6：停机状态（正常停止，无故障） ----- */
        case MOTOR_STOP:
        {
            // 输出零占空比，电机停机
            MC.Foc.DutyCycleA = PWM_CYCLE / 2;
            MC.Foc.DutyCycleB = PWM_CYCLE / 2;
            MC.Foc.DutyCycleC = PWM_CYCLE / 2;
        }
        break;

        default:
        break;
    }
}
