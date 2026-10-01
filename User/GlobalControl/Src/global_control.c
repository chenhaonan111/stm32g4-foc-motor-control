#include "global_control.h"
#include "tim.h"
#include "usart_task.h"
#include "key_task.h"
#include "adc.h"
#include "motor_publicdata.h"
#include "motor_system.h"
#include "deadtime_comp.h"

extern volatile uint16_t UsartTaskTim;
extern volatile uint16_t KeyTaskTim;

void Global_Init(void)
{
    HAL_Delay(100);                                         //延时等待电源稳定

    Motor_System_Init();                                    //全局结构初始化(电流/角度/SVPWM参数)

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);  //ADC校准
    HAL_Delay(10);                                          //等待ADC校准完成
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)MC.Sample.AdcBuff, 3);   //启动ADC规则组DMA搬运(母线电压等)

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);         //启动编码器接口

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_CYCLE / 2);// 4250=ARR，0%占空比→下管全开(000)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_CYCLE / 2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_CYCLE / 2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);               //开启三相PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim1);                          //开启TIM1更新中断(触发ADC注入 + 跑FOC)
    HAL_TIM_Base_Start_IT(&htim2);                          //开启TIM2节拍中断(VOFA + 按键)

    /* 阶段3 安全措施:驱动IC使能先注释掉。
       第一步上电:保持注释,只验证开环算法(VOFA看Uq斜坡/三相占空比正弦/角度锯齿);
       验证OK后,取消下面三行注释,重新烧录,电机才会转。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //使能SD1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  //使能SD2
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  //使能SD3
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim1.Instance)            //TIM1更新中断
    {
        HAL_ADCEx_InjectedStart_IT(&hadc2);         //软触发注入组转换(采两相电流)
    }
    if (htim->Instance == htim2.Instance)            //TIM2节拍(10kHz)
    {
        UsartTaskTim++;
        KeyTaskTim++;
        Usart_Task();
        Key_Task();
    }
}

uint16_t Encoder_Data_Get(void)
{
    return TIM3->CNT;
}

void Target_Set(void)
{
    switch(MC.Motor.RunMode)
    {
        case CURRENT_CLOSE_LOOP:                                           //电流闭环
        {                        
            MC.IqPid.Ref = MC.Sample.AdcBuff[1] * 0.002f;                    //使用波轮电位器给电机目标电流（电流闭环模式下）
        }break;    
        
        case SPEED_CURRENT_LOOP:                                           //速度闭环
        {        
            MC.Speed.MechanicalSpeedSet  =  Speed_Set_Dir * MC.Sample.AdcBuff[1] * 0.5f;            //使用波轮电位器给电机目标转速（速度闭环模式下）
            if(MC.Speed.MechanicalSpeedSet <= 5 && MC.Speed.MechanicalSpeedSet >= -5)
            {
                MC.Speed.MechanicalSpeedSet = 0;                               //消除电位器在0位附近采样值抖动引起电机蠕动
            }                
        }break;
        
        case POS_SPEED_CURRENT_LOOP:
        {
            MC.Position.MechanicalPosSet = -MC.Sample.AdcBuff[1];
        }break;
        
        case STRONG_DRAG_CURRENT_OPEN:
        {        
            MC.Speed.MechanicalSpeedSet  =  Speed_Set_Dir * MC.Sample.AdcBuff[1] * 0.5f;
            if(MC.Speed.MechanicalSpeedSet <= 5 && MC.Speed.MechanicalSpeedSet >= -5)
            {
                MC.Speed.MechanicalSpeedSet = 0;
            }                
        }break;
        
        case STRONG_DRAG_CURRENT_CLOSE:
        {        
            MC.Speed.MechanicalSpeedSet  =  Speed_Set_Dir * MC.Sample.AdcBuff[1] * 0.5f;
            if(MC.Speed.MechanicalSpeedSet <= 5 && MC.Speed.MechanicalSpeedSet >= -5)
            {
                MC.Speed.MechanicalSpeedSet = 0;
            }                
        }break;
        
        case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:
        {    
            MC.Speed.MechanicalSpeedSet  =  Speed_Set_Dir * MC.Sample.AdcBuff[1] * 0.5f;             //使用波轮电位器给电机目标转速（速度闭环模式下）
        }break;

        case SQHFI_SPEED_CURRENT_CLOSE:
        {
            MC.Speed.MechanicalSpeedSet = Speed_Set_Dir * MC.Sample.AdcBuff[1] * 0.5f;
            if(MC.Speed.MechanicalSpeedSet <= 5 && MC.Speed.MechanicalSpeedSet >= -5)
            {
                MC.Speed.MechanicalSpeedSet = 0;
            }
        }break;
        
        case SQHFI_SMO_SPEED_CURRENT_CLOSE:
        {
            MC.Speed.MechanicalSpeedSet  =  Speed_Set_Dir * MC.Sample.AdcBuff[1] * 0.5f;            //使用波轮电位器给电机目标转速（速度闭环模式下）
            if(MC.Speed.MechanicalSpeedSet <= 5 && MC.Speed.MechanicalSpeedSet >= -5)
            {
                MC.Speed.MechanicalSpeedSet = 0;                               //消除电位器在0位附近采样值抖动引起电机蠕动
            }
        }break;

        case SINEHFI_SPEED_CURRENT_CLOSE:
        {
            // 正弦高频注入速度电流闭环：电位器给速度给定（进入环路后还会被SpeedMax限幅到HFI低速域）
            MC.Speed.MechanicalSpeedSet = Speed_Set_Dir * MC.Sample.AdcBuff[1] * 0.5f;
            if(MC.Speed.MechanicalSpeedSet <= 5 && MC.Speed.MechanicalSpeedSet >= -5)
            {
                MC.Speed.MechanicalSpeedSet = 0;                               //消除电位器在0位附近采样值抖动引起电机蠕动
            }
        }break;
    }
}

/**
 * ADC注入转换完成中断回调:FOC 控制核心入口(每个控制周期执行一次)。
 * 阶段3 流程:先做电流零点校准 → 校准完成后跑开环强拖。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 1. 读ADC原始值 */
    MC.Sample.IuRaw  = ADC2->JDR1;                   //U相电流原始值
    MC.Sample.IwRaw  = ADC2->JDR2;                   //W相电流原始值
    MC.Sample.BusRaw = MC.Sample.AdcBuff[0];         //母线电压(规则组DMA缓冲第0个)
    MC.EAngle.EncoderVal = Encoder_Data_Get();       //获取编码器值
    
    Target_Set();
    
    Motor_System_Run();

    /* ============ 死区补偿 ============ */
    if(MC.Sample.CalibEndFlag == 1 && 
       MC.Motor.RunState != MOTOR_STOP && 
       MC.Motor.RunState != MOTOR_ERROR &&
       MC.Motor.RunState != MOTOR_IDENTIFY)
    {
        // 计算三相电流极性和补偿量
        Deadtime_Comp_Calculate(&MC.Dtc,
                                MC.Sample.IuReal,
                                MC.Sample.IvReal,
                                MC.Sample.IwReal);

        // 应用补偿到占空比
        Deadtime_Comp_Apply(&MC.Dtc,
                            &MC.Foc.DutyCycleA,
                            &MC.Foc.DutyCycleB,
                            &MC.Foc.DutyCycleC,
                            PWM_CYCLE / 2);
    }
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MC.Foc.DutyCycleA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MC.Foc.DutyCycleB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MC.Foc.DutyCycleC);
}
