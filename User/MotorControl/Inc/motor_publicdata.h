#ifndef __MOTOR_PUBLICDATA_H__
#define __MOTOR_PUBLICDATA_H__

#include "main.h"
#include "sample_drv.h"
#include "eangle_drv.h"
#include "foc_drv.h"
#include "math_drv.h"
#include "pid_drv.h"
#include "speed_drv.h"
#include "position_drv.h"
#include "observer_drv.h"

#define LOW_RESITOR        4.7f     //母线电压检测下端电阻(KΩ)
#define HIGH_RESITOR       100.0f   //母线电压检测上端电阻(KΩ)
#define SAMPLING_RESITOR   0.005f   //相电流采样电阻(Ω)
#define MAGNIFICATION      10.0f    //采样放大倍数
#define ADC_RESOLUTION     4096.0f  //ADC分辨率
#define ADC_VREF           3.3f     //ADC基准电压（V）
#define PWM_LIMLT          7800     //限制最大占空比
#define PWM_CYCLE          8500     //PWM周期占空比(=2×ARR,SVPWM内部周期参数)
#define TS                 0.00005f //FOC执行间隔（S）,=1/20kHz(注意TIM1重复计数=1,实际控制频率10kHz)

#define VBUS_FACTOR            ((ADC_VREF / ADC_RESOLUTION) / (LOW_RESITOR / (LOW_RESITOR + HIGH_RESITOR))) // 母线电压计算系数
#define PHASE_CURRENT_FACTOR   ((ADC_VREF / ADC_RESOLUTION) / MAGNIFICATION / SAMPLING_RESITOR)             // 相电流计算系数

#define INV_SQRT3          0.57735f //1/√3
#define OVER_CURRENT       12.0f    //最大相电流值(A)  超过这个值报错停机
#define BUS_VOLTAGE_MIN    10.0f    //最小供电电压(V)  小于这个值报错停机
#define BUS_VOLTAGE_MAX    40.0f    //最大供电电压(V)  大于这个值报错停机

#define B_VALUE            3434.0f  //B值 (25℃/50℃)3380K  (25℃/85℃)3434K  (25℃/100℃)3455K
#define TEMP_REF           298.15f  //参考温度值  25℃ + 273.15
#define RESISTOR_REF       10000.0f //参考温度下的阻值
#define RESISTOR_OTHER     10000.0f //分压的阻值

/* 电机与方向参数 */
#define POLEPAIRS          7        //电机极对数(默认7对极,请按你的电机改)
#define CW                 0        //顺时针方向
#define CCW                1        //逆时针方向

#define HALF_PI  1.5707963f
#define ONE_PI   3.1415926f
#define TWO_PI   6.2831853f

#define ACCELERATION 4800                   //默认加速度(rpm/s)

#define ENCODER_LINE 1024                   //编码器线数
#define PUL_MAX (4*ENCODER_LINE -1)         //单圈脉冲最大值
#define PUL_MAX_HALF (PUL_MAX / 2)
#define PUL_ANGLE_FACTOR (4095.0f/PUL_MAX)  //角度系数

/****************有感运行模式*******************/
#define ENCODER_CALIB                       0X00  // 编码器校准
#define CURRENT_OPEN_LOOP                    0X01  // 电流开环
#define CURRENT_CLOSE_LOOP                    0X02  // 电流闭环
#define SPEED_CURRENT_LOOP                  0X03  // 速度闭环
#define POS_SPEED_CURRENT_LOOP                0X04  // 位置闭环

/****************无感运行模式*******************/
#define STRONG_DRAG_CURRENT_OPEN            0X05  // 电流开环强拖
#define STRONG_DRAG_CURRENT_CLOSE            0X06  // 电流闭环强拖
#define STRONG_DRAG_SMO_SPEED_CURRENT_LOOP  0X07  // 强拖切滑膜速度电流闭环
#define HFI_CURRENT_CLOSE                   0X08  // 电流闭环高频注入（测试HFI角度收敛效果）
#define HFI_SPEED_CURRENT_CLOSE             0X09  // 高频注入速度电流闭环
#define HFI_POS_SPEED_CURRENT_CLOSE         0X0A  // 高频注入位置速度电流闭环
#define HFI_SMO_SPEED_CURRENT_CLOSE         0X0B


#define SPEED_DIVISION_FACTOR  2     //速度环分频系数
#define POS_DIVISION_FACTOR    4     //位置环分频系数

/******************电机参数识别*********************/
typedef enum{
    RESISTANCE_IDENTIFICATION = 0,        // 相电阻识别
    INDUCTANCE_IDENTIFICATION,            // 相电感识别
    ENCODER_ROTOR_ALIGN                   // 编码器对齐
}IDENTIFY_STATE;

/******************运行状态*********************/
typedef enum{
    ADC_CALIB = 0,                        // ADC校准
    MOTOR_STOP,                           // 停机
    MOTOR_ERROR,                          // 故障报错
    MOTOR_IDENTIFY,                       // 参数辨识
    MOTOR_SENSORUSE,                      // 有感控制
    MOTOR_SENSORLESS                      // 无感控制
}MOTOR_RUN_STATE;

/******************错误码*********************/
typedef enum{
    NONE_ERR = 0,                         //无错误
    ADC_CALIB_ERR,                        //ADC校准错误
    ENCODER_ERR,                          //编码器错误
    POWER_VOLT_ERR,                       //欠压或过压
    OVER_CURRENT_ERR,                     //过流
    TEMPERATURE_ERR                       //温度过高
}MOTOR_ERROR_CODE;

/******************无感通用运行模式*********************/
typedef enum {
    OPEN_LOOP = 0,                                  //开环
    CLOSE_LOOP                                      //闭环
}GENERAL_MODE;

/******************无感观测器类型*********************/
typedef enum{
    HFI = 0,                                        //高频注入
    FLUX,                                           //磁链
    SMO                                             //滑模
}OBSERVER_MODE;

typedef struct
{    
    MOTOR_RUN_STATE       RunState;        // 运行状态
    MOTOR_ERROR_CODE      ErrorCode;
    uint8_t               RunMode;         // 运行模式
}MOTOR_STRUCT;

typedef struct
{    
    IDENTIFY_STATE        State;
    uint8_t               Flag;
    uint8_t               EndFlag;           // 辨识完成标志    
    uint16_t              Count;             // 计数
    uint16_t              WaitTim;           // 等待时间
    float                 Rs;                // 相电阻
    float                 Ls;                // 相电感
    float                 Lq;                // q轴电感
    float                 Ld;                // d轴电感    
    float                 Flux;              // 磁链
    float                 LsSum;             // 电感累计值
    float                 CurMax;            // 最大识别电流
    float                 CurSum;            // 电流累计值
    float                 CurAverage[2];     // 电流平均值
    float                 VoltageSet[2];     // 电压给定值
}IDENTIFY_STRUCT;

typedef struct{
    GENERAL_MODE          GeneralMode;
    GENERAL_MODE          LastGeneralMode;
    MOTION_STATE          MotionState;
    uint16_t              CloseRunTime;
    float                 OpenCurr;
    float                 OpenCurrLast;
    float                 OpenCurrMax;
    int16_t               CheckCnt;
    float                 ThetaRef;
    float                 ThetaObs;
    float                 ThetaErr;
    float                 OpenSpeed;
    float                 CloseSpeed;
    float                 EleOpenSpeedAbs;
    float                 EleCloseSpeedAbs;
    float                 SpeedErr;
    float                 SpeedErrFlt;
    uint8_t               ErrFlag;
    uint16_t              ErrCnt;
    uint16_t              ErrTimes;
    float                 LastTargetSpeed;
    float                 OpenToCloseSwitchSpeed;
    float                 CloseToOpenSwitchSpeed;
    float                 CloseMinSpeed;
    float                 CurrChangeRate;
    float                 ObsMag;
}STRONG_DRAG_TO_OBSERVER;

typedef struct{
    float           SpeedMin;
    float           SpeedMid;
    float           SpeedMax;
    float           SpeedRef;
    float           SpeedRefAbs;
    float           HfiEleSpeed;
    float           HfiEleSpeedAbs;
    float           HfiTheta;
    float           ObsEleSpeed;
    float           ObsEleSpeedAbs;
    float           ObsTheta;
    float           ThetaErr;
    int16_t         CheckCnt;
    float           EleSpeedOut;
    float           ThetaOut;
    float           ThetaOffset;
    OBSERVER_MODE   ObsMode;
    OBSERVER_MODE   LastObsMode;
    float           SpeedChangeRate;
    float           SpeedLast;
    float           HfiInjectMagMax;
    float           HfiInjectMagMin;
}HFI_TO_OBSERVER;

typedef struct
{
    MOTOR_STRUCT            Motor;
    SAMPLE_STRUCT           Sample;
    E_ANGLE_STRUCT          EAngle;
    FOC_STRUCT              Foc;
    IDENTIFY_STRUCT         Identify;
    PID_STRUCT              IqPid;                                    
    PID_STRUCT              IdPid;
    PID_STRUCT              SpdPid;
    PID_STRUCT              PosPid;
    SPEED_STRUCT            Speed;
    TSHAPEDACCDEC_STRUCT    TAccDec;
    POSITION_STRUCT         Position;
    SMO_STRUCT              SMO;
    PLL_STRUCT              SPLL;
    PLL_STRUCT              HPLL;
    STRONG_DRAG_TO_OBSERVER StrongDragToObs;
    HFI_STRUCT              HFI;
    HFI_TO_OBSERVER         HfiToObs;
} MOTORCONTROL_STRUCT;

extern MOTORCONTROL_STRUCT MC;
extern int8_t Speed_Set_Dir;
/* 全局结构初始化 */
void Motor_Struct_Init(void);

#endif
