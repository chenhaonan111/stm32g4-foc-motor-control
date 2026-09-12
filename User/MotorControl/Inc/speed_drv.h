#ifndef __SPEED_DRV_H__
#define __SPEED_DRV_H__

#include "main.h"
#include "butter_filter.h"

/*运动状态指示*/
typedef enum{
    ACCELERATE = 0,     //加速
    UNIFORM,            //匀速
    DECELERATE          //减速
}MOTION_STATE;

typedef struct
{
    uint8_t           PolePairs;
    int16_t           CurrentSpeedDir;
    uint16_t          SpeedCalculateCnt;        // 速度计算计数    
    uint16_t          SpeedDivsionFactor;       // 速度分频系数
    uint16_t          ElectricalValMax;         // 电角度最大值
    float             ElectricalPosThis;        // 本次电角位置
    float             ElectricalPosLast;        // 上次电角位置
    float             ElectricalPosChange;      // 单位时间位移
    float             ElectricalSpeedFactor;    // 电角速度系数
    float             ElectricalSpeedRaw;       // 原始电角速度    
    float             ElectricalSpeedLPF;       // 原始电角速度滤波值    
    float             ElectricalSpeedLPFFactor; // 原始电角速度滤波系数
    BUTTER_LPF_STRUCT ButterLPF;                // 二阶巴特沃斯速度滤波器(ElectricalSpeedLPF 复用为结果)
    float             MechanicalSpeed;          // 滤波后机械速度
    float             MechanicalSpeedSet;       // 目标机械速度
    float             MechanicalSpeedSetLast;   // 上次目标机械速度
    
}SPEED_STRUCT;  

typedef struct
{  
    int8_t          SpeedOutDir;          //实时速度方向
    uint8_t         PolePairs;            //电机极对数
    float           Ts;
    float           TargetSpeed;          //目标速度
    float           AccSpeed;             //加速度
    
    float           SpeedTargetIncrement; //目标速度增量
    float           SpeedIncrement;       //加速度速度增量
    float           SpeedChangeIncrement; //变速阶段速度增量
    float           SpeedOut;             //实时参考速度
    
    float           SpeedIincrementDelta;
    float           SpeedIncrementLast;
    MOTION_STATE    MotionState;
    uint8_t         FinishFlag;           //加减速完成标志
}TSHAPEDACCDEC_STRUCT;


void Calculate_Speed(SPEED_STRUCT *p);
void T_Shaped_Acc_Dec(TSHAPEDACCDEC_STRUCT *p);

#endif

