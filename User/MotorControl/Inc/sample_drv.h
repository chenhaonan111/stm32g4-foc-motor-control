#ifndef __SAMPLE_DRV_H__
#define __SAMPLE_DRV_H__

#include "main.h"


typedef struct
{
    int8_t              CurrentDir;     // 电流采样方向
    uint8_t             CalibEndFlag;   // 校准完成标志    
    uint16_t            OffsetCnt;      // 基准值计算次数
    signed long long    BusOffset;      // 母线电压基准值    
    signed long long    IuOffset;       // U相电流偏置值
    signed long long    IvOffset;       // V相电流偏置值        
    signed long long    IwOffset;       // W相电流偏置值
    signed long long    BusRaw;         // 母线电压原始值
    signed long long    IuRaw;          // U相电流原始值
    signed long long    IvRaw;          // V相电流原始值
    signed long long    IwRaw;          // W相电流原始值    
    float               IuReal;         // U相电流真实值           
    float               IvReal;         // V相电流真实值  
    float               IwReal;         // W相电流真实值 
    float               BusReal;        // 母线电压真实值(动态)
    float               BusCalibReal;   // 母线电压真实值(校准值，静态)
    float               BusChange;      // 母线电压变化值
    float               BusFactor;      // 母线电压计算系数
    float               CurrentFactor;  // 相电流计算系数
    uint16_t            AdcBuff[3];     // 用于ADC规则通道接收数据    
} SAMPLE_STRUCT;

/* 采样驱动函数声明 */
void Calculate_Adc_Offset(SAMPLE_STRUCT *p);      //电流/电压零点校准(采1024次取平均)
void Calculate_Phase_Current(SAMPLE_STRUCT *p);   //三相电流实际值计算(Iu/Iw实测,Iv=-Iu-Iw)
void Calculate_Bus_Voltage(SAMPLE_STRUCT *p);     //母线电压实际值及变化量计算

#endif

