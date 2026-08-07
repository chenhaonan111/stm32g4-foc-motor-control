#ifndef __OBSERVER_DRV_H__
#define __OBSERVER_DRV_H__

#include "main.h"

typedef struct
{      
    float Ts;                   //调用周期    
    float Rs;                   //相电阻
    float Ld;                   //相电感
    float Gain;                 //滑膜观测器增益

    float Ialpha;               //α轴实际电流
    float Ibeta;                //β轴实际电流        

    float IalphaFore;           //α轴预测电流
    float IbetaFore;            //β轴预测电流

    float Ualpha;               //α轴实际电压
    float Ubeta;                //β轴实际电压    

    float EalphaFore;           //α轴预测反势
    float EalphaForeLPF;        //α轴预测反势滤波值    

    float EbetaFore;            //β轴预测反势
    float EbetaForeLPF;         //β轴预测反势滤波值

    float EabForeLPFFactor;     //αβ轴预测反势滤波系数

    float EMag;                 //反电动势幅值
}SMO_STRUCT;

typedef struct
{
    uint8_t    Enable;
    uint8_t    NSDOut;             //辨识结果输出
    uint8_t    NSDFlag;            //极性辨识标志    
    uint16_t   NSDCount;           //计数变量
    float      NSDSum1;
    float      NSDSum2;    
              
    float      IdRef;              //极性辨识时d轴给定
    float      IdHigh;             //极性辨识时只用到d轴高频分量

    uint8_t    Dir;                //高频注入信号极性    
    float      Uin;                //高频注入信号幅值
              
    float      Id;                 //当前d轴电流值
    float      IdLast;             //上次d轴电流值
    float      IdBase;             //d轴基频分量    
              
    float      Iq;                 //当前q轴电流值
    float      IqLast;             //上次q轴电流值
    float      IqBase;             //q轴基频分量    
              
    float      Ialpha;             //α轴当前值
    float      IalphaLast;         //α轴上次值    
    float      IalphaHigh;         //α轴高频分量
    float      IalphaHighLast;     //α轴上次高频分量        
              
    float      Ibeta;              //β轴当前值    
    float      IbetaLast;          //β轴上次值    
    float      IbetaHigh;          //β轴高频分量
    float      IbetaHighLast;      //β轴上次高频分量
              
    float      IalphaOut;          //电流包络
    float      IbetaOut;           //电流包络
}HFI_STRUCT;

typedef struct
{    
    int8_t   Dir;                  //反电动势计算方向 
    float    Ts;                   //调用周期        
             
    float    Ain;                  //输入A
    float    Bin;                  //输入B
             
    float    ThetaErr;             //观测角度误差        
    float    ThetaFore;            //观测角度 单位：弧度
    float    ThetaCompensate;      //补偿后的观测角度 单位：弧度
    float    ETheta;               //补偿后的观测角度 0-3999
    float    EThetaPU;
             
    float    SinVal;               //正弦值
    float    CosVal;               //余弦值
             
    float    Kp;                   //锁相环KP
    float    Ki;                   //锁相环KI
    float    PPart;                //积分项
    float    IPart;                //积分项
             
    float    WeFore;               //观测电角速度（rad/s）    
    float    WeForeLPF;            //观测电角速度滤波值    
    float    WeForeLPFFactor;      //观测电角速度滤波系数    
}PLL_STRUCT;

void SMO_Calculate(SMO_STRUCT *p);
void PLL_Calculate(PLL_STRUCT *p);
void HFI_Calculate(HFI_STRUCT *p);

static inline float Sat(float value, float min, float max){
    if(value >= max){
        return max;
    }
    if(value <= min){
        return min;
    }
    return value;
}

#endif

