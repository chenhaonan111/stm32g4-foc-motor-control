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

void SMO_Calculate(SMO_STRUCT *p);

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

