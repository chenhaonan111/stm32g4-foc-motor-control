#ifndef __SQHFI_DRV__
#define __SQHFI_DRV__

#include "main.h"

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
}SQHFI_STRUCT;

void SQHFI_Calculate(SQHFI_STRUCT *p);

#endif

