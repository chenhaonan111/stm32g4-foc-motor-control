#include "sample_drv.h"

/*获取相电流和母线电压的基准值（偏移量校准）*/
void Calculate_Adc_Offset(SAMPLE_STRUCT *p)
{
    // 若计数器为0，表示开始新的校准过程，初始化所有累加变量和标志
    if (p->OffsetCnt == 0)
    {
        p->CalibEndFlag = 0;          // 清除校准完成标志
        p->IuOffset = 0;         // U相电流偏移累加和清零
        p->IwOffset = 0;         // W相电流偏移累加和清零
        p->BusOffset = 0;        // 母线电压偏移累加和清零
        p->OffsetCnt = 0;        // 采样计数器清零
    }

    // 采样次数未达到1024次时，累加原始ADC值
    if (p->OffsetCnt < 1024)
    {
        p->IuOffset += p->IuRaw;      // 累加U相原始值
        p->IwOffset += p->IwRaw;      // 累加W相原始值
        p->BusOffset += p->BusRaw;    // 累加母线电压原始值
        p->OffsetCnt++;               // 计数器递增
    }
    else
    {
        // 已累加1024次，右移10位（除以1024）求平均值
        p->IuOffset = p->IuOffset >> 10;   // U相电流偏移量（ADC码值）
        p->IwOffset = p->IwOffset >> 10;   // W相电流偏移量（ADC码值）
        p->BusOffset = p->BusOffset >> 10; // 母线电压偏移量（ADC码值）

        // 将母线电压偏移量转换为实际电压值（乘以电压转换系数BusFactor）
        p->BusCalibReal = p->BusOffset * p->BusFactor;

        p->OffsetCnt = 0;          // 计数器归零，便于下次重新校准
        p->CalibEndFlag = 1;            // 设置校准完成标志
    }
}
/*计算三相电流实际值（以流入电机中性点方向为正）*/
void Calculate_Phase_Current(SAMPLE_STRUCT *p)
{
    // 计算U相实际电流： (原始值 - 偏移量) * 方向 * 转换系数
    p->IuReal = p->CurrentDir * (p->IuRaw - p->IuOffset) * p->CurrentFactor;
    // 计算W相实际电流
    p->IwReal = p->CurrentDir * (p->IwRaw - p->IwOffset) * p->CurrentFactor;
    // 计算V相实际电流： Iu + Iv + Iw = 0  => Iv = -Iu - Iw
    p->IvReal = -p->IuReal - p->IwReal;
}

/*计算实时母线电压及电压变化量*/
void Calculate_Bus_Voltage(SAMPLE_STRUCT *p)
{
    // 实时母线电压 = 原始ADC值 * 转换系数
    p->BusReal = p->BusRaw * p->BusFactor;
    // 母线电压变化量 = 实时电压 - 校准基准电压
    p->BusChange = p->BusReal - p->BusCalibReal;
}

