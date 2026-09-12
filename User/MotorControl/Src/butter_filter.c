#include "butter_filter.h"
#include "math.h"

/* NaN/Inf 检查：按 IEEE754 位域判定，纯位运算，无库依赖
 * 指数域全1且尾数为0 -> Inf；指数域全1且尾数非0 -> NaN
 * static = 内部链接：仅本文件可见，属于模块私有实现，不导出符号 */
static int Value_isinf(float x)
{
    union { float f; uint32_t i; } u = {x};
    if ((u.i & 0x7F800000u) == 0x7F800000u && (u.i & 0x007FFFFFu) == 0u) {
        return 1;
    }
    return 0;
}

static int Value_isnan(float x)
{
    union { float f; uint32_t i; } u = {x};
    if ((u.i & 0x7F800000u) == 0x7F800000u && (u.i & 0x007FFFFFu) != 0u) {
        return 1;
    }
    return 0;
}

/* 二阶巴特沃斯低通系数计算（预畸变双线性变换）
 * 离散化时以 2/T -> Wc/tan(Wc*Ts/2) 替换，保证 -3dB 点精确落在 Wc
 * 令 m = tan(Wc*Ts/2)，则：
 *   num = [m^2, 2*m^2, m^2]
 *   den = [1+sqrt(2)m+m^2, -2+2*m^2, 1-sqrt(2)m+m^2]
 * 修改 Wc/Ts 后需重新调用本函数 */
void Butter_LPF_Init(BUTTER_LPF_STRUCT *p)
{
    float wt = p->Wc * p->Ts * 0.5f;
    float m;

    /* 保护：wt 逼近 π/2（截止频率逼近奈奎斯特 fs/2）时 tan 发散。
       正常设计 fc 远低于 fs/2 不会触发；1.3rad 约对应 fc = 0.41*fs */
    if (wt > 1.3f) {
        wt = 1.3f;
    }

    m = tanf(wt);
    p->num[0] = m * m;
    p->num[1] = 2.0f * m * m;
    p->num[2] = m * m;
    p->den[0] = 1.0f + 1.41421356f * m + m * m;
    p->den[1] = -2.0f + 2.0f * m * m;
    p->den[2] = 1.0f - 1.41421356f * m + m * m;

    for (int n = 0; n < 3; n++) {
        p->x[n] = 0.0f;
        p->y[n] = 0.0f;
    }
    p->Input = 0.0f;
    p->Output = 0.0f;
}

/* 二阶巴特沃斯低通差分方程，每个滤波周期调用一次
 * 调用前给 Input 赋值，调用后从 Output 取结果
 * 差分方程：den[0]*y[k] = num*x序列 - den[1]*y[k-1] - den[2]*y[k-2]
 * 含 NaN/Inf 保护，防止异常值经反馈进入 PID */
void Butter_LPF_Calc(BUTTER_LPF_STRUCT *p)
{
    float num;
    float den;

    /* 历史移位：x[1]->x[2], x[0]->x[1]，y 同理 */
    p->x[2] = p->x[1];
    p->x[1] = p->x[0];
    p->y[2] = p->y[1];
    p->y[1] = p->y[0];

    p->x[0] = p->Input;
    num = p->num[0] * p->x[0] + p->num[1] * p->x[1] + p->num[2] * p->x[2];
    den = p->den[1] * p->y[1] + p->den[2] * p->y[2];

    if (p->den[0] != 0.0f && !Value_isnan(den) && !Value_isinf(den)) {
        p->y[0] = (num - den) / p->den[0];
    }
    if (Value_isnan(p->y[0]) || Value_isinf(p->y[0])) {
        p->y[0] = 0.0f;
    }
    p->Output = p->y[0];
}

