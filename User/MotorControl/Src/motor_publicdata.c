#include "motor_publicdata.h"

MOTORCONTROL_STRUCT MC;

//电位器给定速度转向
int8_t Speed_Set_Dir = 1;

void Motor_Struct_Init(void)
{
    // ============================================================================
    // 1. 电机基本运行状态与模式
    // ============================================================================
    // RunState: 电机运行状态机状态，ADC_CALIB表示初始状态为ADC校准（采样偏移校准）
    MC.Motor.RunState = ADC_CALIB;
    // ErrorCode: 故障代码，初始无错误
    MC.Motor.ErrorCode = NONE_ERR;
    // RunMode: 电机运行时的控制模式，SINEHFI_SPEED_CURRENT_CLOSE 表示正弦HFI速度电流闭环
    //          （上电先自动执行一次NSD极性辨识，完成后速度环接管，电位器给速度）
    MC.Motor.RunMode = SINEHFI_SPEED_CURRENT_CLOSE;

    // DutyCycle: 三相占空比初值 = ARR(0%占空比,000零矢量,下管全开安全态)
    MC.Foc.DutyCycleA = PWM_CYCLE / 2;
    MC.Foc.DutyCycleB = PWM_CYCLE / 2;
    MC.Foc.DutyCycleC = PWM_CYCLE / 2;
    
    // ============================================================================
    // 2. 采样参数（电流、电压）
    // ============================================================================
    // CurrentDir: 电流采样方向修正（1表示硬件采样方向与定义相反时取反）
    MC.Sample.CurrentDir = 1;
    // CurrentFactor: 相电流计算系数，将ADC原始值转换为实际安培数
    //                由采样电阻、运放放大倍数、ADC参考电压及分辨率决定
    MC.Sample.CurrentFactor = PHASE_CURRENT_FACTOR;
    // BusFactor: 母线电压计算系数，将ADC原始值转换为实际伏特数
    //            由分压电阻分压比和ADC分辨率决定
    MC.Sample.BusFactor = VBUS_FACTOR;

    // ============================================================================
    // 3. 编码器与电角度参数
    // ============================================================================
    // Dir: 编码器旋转方向，CCW（逆时针）表示角度递增方向为正
    MC.EAngle.Dir = CCW;
    // PolePairs: 电机极对数（永磁体磁极对数 = 磁钢数量/2）
    MC.EAngle.PolePairs = POLEPAIRS;
    // EncoderValMax: 编码器单圈脉冲数（增量式编码器线数 × 4倍频后的值，或绝对式编码器最大码值）
    MC.EAngle.EncoderValMax = PUL_MAX;
    // Ts: 控制周期（秒），用于角度积分和观测器时间基准
    MC.EAngle.Ts = TS;
    
    // ============================================================================
    // 4. FOC（磁场定向控制）参数
    // ============================================================================
    // IdLPFFactor / IqLPFFactor: d/q轴电流低通滤波系数（一阶滤波α值，0~1）
    //                             0.1表示滤波效果较强，响应稍慢
    //                             （曾试0.79（等效截止≈5kHz）提高反馈带宽，但在SineHfi
    //                               陷波反馈路径下电流环穿越频率落入Wo陷波相位谷导致
    //                               裕度崩溃，已回退。调整须与电流环增益联动仿真复核）
    MC.Foc.IdLPFFactor = 0.1f;
    MC.Foc.IqLPFFactor = 0.1f;
    // PwmCycle: PWM周期（通常为定时器的自动重装载值），用于计算占空比
    MC.Foc.PwmCycle = PWM_CYCLE;
    // PwmLimit: PWM输出限幅值，防止占空比过大导致上下桥臂直通或过调制
    MC.Foc.PwmLimit = PWM_LIMLT;
    
    // ============================================================================
    // 5. 位置环参数
    // ============================================================================
    // ElectricalValMax: 电角度单圈脉冲最大值（与编码器单圈最大值相同）
    MC.Position.ElectricalValMax = PUL_MAX;

    // ============================================================================
    // 6. T型加减速参数（速度斜坡）
    // ============================================================================
    MC.TAccDec.PolePairs = POLEPAIRS;   // 极对数，用于机械速度与电速度转换
    MC.TAccDec.Ts = TS;                 // 控制周期
    MC.TAccDec.AccSpeed = ACCELERATION; // 加速度设定值（单位：rpm/s 或 转/秒2）

    // ============================================================================
    // 7. 速度环参数
    // ============================================================================
    MC.Speed.PolePairs = POLEPAIRS;
    // SpeedDivsionFactor: 速度环分频因子（速度环执行周期 = 电流环周期 × 该因子）
    MC.Speed.SpeedDivsionFactor = SPEED_DIVISION_FACTOR;
    // ElectricalValMax: 电角度单圈最大脉冲数
    MC.Speed.ElectricalValMax = PUL_MAX;
    // ElectricalSpeedLPFFactor: 速度反馈的低通滤波系数（一阶滤波）
    MC.Speed.ElectricalSpeedLPFFactor = 0.02f;
    // ElectricalSpeedFactor: 速度计算系数，将电角度差转换为实际转速（rpm）
    //                        公式： (1 / (Ts * SpeedDivsionFactor)) * 60
    MC.Speed.ElectricalSpeedFactor = (1.0f / (TS * SPEED_DIVISION_FACTOR)) * 60.0f;

    // 二阶巴特沃斯速度滤波参数：Calculate_Speed 按速度环分频周期执行
    MC.Speed.ButterLPF.Wc = 628.0f;        // 100Hz，速度环带宽(典型5~50Hz)的5~10倍
    MC.Speed.ButterLPF.Ts = TS * SPEED_DIVISION_FACTOR;   // 速度环周期 = 电流环周期 × 分频系数
    Butter_LPF_Init(&MC.Speed.ButterLPF);

    // ============================================================================
    // 8. 电机参数识别相关（电阻电感识别）
    // ============================================================================
    // CurMax: 识别过程中允许的最大电流（防止过流损坏电机）
    MC.Identify.CurMax = 0.6f;
    
    // ============================================================================
    // 9. 滑模观测器（SMO）参数（用于无传感器中高速段）
    // ============================================================================
    // Gain: 滑模增益，越大观测收敛越快但可能引起抖振
    MC.SMO.Gain = 14.0f;
    MC.SMO.Ts = TS;                         // 观测器运行周期
    // EabForeLPFFactor: 反电动势（Ealpha, Ebeta）低通滤波系数
    MC.SMO.EabForeLPFFactor = 0.1f;
    
    // ============================================================================
    // 10. 锁相环（SPLL）参数（用于从反电动势提取转速和角度）
    // ============================================================================
    MC.SPLL.Ts = TS;
    MC.SPLL.Kp = 1200.0f;                  // 比例系数
    MC.SPLL.Ki = 100.0f;                   // 积分系数
    MC.SPLL.WeForeLPFFactor = 0.01f;       // 观测电角速度低通滤波系数

    // ============================================================================
    // 11. 高频注入（SQHFI）及锁相环（HPLL）参数（用于低速无传感器）
    // ============================================================================
    MC.SqHfi.Enable = 1;                     // 使能高频注入
    MC.SqHfi.Uin = 1.4f;                     // 高频注入电压幅值（伏特）
    MC.HPLL.Dir = 1;                       // 锁相环输入方向（1为正方向）
    MC.HPLL.Kp = 900.0f;                   // 比例系数
    MC.HPLL.Ki = 20.0f;                    // 积分系数
    MC.HPLL.Ts = TS;                       // 运行周期
    MC.HPLL.WeForeLPFFactor = 0.01f;       // 观测电角速度低通滤波系数

    // ============================================================================
    // 12. 正弦高频注入（SineHfi）及标量误差锁相环（HFI_PLL）参数
    //     （脉振正弦注入，低速无感；适配本板硬件：G4+双电阻采样+EG2104栅驱，
    //       7对极电机，相电阻约0.19Ω）
    // ============================================================================
    MC.SineHfi.T = TS;
    MC.SineHfi.Wo = 6855.8f;              // 注入角频率（rad/s，≈1091Hz）
    MC.SineHfi.h = 1.0f;                  // 解调增益
    MC.SineHfi.Uh = 1.4f;                 // 注入电压幅值(V) 
    MC.SineHfi.K1 = 0.3f;                 // 陷波器分母阻尼比
    MC.SineHfi.K2 = 0.0f;                 // 陷波器分子阻尼比(0=理想深陷波)
    MC.SineHfi.Zeta = 0.2f;               // 带通滤波器阻尼比
    HFI_Init(&MC.SineHfi);                // 滤波器系数计算与状态清零（须在以上参数赋值后调用）

    // SineHfi专用标量误差锁相环（跟踪机械角度/机械角速度；与上方HPLL正交锁相环是不同结构）
    MC.SineHfi.Pll.T = TS;
    MC.SineHfi.Pll.Kp = 650.0f;           // PLL比例增益（跟踪机械角，电角度=OutRe×极对数）
    MC.SineHfi.Pll.Ki = 210000.0f;
    HFI_PLL_Init(&MC.SineHfi.Pll);

    MC.SineHfi.Re = 0.0f;                 // 估计电角度初值
    // 观测速度二阶巴特沃斯低通（Wc=100rad/s≈15.9Hz，ζ=1/√2）
    // 本滤波在20kHz电流环中断内每拍执行一次，故 Ts = TS
    MC.SineHfi.SpeedLpf.Wc = 100.0f;      // 截止角频率(rad/s)
    MC.SineHfi.SpeedLpf.Ts = TS;          // 执行周期 = 电流环周期
    Butter_LPF_Init(&MC.SineHfi.SpeedLpf);
    MC.SineHfi.SpeedLPF = 0.0f;
    MC.SineHfi.SpeedMax = 3500.0f;         // HFI速度给定限幅(电rpm，环路内按极对数折算到机械rpm后限幅)
    MC.SineHfi.DebugIdBias = 0.0f;        // 自检模式Id偏置电流(A)：0=仅注入；0.2~0.5=加偏置检测饱和凸极性
    MC.SineHfi.NsdCurrent = 5.0f;         // NSD极性辨识脉冲电流幅值(A)：
                                          //   台架可用自检模式(DebugIdBias)实测饱和所需电流后调整

    // ============================================================================
    // 13. 强拖（强制拖动）到观测器切换参数
    //     用于启动时从开环强拖平滑切换至闭环观测器
    // ============================================================================
    MC.StrongDragToObs.GeneralMode = OPEN_LOOP;              // 当前模式：开环强拖
    MC.StrongDragToObs.LastGeneralMode = OPEN_LOOP;          // 上一次模式
    MC.StrongDragToObs.CloseRunTime = 0;                     // 闭环运行计时
    MC.StrongDragToObs.OpenCurr = 0.05f;                     // 开环强拖电流（安培）
    MC.StrongDragToObs.OpenCurrLast = 0;                     // 上次开环电流
    MC.StrongDragToObs.OpenCurrMax = 3.0f;                   // 开环最大电流限制
    MC.StrongDragToObs.CheckCnt = 0;                         // 切换检测计数器
    MC.StrongDragToObs.ThetaRef = 0;                         // 参考角度（开环给定）
    MC.StrongDragToObs.ThetaObs = 0;                         // 观测器角度
    MC.StrongDragToObs.ThetaErr = 0;                         // 角度误差
    MC.StrongDragToObs.OpenSpeed = 0;                        // 开环目标速度
    MC.StrongDragToObs.CloseSpeed = 0;                       // 闭环反馈速度
    MC.StrongDragToObs.SpeedErr = 0;                         // 速度误差
    MC.StrongDragToObs.SpeedErrFlt = 0;                      // 滤波后速度误差
    MC.StrongDragToObs.ErrFlag = 0;                          // 错误标志
    MC.StrongDragToObs.ErrCnt = 0;                           // 错误计数
    MC.StrongDragToObs.ErrTimes = 0;                         // 错误次数
    MC.StrongDragToObs.OpenToCloseSwitchSpeed = 4200.0f;     // 开环->闭环切换速度阈值（rpm）
    MC.StrongDragToObs.CloseToOpenSwitchSpeed = 3000.0f;     // 闭环->开环切换速度阈值（rpm）
    MC.StrongDragToObs.CloseMinSpeed = 2000.0f;              // 允许闭环的最低速度
    MC.StrongDragToObs.ObsMag = 0;                           // 观测器幅值
    
    // ============================================================================
    // 14. 高频注入（SQHFI）到滑模观测器（SMO）切换参数
    //     实现低速SQHFI到中高速SMO的无缝过渡
    // ============================================================================
    MC.SqHfiToObs.SpeedMin = 3000.0f;          // SQHFI工作最低速度（低于此值仅用SQHFI）
    MC.SqHfiToObs.SpeedMid = 2000.0f;          // 中间速度（用于切换缓冲区）
    MC.SqHfiToObs.SpeedMax = 4200.0f;          // SMO完全接管速度（高于此值仅用SMO）
    MC.SqHfiToObs.SqHfiEleSpeed = 0;             // SQHFI观测的电角速度
    MC.SqHfiToObs.SqHfiEleSpeedAbs = 0;          // SQHFI电角速度绝对值
    MC.SqHfiToObs.SqHfiTheta = 0;                // SQHFI观测的角度
    MC.SqHfiToObs.ObsEleSpeed = 0;             // SMO观测的电角速度
    MC.SqHfiToObs.ObsEleSpeedAbs = 0;          // SMO电角速度绝对值
    MC.SqHfiToObs.ObsTheta = 0;                // SMO观测的角度
    MC.SqHfiToObs.ThetaErr = 0;                // 两种观测器的角度误差
    MC.SqHfiToObs.CheckCnt = 0;                // 切换检测计数器
    MC.SqHfiToObs.EleSpeedOut = 0;             // 最终输出的电角速度
    MC.SqHfiToObs.ThetaOut = 0;                // 最终输出的电角度
    MC.SqHfiToObs.ThetaOffset = 0;             // 角度偏移补偿
    MC.SqHfiToObs.ObsMode = SQHFI;               // 当前观测器模式（初始为SQHFI）
    MC.SqHfiToObs.SqHfiInjectMagMax = 5.0f;      // 高频注入最大电压幅值
    MC.SqHfiToObs.SqHfiInjectMagMin = 1.4f;      // 高频注入最小电压幅值
    
    // ============================================================================
    // 15. 电流环PID参数（Iq、Id环）
    //     【整定记录】曾试Kp=0.3082/Ki=0.0566+α=0.79反馈滤波，实机反而失稳：
    //     SineHfi模式电流反馈路径含Wo=1091Hz陷波器，在700~1500Hz形成相位谷，
    //     该组增益使电流环穿越频率(~763Hz)落入相位谷，裕度不足(仿真PM仅28°)，
    //     叠加实际延迟后失稳，已回退。有感/强拖路径(反馈无陷波器)可用的增益
    //     不可直接用于SineHfi路径。
    //     后续整定应按辨识实测Ls/Rs做极点对消：Kp=Ls*ωc、Ki(每拍)=Rs*ωc*TS，
    //     ωc取2π×300Hz左右（穿越须远离陷波相位谷），并仿真复核SineHfi路径裕度。
    // ============================================================================
    MC.IqPid.Kp = 0.2f;                     // 比例系数(V/A)
    MC.IqPid.Ki = 0.002f;                   // 积分系数(每拍累加口径)
    MC.IqPid.Kd = 0.0f;                     // 微分系数
    MC.IqPid.OutMax = 10;                   // 输出电压上限（伏特）
    MC.IqPid.OutMin = -10;                  // 输出电压下限（伏特）

    MC.IdPid.Kp = 0.2f;
    MC.IdPid.Ki = 0.002f;
    MC.IdPid.Kd = 0.0f;
    MC.IdPid.OutMax = 10;
    MC.IdPid.OutMin = -10;
    // ============================================================================
    // 16. 速度环PID参数（带分段限制）
    // ============================================================================
    MC.SpdPid.Kp = 0.001f;                  // 默认比例系数
    MC.SpdPid.KpMax = 0.005f;               // 比例系数最大值（用于变速调参）
    MC.SpdPid.KpMin = 0.001f;               // 比例系数最小值
    MC.SpdPid.Ki = 0.000002f;               // 积分系数
    MC.SpdPid.OutMax = 6;                   // 输出上限（对应Iq电流参考值，安培）
    MC.SpdPid.OutMin = -6;                  // 输出下限
    // ============================================================================
    // 17. 位置环PID参数（用于位置伺服控制）
    // ============================================================================
    MC.PosPid.Kp = 0.5f;                    // 比例系数
    MC.PosPid.Ki = 0;                       // 积分系数（纯比例控制）
    MC.PosPid.Kd = 0;                       // 微分系数（未使用）
    MC.PosPid.OutMax = 14000;               // 输出上限（对应速度参考值，rpm）
    MC.PosPid.OutMin = -14000;              // 输出下限
}
