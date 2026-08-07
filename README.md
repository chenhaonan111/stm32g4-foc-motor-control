# STM32 FOC Brushless Motor Control / STM32 无刷电机磁场定向控制

A field-oriented control (FOC) firmware for PMSM/BLDC motors on STM32G4, supporting both **sensored (encoder)** and **sensorless** (SMO back-EMF observer + HFI high-frequency injection) operation with current / speed / position cascaded loops.

基于 STM32G4 的 PMSM/BLDC 磁场定向控制(FOC)固件,支持**有感(编码器)**与**无感**(SMO 反电势观测器 + HFI 高频注入)两种方式,包含电流 / 速度 / 位置三环级联控制。

---

## 功能特性 / Features

- **FOC 全链路**:Clark / Park / 反 Park 变换、SVPWM、双轴电流 PID(带 LPF)。
- **有感控制**:基于 TIM3 增量式编码器,实现电流环、速度环、位置环(三环级联)。
- **无感控制**:
  - **SMO 滑模观测器**:基于反电势观测 + SPLL 锁相环提取电角度与转速,适用于中高速。
  - **HFI 高频注入**:利用磁饱和诱导凸极性,低速区通过高频电压注入 + 异差解调 + NSD(南北极检测)消除 2θ 歧义,提取转子角度。
  - **强拖启动**:开环/闭环强拖将电机拉至观测器可工作转速后切入闭环。
- **HFI + SMO 混合无感**:低速走 HFI、高速走 SMO,按转速区间自动平滑切换。
- **保护机制**:母线过压 / 欠压、相电流过流检测,故障态硬件关断驱动使能。
- **参数在线辨识**:上电自动辨识定子电阻 Rs、电感 Ld/Lq、编码器零点偏移。
- **调试**:USART1 + DMA 经 VOFA+(JUST_FLOAT 协议)实时上传观测波形。

## 芯片与开发环境 / MCU & Toolchain

| 项目 | 说明 |
|------|------|
| MCU | STM32G431RBTx(Cortex-M4,170 MHz) |
| IDE / 编译器 | Keil MDK-ARM,AC5(armcc) |
| 外设配置 | STM32CubeMX 生成,HAL 库 |

## 运行模式 / Run Modes

通过 `MC.Motor.RunMode` 选择运行模式,`MC.Motor.RunState` 选择有感 / 无感调度链。

| 模式 | 代码 | 类型 | 说明 |
|------|------|------|------|
| 电流闭环 Current Close Loop | `0x02` | 有感 | d/q 双轴电流 PID |
| 速度电流闭环 Speed Current Loop | `0x03` | 有感 | 速度外环 → 电流内环 |
| 位置速度电流闭环 Position Loop | `0x04` | 有感 | 位置 → 速度 → 电流 三环级联 |
| 强拖开环 Strong Drag Open | `0x05` | 无感 | 开环电压强拖启动 |
| 强拖闭环 Strong Drag Close | `0x06` | 无感 | 强拖 + 电流闭环 |
| 强拖切 SMO 速度电流 | `0x07` | 无感 | 强拖启动 → SMO 观测 → 速度电流闭环 |
| HFI 电流闭环 HFI Current | `0x08` | 无感 | 高频注入,测试角度收敛 |
| HFI 速度电流 HFI Speed | `0x09` | 无感 | HFI + 速度电流闭环 |
| HFI 位置速度电流 | `0x0A` | 无感 | 预留(reserved,未实现) |
| HFI + SMO 混合 | `0x0B` | 无感 | 低速 HFI / 高速 SMO 自动切换 |

> 速度环量纲说明:内部统一使用电气 rpm(机械 rpm × 极对数);强拖切闭环阈值(4200 / 3000 / 2000)均为电气 rpm。

## 硬件与外设 / Hardware & Peripherals

| 外设 | 用途 |
|------|------|
| TIM1 | 中心对齐 PWM(20 kHz),驱动三相桥,OC + SD 使能 |
| ADC2(注入) | 与 PWM 同步采样两相电流:Iu(PA6)、Iw(PA4) |
| ADC(规则 + DMA) | 采样母线电压 |
| TIM3 | 编码器接口模式(正交解码) |
| TIM2 | 系统节拍(100 µs),调度按键 / 串口任务 |
| USART1 + DMA | VOFA+ JUST_FLOAT 调试波形上传 |

- FOC 控制周期:`TS = 50 µs`(20 kHz)。
- 方向约定:电机逆时针为正;电位器顺时针增大。

## 辨识参数(示例)/ Identified Parameters (example)

实测辨识所得(具体值依电机而定):

| 参数 | 示例值 |
|------|--------|
| 定子电阻 Rs | ≈ 0.196 Ω |
| 电感 Ld/Lq | ≈ 54 µH(表贴式 Ld≈Lq) |
| 极对数 | 7 |
| 编码器线数 | 1024 |
| 编码器零点偏移 CalibOffset | 149 |

## 目录结构 / Directory

```
FOCProject/
├── Core/               # CubeMX 生成:main / 中断 / 外设初始化
├── Drivers/            # STM32 HAL + CMSIS
├── MDK-ARM/            # Keil 工程(FOCProject.uvprojx)
├── User/
│   ├── GlobalControl/  # 全局控制、状态机、中断回调、目标设定
│   ├── MotorControl/   # FOC 核心:观察器/变换/PID/角度/速度/位置/辨识/有感无感控制
│   ├── KeyControl/     # 按键扫描
│   └── VOFAControl/    # VOFA+ 串口调试输出
└── FOCProject.ioc      # STM32CubeMX 配置
```

## 编译与烧录 / Build & Flash

1. 用 Keil MDK 打开 `MDK-ARM/FOCProject.uvprojx`。
2. 选择 Target `FOCProject`,点击 Build / Rebuild。
3. 通过 J-Link / ST-Link 下载生成的 `FOCProject.hex`。

> 上电流程:ADC 零点校准 → 参数辨识(Rs/Ld/编码器零点)→ 自动进入运行模式。

## 调试 / Debug

上位机使用 [VOFA+](https://www.vofa.plus),串口接收 JUST_FLOAT 协议数据,可实时观察 Id / Iq / 转速 / 角度等波形。

## 状态 / Status

代码开发完成,各模式功能均已实现;硬件实测调试进行中。
