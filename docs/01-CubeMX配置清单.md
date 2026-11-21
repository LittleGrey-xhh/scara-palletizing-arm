# 01 · CubeMX 配置清单

> 目标：生成一个能编译、能烧、外设全配好的空工程 `stm32\Arm\`
> 你只管照着点，配置清单以外的代码我来写
> 更新时间：2026-11-07

---

## 零、先改两个引脚（和交接文档不一样，说明原因）

| 功能 | 原计划 | **改成** | 为什么 |
|---|---|---|---|
| 步进 ENA | PC13 | **PB14** | F103 的 PC13/14/15 挂在 VBAT 电源域，**最大只能灌/拉 3mA**，而驱动器光耦要 8~15mA。PC13 大概率拉不动 ENA，表现为"使能无效、电机永远松着" |
| 其余不变 | | | |

**PA8 那一颗 LED 电阻可以先不拆。** 330Ω 的 LED 只多灌 4.2mA，PA8 能到 20mA，加上光耦 10mA 是 14.2mA，撑得住。只是发脉冲时 LED 会跟着闪，看着像跑马灯，不影响功能。真出问题再拆（拆的是 LED 串联的那颗 330Ω，不是限流给驱动器的）。

---

## 一、新建工程

1. CubeMX → `File` → `New Project`
2. 搜索 `STM32F103C8`，选 **STM32F103C8Tx**，双击
3. 弹窗问要不要默认初始化，选 **No**（不勾 TrustZone / 不要默认外设）

---

## 二、System Core

### 2.1 RCC（时钟源）

`System Core` → `RCC`

| 项目 | 值 |
|---|---|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator** |
| Low Speed Clock (LSE) | Disable |

> LSE 一定关掉。本板 PC15 拿去接别的东西了（OSC32_OUT），开 LSE 会一直卡在起振失败。

### 2.2 SYS（调试口）

`System Core` → `SYS`

| 项目 | 值 |
|---|---|
| Debug | **Serial Wire** |
| Timebase Source | SysTick |

> Debug 一定要选 Serial Wire，不选的话 PA13/PA14 被当成普通 IO，**烧一次就再也连不上 J-Link**，只能接复位线救。

### 2.3 Clock Configuration（时钟树）

切到 `Clock Configuration` 页：

1. `Input frequency` 填 **8**
2. `PLLMul` 选 **X9**
3. `System Clock Mux` 选 **PLLCLK**
4. `AHB Prescaler` = /1 → **72MHz**
5. `APB1 Prescaler` = /2 → **36MHz**
6. `APB2 Prescaler` = /1 → **72MHz**
7. 在 `HCLK` 框里敲 `72` 回车，让 CubeMX 自动解一遍，确认没有红色

**为什么 APB1 是 36 但定时器还是 72：** APB 预分频 ≠ 1 时，挂在该总线上的定时器时钟会自动 ×2。所以 TIM2/TIM3/TIM4 实际还是 72MHz，和 TIM1 一样。这样所有定时器都能统一用 `PSC = 71 → 1MHz`。

---

## 三、定时器

### 3.1 TIM1 —— 步进脉冲（PA8）

`Timers` → `TIM1`

| 页签 | 项目 | 值 |
|---|---|---|
| Mode | Clock Source | **Internal Clock** |
| Mode | Channel1 | **PWM Generation CH1** |
| Parameter | Prescaler | **71** |
| Parameter | Counter Mode | Up |
| Parameter | Counter Period | **499**（= 5mm/s 的初值，运行时改） |
| Parameter | Internal Clock Division | No division |
| Parameter | auto-reload preload | **Enable** |
| Parameter | PWM Mode（CH1） | **PWM mode 1** |
| Parameter | Pulse（CH1） | **250** |
| Parameter | CH Polarity | High |
| NVIC | TIM1 update interrupt | **勾上** |

**算一下：** PSC=71 → 计数频率 1MHz。一个 PWM 周期就是一个脉冲，所以：

```
ARR = 1,000,000 / f(脉冲每秒) - 1
```

| 速度 | 脉冲频率 | ARR | Pulse（ARR 的一半） |
|---|---|---|---|
| 5 mm/s（回零慢速） | 2000 Hz | **499** | 250 |
| 20 mm/s | 8000 Hz | **124** | 62 |
| 50 mm/s（快速定位） | 20000 Hz | **49** | 24 |

脉冲当量 0.0025mm/脉冲（导程 4mm ÷ 1600 脉冲/转）。

**为什么 PSC 用 71 而不是 0（用满 72MHz）：** 16 位 ARR 最大 65535。用 1MHz 计数时最低能到 15Hz（0.038mm/s），回零那种慢速下得去；如果直接拿 72MHz 数，最低只能到 1098Hz（2.7mm/s），回零就嫌快了。

> CubeMX 里填的 ARR 只决定上电初值，运行中我用 `__HAL_TIM_SET_AUTORELOAD()` 动态改。
>
> **Pulse 必须跟着 ARR 一起改。** 占空比固定 50%（Pulse = ARR/2），改 ARR 不改 Pulse 的话，高速时脉宽会变得很窄，光耦来不及响应。

### 3.2 TIM2 —— 大臂 + 小臂舵机（PA0 / PA1）

`Timers` → `TIM2`

| 页签 | 项目 | 值 |
|---|---|---|
| Mode | Clock Source | **Internal Clock** |
| Mode | Channel1 | **PWM Generation CH1** |
| Mode | Channel2 | **PWM Generation CH2** |
| Parameter | Prescaler | **71** |
| Parameter | Counter Period | **19999** |
| Parameter | auto-reload preload | Enable |
| Parameter | PWM Mode 1/2 | PWM mode 1 |
| Parameter | Pulse CH1 | **1500** |
| Parameter | Pulse CH2 | **1500** |

**算一下：** 1MHz / 20000 = 50Hz，周期 20ms。CCR 直接就是脉宽的微秒数，`CCR = 1500` 就是 1.5ms 中位。这个映射最直观，不用换算。

**TIM2 不开中断**，舵机不需要。

### 3.3 TIM4 —— 腕转 + 夹爪（PB8 / PB6）

`Timers` → `TIM4`

| 页签 | 项目 | 值 |
|---|---|---|
| Mode | Clock Source | **Internal Clock** |
| Mode | **Channel1** | **PWM Generation CH1**（→ PB6，夹爪） |
| Mode | **Channel3** | **PWM Generation CH3**（→ PB8，腕转） |
| Parameter | Prescaler | **71** |
| Parameter | Counter Period | **19999** |
| Parameter | Pulse CH1 | **1500** |
| Parameter | Pulse CH3 | **1500** |
| Parameter | auto-reload preload | Enable |

**同一个定时器的多个通道共享周期（ARR），但各自的 CCR 独立。** 所以两路舵机都是 50Hz、脉宽各调各的，互不干扰。

> **为什么腕转是 CH3、夹爪是 CH1：** PB8 这个脚只映射到 TIM4_CH3；PB6 映射到 TIM4_CH1。不是随便挑的，是芯片引脚复用的唯一选择。
>
> **★ PB6/PB7 在中控板上接到了 OLED 的 I2C（SCL/SDA）。** 模块没焊就是干净的空焊盘，可以直接用；**如果焊了排母并插了屏，PB6 会被屏的 I2C 上拉电阻拉住，PWM 出不来**，那就只能放弃编码器，把夹爪改到 PB4（TIM3_CH1）。
>
> **为什么不动 TIM4_CH4（PB9）：** PB9 被步进 DIR 占了。

---

## 四、GPIO

`System Core` → `GPIO`

### 4.1 输出脚

| 引脚 | 模式 | 输出电平 | 上下拉 | 速度 | User Label | 用途 |
|---|---|---|---|---|---|---|
| **PB9** | Output Push Pull | **Low** | No pull | Low | `STEP_DIR` | 步进方向 |
| **PB14** | Output Push Pull | **High** | No pull | Low | `STEP_ENA` | 步进使能（低有效，上电先失能） |

### 4.2 输入脚

| 引脚 | 模式 | 上下拉 | 触发 | User Label | 用途 |
|---|---|---|---|---|---|
| **PB12** | **External Interrupt Mode with Falling edge trigger detection** | **Pull-up** | Falling | `LIMIT_Z` | 限位（NPN 常开，检测到拉低） |

> **一定要填 User Label。** 填了之后 CubeMX 会在 `main.h` 里生成 `STEP_DIR_Pin` / `STEP_ENA_Pin` / `LIMIT_Z_Pin` 这样的宏，代码里就不用写 `GPIO_PIN_9` 这种裸名字，看代码的人一眼知道在干嘛。
>
> 注意：PB12 在中控板底板上挂着一个 SW2 按键。NPN 集电极开路输出和按键并联，两者都是"拉低有效"，正好兼容。如果按键电路带 100nF 滤波电容，边沿会延迟约 1ms —— 回零速度 5mm/s 时对应 0.005mm，可以忽略。

---

## 五、串口 USART1（PA9 / PA10）

`Connectivity` → `USART1`

| 项目 | 值 |
|---|---|
| Mode | **Asynchronous** |
| Baud Rate | **115200** |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| Over Sampling | 16 Samples |

**NVIC 里 USART1 global interrupt 先不勾。** 第一版用轮询收命令（`HAL_UART_Receive` 带超时），简单可靠；等协议定了再改中断 + DMA。

---

## 六、NVIC 优先级

`System Core` → `NVIC`

| 中断 | 使能 | Preemption Priority |
|---|---|---|
| TIM1 update interrupt | ✔ | **1** |
| EXTI line[15:10] interrupts | ✔ | **1** |
| USART1 global | ✘ | — |

> **TIM1 和 EXTI 必须同优先级 1。** 限位中断里要读 `g_pulse_count`，如果 TIM1 中断能抢占它，读到的计数值可能是"差一个脉冲"的旧值。同优先级不能互相抢占，就安全了。
>
> 这里没用 FreeRTOS，不用管 `configMAX_SYSCALL_INTERRUPT_PRIORITY`。

---

## 七、Project Manager

切到 `Project Manager` 页。

### 7.1 Project

| 项目 | 值 |
|---|---|
| Project Name | **Arm** |
| Project Location | `firmware/`（本仓库内；本地开发时是自己的工作目录） |
| Application Structure | Advanced |
| Toolchain / IDE | **MDK-ARM**，Min Version **V5** |

### 7.2 Code Generator

| 项目 | 值 |
|---|---|
| Copy only the necessary library files | ✔ |
| Generate peripheral initialization as a pair of '.c/.h' files per peripheral | **✔ 必勾** |
| Delete previously generated files when not re-generated | ✘ **不要勾** |
| Keep User Code when re-generating | ✔ |

> **"a pair of .c/.h files" 必须勾。** 不勾的话所有外设初始化全塞在 `main.c` 一个文件里，改起来没法看。

### 7.3 生成

点右上角 `GENERATE CODE`。第一次会提示下载固件包，等它下完（F1 的包约 200MB）。

---

## 七之二、★★ 生成后立刻改栈大小（1KB 会 HardFault）

**CubeMX 默认 `Stack_Size = 0x400`（1KB），这个项目不够用。**

现象：程序烧进去跑几秒就 HardFault，`IPSR = 003`，CFSR 报 `IMPRECISERR`（bit 10），SP 已经落进变量区。

**为什么不够：**

| 吃栈的东西 | 大概占多少 |
|---|---|
| `printf` 格式化（哪怕只印几个 int） | 300~600 字节 |
| 软浮点运算（`sqrtf` / `acosf` / 浮点除法） | 100~300 字节 |
| 中断嵌套（TIM1 更新中断） | 几十字节 |
| 逆运动学的函数调用链 | 100~200 字节 |

加起来轻松破 1KB，**栈一溢出就踩变量，表现是莫名其妙的 HardFault 或数据错乱**。

**改法（两个地方都要改）：**

**① `MDK-ARM\startup_stm32f103xb.s` 第 32 行**

```asm
Stack_Size      EQU     0x400      ; 改成 0x1000
```

**② `Arm.ioc` 里同步改（防止下次 Generate 冲掉）**

```
ProjectManager.StackSize=0x400     →  0x1000
```

改完 `ZI-data` 会从 1424 涨到 4496（多 3KB），RAM 总占用 4.5KB / 20KB，还很宽裕。

> **判据：** 编译后看 `Program Size`，`ZI-data` 应该在 4500 左右。如果还是 1400 多，说明栈没改成功。

---

## 八、生成之后（每次 Generate 都要做一遍）

### 8.1 编码转换（★ 否则 Keil 里中文注释全是乱码）

```bash
cd /e/a2_QianRuShi/stm32/Arm
python /e/a2_QianRuShi/stm32/tools/enc.py utf8
```

> CubeMX 是按 GBK 读写文件的，生成出来的注释编码会和你原来的 UTF-8 混在一起。这个脚本统一转 UTF-8 无 BOM。
> **反过来，下次再进 CubeMX 改配置之前先跑 `enc.py gbk`**，不然 CubeMX 保存时会把 UTF-8 中文写成乱码。

### 8.2 绑定 User 分组

```bash
python /e/a2_QianRuShi/stm32/tools/add_user_sources.py MDK-ARM/Arm.uvprojx
```

这条命令把 `User/` 目录下的所有 `.c` 绑进 Keil 工程分组，并把 `User` 的 include path 加进去。新工程只需要跑一次。

### 8.3 第一次编译验证

```bash
/e/a2_QianRuShi/tool/keil/Keil_v5/UV4/UV4.exe -j0 -b MDK-ARM/Arm.uvprojx -o build.log
```

退出码 **0 = 无错无警告**，1 = 有警告，2 = 有错误。

---

## 九、烧录器设置（J-Link）

1. Keil 打开工程 → 魔术棒 `Options for Target` → `Debug` 页
2. 右侧下拉选 **J-LINK / J-TRACE Cortex**
3. 点 `Settings`：
   - `Port` = **SW**
   - `Max Clock` 先设 **1MHz**，连上再降到 **400kHz**（这板子 400kHz 最稳）
4. `Flash Download` 页 → 勾 `Reset and Run`

> 中控板 J-Link 用 400kHz，终端板要 100kHz。这块是新板，先按 400kHz 试，连不上降到 100kHz。

---

## 十、配置完必须自检的 4 条

万一手抖点错了，这 4 条能兜住：

1. **`Core/Inc/main.h` 里能看到** `STEP_DIR_Pin` `STEP_ENA_Pin` `LIMIT_Z_Pin` 三个宏 → 说明 User Label 填对了
2. **`MDK-ARM/Arm.uvprojx` 的 Include Paths 里有 `..\User\bsp`** → 说明 8.2 跑成功了
3. **时钟树最右边显示 `72 MHz`**，APB1 显示 `36 MHz` → 时钟没配错
4. **`gpio.c` 里 PB12 那行是 `GPIO_MODE_IT_FALLING`** → 限位边沿配对了

四条全过，就可以把 `软件设计\code\User\` 下的代码拷进去了。
