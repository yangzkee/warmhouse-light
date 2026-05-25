# LIGHT 巡线工程入门指南（STM32G071 + Keil MDK）

本文档面向初学者，说明如何从零理解本工程、如何用 **STM32CubeMX** 配置硬件、如何用 **Keil MDK-5** 编译与下载程序，并逐块解释代码职责。

---

## 目录

1. [你需要准备什么](#1-你需要准备什么)
2. [工程与文件夹在说什么](#2-工程与文件夹在说什么)
3. [用 STM32CubeMX 打开与理解配置](#3-用-stm32cubemx-打开与理解配置)
4. [CubeMX 关键配置项对照表](#4-cubemx-关键配置项对照表)
5. [用 Keil MDK-5 打开工程、编译、下载](#5-用-keil-mdk-5-打开工程编译下载)
6. [代码结构总览](#6-代码结构总览)
7. [各模块详细说明](#7-各模块详细说明)
8. [巡线运行时数据怎么走](#8-巡线运行时数据怎么走)
9. [接线与波特率（与代码一致）](#9-接线与波特率与代码一致)
10. [常见问题](#10-常见问题)

---

## 1. 你需要准备什么

| 项目 | 说明 |
|------|------|
| 电脑 | Windows 10/11 常见 |
| STM32CubeMX | 建议 6.x 及以上（本工程由 6.14.1 创建） |
| Keil MDK-5 | 需安装 **ARM Compiler 5** 或兼容工具链；本工程 `uvprojx` 面向 **MDK-ARM** |
| ST 芯片支持包 | **STM32G0xx DFP**（Keil 里 Pack Installer 安装） |
| 调试器 | **ST-Link**（或兼容 SWD 的下载器） |
| USB 线 | 连接 ST-Link 与电脑 |
| 目标板 | 本工程芯片：**STM32G071CBUx**（UFQFPN48） |

> 若你只有 STM32CubeIDE 而没有 Keil，需要自行迁移工程，本文以 **Keil + CubeMX** 为主线。

---

## 2. 工程与文件夹在说什么

工程根目录（以 `light_test` 文件夹为例）大致结构：

```
light_test/
├── light_test.ioc              ← CubeMX 工程文件（引脚、时钟、外设）
├── Core/
│   ├── Inc/                   ← 用户与 HAL 头文件
│   └── Src/                   ← 用户应用代码 + HAL 生成代码
├── Drivers/
│   ├── CMSIS/                 ← 内核与启动
│   └── STM32G0xx_HAL_Driver/  ← ST 官方 HAL 库源码
├── MDK-ARM/
│   ├── light_test.uvprojx      ← Keil 工程（双击打开）
│   ├── light_test.uvoptx
│   └── startup_stm32g071xx.s    ← 启动汇编
└── docs/
    └── LIGHT巡线工程入门指南.md  ← 本文档
```

- **`.ioc`**：图形化配置“哪个引脚干什么、时钟多少、开哪些中断”，可反复修改并 **Generate Code** 生成 `Core/Src` 里带 `USER CODE` 标记的文件。
- **`Core/Src/main.c` 等**：Cube 会生成初始化框架；**USER CODE BEGIN/END** 之间的内容在重新生成时**一般会被保留**（勿删这些注释对）。
- **巡线相关应用代码**：主要在 `Core/Src` 下的 `LineTracking.c`、`DF_Communication.c`、`nav_*.c`、`ir_grayscale.c` 等。

---

## 3. 用 STM32CubeMX 打开与理解配置

### 3.1 打开工程

1. 启动 **STM32CubeMX**。
2. **File → Open Project**，选择工程里的 **`light_test.ioc`**。
3. 等待加载完成，界面中央是 **芯片引脚图**。

### 3.2 确认芯片型号

- 在 CubeMX 右上角或 **Project Manager** 中应显示：**STM32G071CBUx**，封装 **UFQFPN48**。
- 与 BOM/丝印 **STM32G071CBU6** 对应（同一芯片不同写法）。

### 3.3 看引脚功能（Pinout）

在 **Pinout view** 中，已配置的引脚会显示绿色并带功能名，本工程主要包括：

| 引脚 | 信号（.ioc 中） | 用途（固件中） |
|------|-----------------|----------------|
| PF0 / PF1 | HSE 晶振 | 外部高速时钟输入（若硬件焊接晶振） |
| PA9 / PA10 | USART1_TX / RX | **底盘 Dcar 协议** |
| PA2 / PA3 | USART2_TX / RX | **8 路灰度传感器** |
| PB10 / PB11 | LPUART1_RX / TX | **调试串口**（应用里把宏名 `huart3` 映射到 LPUART1） |
| PA5 / PB0 | USART3_TX / RX | 可选第三路 USART（main 里默认未初始化） |
| PA0 / PA1 | USART4_TX / RX | 可选第四路 USART（main 里默认未初始化） |
| PB8 / PB9 | I2C1_SCL / SDA | OLED 等（示例工程保留） |
| PB6 / PB7 | GPIO_Output | 示例 LED/测试输出 |
| PA13 / PA14 | SWDIO / SWCLK | **下载调试**（勿占用为普通 GPIO） |

> **重要**：`usart.c` 里在 USER 区可能改过 **波特率**（例如 USART1/LPUART1 为 **460800**）。CubeMX 里显示的默认值（如 LPUART 115200）**以最终 `Core/Src/usart.c` 为准**。改 `.ioc` 后重新生成代码时，注意不要覆盖你已手改的 USER 代码。

### 3.4 时钟 RCC（Clock Configuration）

1. 切到 **Clock Configuration** 标签页。
2. 本工程 `.ioc` 中系统时钟目标约 **64 MHz**（PLL 来自 HSI 等，以 Cube 计算为准）。
3. 若你的硬件使用 **外部 HSE**，需在 **RCC** 里使能 HSE，并保证 **HSE_VALUE** 与 `system_stm32g0xx.c` / 工程定义一致（常见 8 MHz 或 25 MHz）。**改晶振频率后必须重配 PLL，否则串口波特率会全错。**

### 3.5 外设参数（Configuration）

1. 打开 **Pinout & Configuration** 左侧树状图。
2. 依次点 **Connectivity → USART1 / USART2 / LPUART1 / …**，可看到模式为 **Asynchronous**。
3. **Parameter Settings** 里可看到波特率等；再次提醒：**与 `usart.c` 中实际初始化是否一致**要核对。

### 3.6 NVIC（中断）

在 **System Core → NVIC** 中，本工程使能了例如：

- `USART1 global interrupt`
- `USART2 global interrupt`
- `USART3, USART4, LPUART1 global`（G0 上常合并为一条向量）
- `TIM7`（用作 HAL 时基时）

应用依赖 **串口中断** 收字节，请勿随意关闭。

### 3.7 生成代码（第一次或改 .ioc 后）

1. **Project Manager** 标签：
   - **Toolchain / IDE**：选择 **MDK-ARM V5**（与当前 Keil 工程一致）。
   - 设置工程名、路径（谨慎覆盖，建议先备份）。
2. 点右上角 **GENERATE CODE**。
3. 用 Keil 重新打开工程编译。

> 若你**只读文档不改 .ioc**，可以**不点生成**，避免覆盖本地修改。

---

## 4. CubeMX 关键配置项对照表

| 配置类 | 在 CubeMX 中的位置 | 说明 |
|--------|-------------------|------|
| 器件 | New Project → MCU Selector | STM32G071CBUx |
| 串口模式 | USARTx → Mode = Asynchronous | 异步 UART |
| 引脚复用 | Pinout 里点选引脚 | 自动分配 AF |
| 全局中断 | NVIC | 使能串口、TIM7 等 |
| HAL 时基 | SYS → Timebase Source | 本工程为 **TIM7**（见 `stm32g0xx_hal_timebase_tim.c`） |
| 调试口 | SYS → Debug | **Serial Wire**（保留 SWD） |

---

## 5. 用 Keil MDK-5 打开工程、编译、下载

### 5.1 打开工程

1. 进入文件夹 **`MDK-ARM`**。
2. 双击 **`light_test.uvprojx`**（Keil 工程文件）。

### 5.2 选择目标芯片 Pack（首次）

若提示缺少 **Device Family Pack**：

1. 打开 **Pack Installer**（Keil 菜单或图标）。
2. 搜索 **STM32G0**，安装 **Keil.STM32G0xx_DFP** 等官方包。

### 5.3 配置调试器（ST-Link）

1. 菜单 **Project → Options for Target ‘light_test’**（或魔术棒图标）。
2. 选 **Debug** 标签 → **Use**: **ST-Link Debugger**（或你实际使用的）。
3. 点 **Settings**：
   - **Port**: SW。
   - 能 **Scan** 到芯片说明接线正确。
4. **Flash Download** 标签：勾选 **Reset and Run**（可选，方便下载后自动跑）。

### 5.4 编译

1. 菜单 **Project → Rebuild all target files**（或快捷键 **F7**）。
2. 下方 **Build Output** 应显示 **0 Error(s)**。若有错误，根据提示双击定位。

### 5.5 下载到芯片

1. 用 **SWD** 连接 **ST-Link 与板子**：**SWDIO、SWCLK、GND、3.3V**（供电按板子要求）。
2. 目标板 **上电**。
3. 点 **Download** 按钮（或 **F8**）。
4. 成功会提示 Programming Finished。

### 5.6 在线调试（可选）

1. 点 **Debug** 进入调试模式。
2. 可设断点、单步、看 **Peripherals** 寄存器（需配置 SVD）。

### 5.7 生成 HEX（可选）

**Options for Target → Output** 勾选 **Create HEX File**，编译后在 `MDK-ARM/light_test/` 下生成 `.hex`，可用其它工具烧录。

---

## 6. 代码结构总览

### 6.1 分层示意

```
┌─────────────────────────────────────────────┐
│  main.c：上电初始化、主循环、UART 回调       │
├─────────────────────────────────────────────┤
│  LineTracking.c：巡线状态机、PID、路线      │
│  RoutePlan.c / nav_route.c / nav_path.c   │
│  nav_odom.c / nav_sense.c                   │
├─────────────────────────────────────────────┤
│  DF_Communication.c：Dcar 协议、发速度、ODOM │
│  ir_grayscale.c：灰度串口协议解析           │
│  debug_odom_uart.c / odom_telemetry.c     │
├─────────────────────────────────────────────┤
│  HAL：GPIO、UART、I2C、TIM、RCC…            │
│  stm32g0xx_it.c：中断服务程序入口           │
└─────────────────────────────────────────────┘
```

### 6.2 Keil 工程组（与左侧工程树类似）

- **Application/MDK-ARM**：启动文件。
- **Application/User/Core**：`main.c`、`gpio.c`、`usart.c`、`stm32g0xx_it.c`、`stm32g0xx_hal_msp.c`、`stm32g0xx_hal_timebase_tim.c`、应用 `.c` 等。
- **Drivers**：HAL 与 CMSIS。

---

## 7. 各模块详细说明

### 7.1 `main.c`

| 内容 | 作用 |
|------|------|
| `HAL_Init()` | 初始化 HAL 库 |
| `SystemClock_Config()` | 配置系统时钟（PLL、总线分频） |
| `MX_*_Init()` | 调用 Cube 生成的外设初始化（GPIO、UART、I2C 等） |
| `MX_UserLED_GPIO_Init()` | 用户板载 LED（`PC13`）推挽输出，与 `LED_Set` 一致 |
| `HAL_UART_Receive_IT()` | 启动串口 **逐字节中断接收**（USART1/2/LPUART1） |
| `System_Init()`、`Odom_Init()` | 协议与里程相关初始化 |
| `IR_Send_Control_Data()` | 配置灰度传感器为数字输出模式 |
| `NavOdom_SetProfile()`、`LineTracking_Init()` | 里程标定与巡线初始化 |
| `while(1)` 中 `LineTracking_Step()` | 周期性执行巡线一步（内部会结合新灰度包与 ODOM） |
| `HAL_UART_RxCpltCallback()` | **每收到 1 字节** 进中断回调：USART1→`Deal_DF_Usart`，USART2→`Deal_IR_Usart` |
| `HAL_UART_ErrorCallback()` | USART1 出错时清标志、复位协议解析器、重启接收 |
| `HAL_TIM_PeriodElapsedCallback()` | TIM7 作为 HAL 滴答时基时调用 `HAL_IncTick()` |
| `printf` 重定向 | 默认 **不要** 往 USART1 打打印（会破坏 Dcar 二进制流）；可关或改 LPUART |

宏 `USE_LINE_TRACKING`、`USE_USART3`、`MAIN_STARTUP_DELAY_MS` 等控制主流程与是否开调试串口。

### 7.2 `main.h`

- 包含 `stm32g0xx_hal.h`。
- 定义 `u8/u16/s32` 等别名，与旧工程类型一致。
- **`#define huart3 hlpuart1`**：把旧代码里的 `huart3` 名字映射到 **LPUART1 句柄**（避免链接不到符号）。

### 7.3 `gpio.c` / `gpio.h`

| 函数 | 作用 |
|------|------|
| `MX_GPIO_Init()` | Cube 生成的 GPIO 输出（如 PB6/PB7） |
| `MX_UserLED_GPIO_Init()` | **PC13** 配置为输出，供 `LED_Set` 控制 |

### 7.4 `usart.c` / `usart.h`

| 外设 | 句柄 | 典型用途（应用） |
|------|------|------------------|
| USART1 | `huart1` | Dcar 底盘，**460800**（以代码为准） |
| USART2 | `huart2` | 灰度模块，**115200** |
| LPUART1 | `hlpuart1` | 调试输出；代码里宏名 `huart3` 指向它 |
| USART3 | `huart3_aux` | Cube 已配，**main 里可未调用** |
| USART4 | `huart4` | Cube 已配，**main 里可未调用** |

`HAL_UART_MspInit/DeInit` 里完成 **GPIO 复用、时钟、NVIC**。

### 7.5 `stm32g0xx_it.c` / `stm32g0xx_it.h`

- 实现 **中断服务函数**（如 `USART1_IRQHandler`, `USART2_IRQHandler`, `USART3_4_LPUART1_IRQHandler`, `TIM7_LPTIM2_IRQHandler`）。
- 内部调用 `HAL_UART_IRQHandler()` / `HAL_TIM_IRQHandler()`，进而触发上面的 **回调**。

### 7.6 `stm32g0xx_hal_msp.c`

- 部分外设的 **底层初始化**（若 Cube 把部分放在此文件），与 `usart.c` 的 MSP 二选一或并存，以工程为准。

### 7.7 `stm32g0xx_hal_timebase_tim.c`

- 用 **TIM7** 产生 **1 ms 滴答**，替代 SysTick 作为 HAL 时基。
- 与 **NVIC** 中 **TIM7** 中断一致。

### 7.8 `i2c.c` / `i2c.h`

- **I2C1**（PB8/PB9）初始化，用于 OLED 等；巡线核心逻辑**不依赖** I2C，可保留。

### 7.9 `system_stm32g0xx.c`

- 上电默认时钟、向量表、复位后 `SystemInit()`。

### 7.10 `DF_Communication.c` / `.h`

- **Dcar 二进制协议**：组帧、帧头帧尾、指令类型、**`sendVel`** 下发速度/角速度。
- **`Deal_DF_Usart`**：逐字节解析，收到完整帧后解析 **ODOM**（`0x6C/0x80`）写入 **`g_odom`**。
- **`requestOdomData`**：请求里程数据。
- **`LED_Set`**：控制 **PC13** LED 亮灭（低电平点亮，按代码逻辑）。

### 7.11 `ir_grayscale.c` / `.h`

- 解析 **USART2** 上灰度模块的 **ASCII 包**（如 `$D,...#`）。
- **`Deal_IR_Usart`**：逐字节收；组包完成后置位 **`g_ir_new_package_flag`**。
- **`Deal_IR_Usart_Data`**：解析到 **`IR_Data_number[0..7]`**（0/1 数字）。
- **`IR_Send_Control_Data`**：发配置命令，切换为数字模式。

### 7.12 `nav_odom.c` / `.h`

- 基于 **`g_odom`** 做 **积分路程、累计 x/y、航向差分**、转弯方向提示等。
- **`NavOdom_SetProfile`**：步进/电机减速比等标定切换。
- **`NavOdom_UpdateStep`**：建议每控制周期调用，更新内部状态。

### 7.13 `nav_route.c` / `.h`、`nav_path.c` / `.h`

- **路线节点、触发条件**（里程、路口等），与巡线任务分段、停车等相关。

### 7.14 `nav_sense.c` / `.h`

- 把 **8 路灰度** 变成 **pattern**、岔路/丢线等感知，供上层逻辑使用。

### 7.15 `LineTracking.c` / `.h`

- **巡线核心**：PID、速度指令、与 `nav_route`、`nav_odom`、灰度 pattern 结合。
- **`LineTracking_Init`**：初始化索引与状态。
- **`LineTracking_Step`**：主循环每周期调用一次，内部若新灰度包则计算控制量并通过 **`sendVel`** 下发。

### 7.16 `RoutePlan.c` / `.h`

- 与路线/任务规划相关的逻辑（与具体比赛规则绑定，见源码注释）。

### 7.17 `debug_odom_uart.c` / `.h`

- 在 **LPUART1** 上周期性打印 **ODOM 相关文本**（如 `OXY,...`），便于串口助手/VOFA+ 观察。
- 受 **`DEBUG_ODOM_UART_ENABLE`** 等宏控制。

### 7.18 `odom_telemetry.c` / `.h`

- 可选 **CSV/文本** 遥测输出；默认常关，避免与二进制工具冲突。

---

## 8. 巡线运行时数据怎么走

1. **底盘** 通过 **USART1** 持续发送 **ODOM 等二进制数据** → `Deal_DF_Usart` → **`g_odom`** 更新。
2. **灰度** 通过 **USART2** 发送文本包 → `Deal_IR_Usart` → **`IR_Data_number`** 更新。
3. **`LineTracking_Step`** 读取新灰度与 ODOM，算 **PID/路线状态**，调用 **`sendVel`** → 仍经 **USART1** 发给底盘。
4. 调试时 **LPUART1** 打印文本，**不要** 与 USART1 混用 `printf`。

---

## 9. 接线与波特率（与代码一致）

| 链路 | MCU 引脚（当前 `usart.c`） | 波特率 | 说明 |
|------|----------------------------|--------|------|
| 底盘 Dcar | PA9 TX, PA10 RX | **460800** | TX↔RX 交叉，共地 |
| 8 路灰度 | PA2 TX, PA3 RX | **115200** | 按模块供电 |
| 调试 USB-TTL | PB11 TX, PB10 RX（LPUART1） | **460800** | TX↔RX 交叉，共地 |

若你修改了 `usart.c` 波特率或引脚，**以最新代码为准** 更新接线表。

---

## 10. 常见问题

**Q：CubeMX 里波特率和 `usart.c` 不一致？**  
A：以 **`Core/Src/usart.c`** 中 `Init.BaudRate` 为准；重新生成代码时注意保留 USER 区修改。

**Q：编译提示找不到 `huart3`？**  
A：本工程在 **`main.h`** 中定义 **`#define huart3 hlpuart1`**，请保证应用头文件先包含 **`main.h`**。

**Q：能同时用 USART1 接底盘和 `printf` 吗？**  
A：**不建议**。`printf` 会插入可打印字符，破坏 Dcar 帧，导致 ODOM 解析失败。调试用 **LPUART1**。

**Q：下载后无反应？**  
A：查 **BOOT0**、供电、SWD 线序；确认 **Reset** 与 Flash 算法选对芯片。

**Q：串口波特率不对？**  
A：检查 **系统时钟** 是否与 HSE/HSI 配置一致；波特率由 **PCLK + 外设分频** 决定。

---

## 附录 A：推荐阅读顺序（初学者）

1. 读完本文 **第 5 节** 能独立编译下载。  
2. 阅读 **`main.c`** 的 `main` 与 `HAL_UART_RxCpltCallback`。  
3. 阅读 **`DF_Communication.h`** 里 `sendVel`、`g_odom` 定义。  
4. 阅读 **`LineTracking.h`** 与 **`LineTracking.c`** 前半段（初始化与 `LineTracking_Step` 入口）。

---

## 附录 B：文档版本

- 文档针对 **STM32G071** + **HAL** + **Keil MDK** 工程结构编写。  
- 若工程路径或文件名变更，请以实际仓库为准。
