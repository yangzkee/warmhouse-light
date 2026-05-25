# Warmhouse Light

这是 Warmhouse 项目的 Light 光电巡线固件工程。仓库已经整理成 Keil MDK 可以直接打开的结构：下载或 clone 后，仓库根目录就是工程根目录，不需要再移动文件。

## 快速使用

1. 安装 Keil MDK-ARM 5。
2. 在 Keil Pack Installer 中安装 `Keil.STM32G0xx_DFP`，建议版本 `2.1.0` 或更新版本。
3. 双击 `open_project_in_keil.bat`，或手动打开 `MDK-ARM/light_test.uvprojx`。
4. 在 Keil 中选择目标 `light_test`，点击 Build/Rebuild。
5. 编译产物会生成到 `MDK-ARM/light_test/light_test.hex`。

如果只需要直接烧录，可使用仓库内已附带的固件：

```text
firmware/light.hex
```

## 一键编译

Windows 上安装 Keil 后，可以双击：

```text
build_with_keil.bat
```

脚本会尝试自动寻找 `UV4.exe`，调用 Keil 重新编译 `light_test` 目标，并在成功后把新的 HEX 同步到 `firmware/light.hex`。

## 目录说明

```text
Core/                         STM32 应用源码
Drivers/                      STM32 HAL、CMSIS 驱动依赖
MDK-ARM/light_test.uvprojx    Keil uVision 工程文件
MDK-ARM/light_test.uvoptx     Keil 工程选项
firmware/light.hex            已编译好的可烧录固件
docs/                         巡线原理、入门指南、代码速查文档
tools/                        OLED 字库/位图生成辅助脚本
light_test.ioc                CubeMX 工程配置参考
LIGHT.pdf                     工程说明文档
```

## 复刻注意事项

- Keil 工程中的源码和头文件引用使用相对路径，仓库整体解压后不要只单独拷贝 `MDK-ARM` 文件夹。
- 当前工程目标芯片是 `STM32G071CBUx`。
- 当前工程记录的 Pack 是 `Keil.STM32G0xx_DFP.2.1.0`。
- 建议以源码和 Keil 工程为准；不要随意用 CubeMX 重新生成代码，否则可能覆盖手工维护过的外设配置。
- 如果 Keil 提示缺少 Device Pack，先在 Pack Installer 安装 STM32G0xx DFP 后再打开工程。
- 如果提示找不到 Arm Compiler 5，请在 Keil 的 Project Options 中选择已安装的 Arm Compiler 5.x，或在 Keil 中安装兼容编译器。

## 资料来源与许可

本仓库按教学/复刻资料公开整理。ST HAL/CMSIS、芯片资料、模块手册等第三方内容遵循其原厂许可或发布条款；项目自有代码如需明确开源许可证，可后续补充 `LICENSE` 文件。
