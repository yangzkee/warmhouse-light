/**
 * @file ir_grayscale.h
 * @brief 亚博智能8路灰度传感器串口通信接口
 * 
 * 协议说明（亚博智能8路巡线红外传感器）：
 * - 串口：USART2，波特率115200
 * - 数据格式：$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0# （数字型）
 * - 控制命令：$0,0,0# 格式，用于配置传感器输出模式
 * - 0=黑线/1=白线（或根据实际传感器定义）
 */
#ifndef __IR_GRAYSCALE_H_
#define __IR_GRAYSCALE_H_

#include "main.h"

/* 1：串口数字量与代码约定相反时（模块发 1=黑、0=白 等），解析后翻成 0=黑、1=白 */
#ifndef IR_GRAYSCALE_INVERT_DIGITAL
#define IR_GRAYSCALE_INVERT_DIGITAL  0
#endif

#define IR_NUM 8  // 探头数量

// 8路灰度传感器数字值（0或1），索引0~7对应传感器1~8
extern u8 IR_Data_number[IR_NUM];

// 接收到新数据包标志（1=有新数据待处理）
extern u8 g_ir_new_package_flag;

// 逐字节接收处理（由USART2中断调用，用户勿直接调用）
void Deal_IR_Usart(u8 rxtemp);

// 解析数字型数据到 IR_Data_number[]（收到新包后调用）；成功写入返回 1，否则 0（勿当 had_new）
u8 Deal_IR_Usart_Data(void);

// 发送配置命令：adjust=1校准, aData=1模拟值, dData=1数字值
void IR_Send_Control_Data(u8 adjust, u8 aData, u8 dData);

#endif
