/**
 * @file radar_uart.h
 * @brief 毫米波雷达 14 字节帧解析（USART4，与底盘 USART1 / 灰度 USART2 分离）
 *
 * 协议与 vision_linetrack-planB_VET6/check_radar 一致：0xAA 0x55，cmd 0x06 0xA2，
 * 距离小端 mm×0.01→米；校验为前 12 字节和等于 buf[12]。
 *
 * 硬件（CubeMX）：USART4 TX=PA0，RX=PA1，115200 8N1。
 */
#ifndef RADAR_UART_H
#define RADAR_UART_H

#include "main.h"

#define RADAR_UART_FRAME_LEN  14u

extern float radar_distance;
extern float radar_angle;
extern volatile u8 radar_data_ready;
extern volatile u8 radar_parsing_enabled;

void RadarUart_OnRxByte(u8 b);

#endif
