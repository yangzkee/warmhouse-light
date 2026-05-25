/**
 * @file odom_telemetry.h
 * @brief 经 USART3（LoRa）可选输出 CSV；115200 见 usart.c
 *
 * 官方「ODOM可视化主程序.exe」需要 **二进制帧**，见 DF_Communication.h 里 ODOM_USART3_FORWARD_RAW。
 * 转发 RAW 时请保持 ODOM_TELEMETRY_ENABLE=0，避免两种数据混在同一串口。
 *
 * CSV 列：yaw, px_m, py_m, vx, vy, valid, path_m, nav_x, nav_y,
 * ms, mlk, xcm, odl（地图：+1/-1、是否锁定、横向 cm、是否 ODOM 闩住）
 */
#ifndef ODOM_TELEMETRY_H
#define ODOM_TELEMETRY_H

#include "main.h"
#include "DF_Communication.h"

/* 默认关闭：不向 USART3 打 CSV，避免占用串口与阻塞式发送；需要 VOFA+ 时在工程里改为 1 */
#ifndef ODOM_TELEMETRY_ENABLE
#define ODOM_TELEMETRY_ENABLE  0
#endif

#ifndef ODOM_TELEMETRY_PERIOD_MS
#define ODOM_TELEMETRY_PERIOD_MS  200U
#endif

void OdomTelemetry_Tick(float total_dist_m);

#endif
