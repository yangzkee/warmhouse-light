/**
 * @file debug_odom_uart.h
 * @brief 调试串经 USART3（LoRa 无线口，PA5/PB0，115200），与 EWM22A 共用。
 *
 * 使用：1) MX_USART3_UART_Init 与 EWM22A_StartReceiveIT（main 已配）；
 *       2) 本头文件中 DEBUG_ODOM_UART_ENABLE 为 1（默认 1，LoRa 看 OXY；不需要可在工程里改为 0）；
 *       3) 对端串口选 115200 8N1（经 LoRa 透传）。
 *          默认 OXY 短行：m,px,py,ms,mlk + 转弯/雷达字符（m/px/py 均为米）；
 *          ms=地图 +1/-1；mlk=已锁定。
 *          LoRa 慢：多半是空口瓶颈；短行利于实时性。
 *       4) LT_RADAR 且 DEBUG_ODOM_UART_FULL_LINE=1 时在同前缀后再追加 rs/rg_c/lm_c/rm_c/rpen 等（台架调试用）。
 */
#ifndef DEBUG_ODOM_UART_H
#define DEBUG_ODOM_UART_H

#include "main.h"

#ifndef DEBUG_ODOM_UART_ENABLE
#define DEBUG_ODOM_UART_ENABLE  0
#endif

/*
 * OXY 行周期（USART3→LoRa）。含 px/yaw/vx/vy 后约 120～160 字节量级。
 * 默认 12ms 周期；再低易与发送阻塞打架。
 *
 * 读数：路程 m = NavOdom_GetTotalDistanceM()；与 px/py 同单位。
 */
#ifndef DEBUG_ODOM_UART_PERIOD_MS
#define DEBUG_ODOM_UART_PERIOD_MS  100U
#endif
/* 0=短 OXY（无 rs/rg_c/lm_c/rm_c/rpen/obs），LoRa 友好；1=含雷达完整字段 */
#ifndef DEBUG_ODOM_UART_FULL_LINE
#define DEBUG_ODOM_UART_FULL_LINE  0
#endif

void DebugOdomUart_Tick(void);
/* 上电发固定 ASCII，不经过 snprintf/sprintf；用于确认 LoRa 口 115200/接线（应立刻看到一行） */
void DebugUsart3_BootPing(void);

#endif
