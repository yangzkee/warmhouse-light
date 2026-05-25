/**
 * @file odom_y_calib.h
 * @brief 实车标定：沿底盘前进方向行驶，当 ODOM 的 N_PosY（与调试行中 py 同源：NavOdom_GetY()）达到给定绝对米数时停车。
 *
 * 与 DF 协议一致：sendVel 的 V_y>0 为前进；g_odom.pos_y 为 N_PosY（米），NavOdom_GetY() = pos_y × 当前 profile 线比例。
 *
 * 停车条件（唯一）：scaled 后的 py ≥ ODOM_Y_CALIB_PY_STOP_M（默认 1.0f），即「里程读数到 1m（该轴）就停」，不是从起点再积 1m。
 * 前提：前进时 py 单调向目标侧变化；若你车前进反而减小 py，需改命令符号或另开任务讨论坐标约定。
 *
 * 使用：将 ODOM_Y_1M_CALIB_TEST 改为 1 后编译烧录；场地清空后上电，[YCAL] armed 后小车前进，至 py≥目标后刹停。
 */
#ifndef ODOM_Y_CALIB_H
#define ODOM_Y_CALIB_H

#include "main.h"

#ifndef ODOM_Y_1M_CALIB_TEST
#define ODOM_Y_1M_CALIB_TEST  0 /* 改为 1：仅 N_PosY 绝对阈值标定，不巡线 */
#endif

/** 停车用的绝对 py（米，与 NavOdom_GetY / OXY 的 py 一致）；默认 1.0 = 「py=1 时停」 */
#ifndef ODOM_Y_CALIB_PY_STOP_M
#define ODOM_Y_CALIB_PY_STOP_M  1.0f
#endif

/* 前进速度（与巡线同单位，见 sendVel） */
#ifndef ODOM_Y_CALIB_VY
#define ODOM_Y_CALIB_VY  15.0f
#endif

#ifndef ODOM_Y_CALIB_TIMEOUT_MS
#define ODOM_Y_CALIB_TIMEOUT_MS  120000U
#endif

void OdomYCalib_Init(void);
void OdomYCalib_Tick(void);

#endif /* ODOM_Y_CALIB_H */
