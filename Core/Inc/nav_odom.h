/**
 * @file nav_odom.h
 * @brief 定位层：消费 Dcar 回传的 ODOM（g_odom），在 C8T6 上做累计弧长与 Δyaw 辅助。
 *
 * 坐标：N_PosX/N_PosY 含义与原点、+Y 朝前、左移 x 增等——见 DF_Communication.h 中「N_PosX / N_PosY」块。
 * 协议为**米**时本模块对位置**不再乘系数**（NAV_ODOM_LINEAR_SCALE_* 默认 1.0）；若协议单位变化只在此处改倍率。
 *
 * - 航向：g_odom.yaw（度）
 * - NavOdom_GetX/GetY：与 g_odom.pos_x/pos_y 同义（默认 1:1），供触发与调试统一引用
 * - NavOdom_GetTotalDistanceM：对相邻 ODOM 帧的 (x,y) 欧氏增量累加，与 N_PosX/Y 同为**米**，无再换算
 *
 * Profile（步进 / 520）仅区分单帧最大合理步长 s_max_step_m 等防跳变参数；线比例默认两套均为 1.0。
 */
#ifndef NAV_ODOM_H
#define NAV_ODOM_H

#include "main.h"

#ifndef NAV_ODOM_YAW_DELTA_DEADZONE_DEG
#define NAV_ODOM_YAW_DELTA_DEADZONE_DEG  0.18f
#endif
#ifndef NAV_ODOM_YAW_LEFT_SIGN
#define NAV_ODOM_YAW_LEFT_SIGN  1.0f
#endif

/* 协议 pos 到本模块使用单位的倍率：协议已为米时保持 1.0 */
#ifndef NAV_ODOM_LINEAR_SCALE_STEPPER
#define NAV_ODOM_LINEAR_SCALE_STEPPER  1.0f
#endif
#ifndef NAV_ODOM_MAX_STEP_M_STEPPER
#define NAV_ODOM_MAX_STEP_M_STEPPER    0.40f
#endif

#ifndef NAV_ODOM_LINEAR_SCALE_ENCODER520
#define NAV_ODOM_LINEAR_SCALE_ENCODER520  1.0f
#endif
#ifndef NAV_ODOM_MAX_STEP_M_ENCODER520
#define NAV_ODOM_MAX_STEP_M_ENCODER520      0.40f
#endif

typedef enum {
	NAV_ODOM_PROFILE_STEPPER = 0,
	NAV_ODOM_PROFILE_ENCODER520 = 1
} NavOdomProfile_t;

#ifndef NAV_ODOM_PROFILE_DEFAULT
#define NAV_ODOM_PROFILE_DEFAULT  NAV_ODOM_PROFILE_ENCODER520
#endif

typedef enum {
	NAV_TURN_UNKNOWN = 0,
	NAV_TURN_LEFT,
	NAV_TURN_RIGHT,
	NAV_TURN_STRAIGHT
} NavOdomTurnHint_t;

void NavOdom_ResetTrajectory(void);

void NavOdom_SetProfile(NavOdomProfile_t profile);
NavOdomProfile_t NavOdom_GetProfile(void);
/** 当前 profile 下 pos 的线倍率（协议米→内部用米，默认 1） */
float NavOdom_GetLinearScale(void);

void NavOdom_UpdateStep(void);

float NavOdom_GetTotalDistanceM(void);
float NavOdom_GetX(void);
float NavOdom_GetY(void);
float NavOdom_GetYawDeg(void);

float NavOdom_GetLastYawDeltaDeg(void);

NavOdomTurnHint_t NavOdom_GetTurnHint(void);

float NavOdom_GetDeltaSinceLastMarkM(void);
void NavOdom_MarkSegmentStart(void);

#endif /* NAV_ODOM_H */
