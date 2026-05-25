/**
 * @file LineTracking.h
 * @brief 8路灰度传感器巡线控制（基于Dcar sendVel）
 */
#ifndef __LINE_TRACKING_H
#define __LINE_TRACKING_H

#include "main.h"

/* 主循环周期（ms）；巡线 PID 与 sendVel 仅在解析到新光电包时更新，积分 dt=相邻两包间隔 */
#define LINE_TRACK_LOOP_MS  2
/* sendVel_NoWait 最小间隔（ms）；10=约 100Hz，减轻 USART1 占用、利于其它串口收包。0=不节流（与主循环同频发） */
#ifndef LINE_TRACK_SENDVEL_PERIOD_MS
#define LINE_TRACK_SENDVEL_PERIOD_MS  10
#endif
/* 每触发一个 nav_route 节点后刹停若干毫秒（发零速），便于调试；0=关闭 */
#ifndef LT_ROUTE_NODE_DEBUG_PAUSE_MS
#define LT_ROUTE_NODE_DEBUG_PAUSE_MS  0U
#endif
/* 巡线核心与 XUNXIANGIT 工程对齐；路径节点表仍以本工程 Core/Src/nav_route.c 为准 */

/*
 * 底盘 sendVel：Vz>0 顺时针=右转，Vz<0 逆时针=左转（与 LineTracking.c 头注释一致）。
 * err：线在车身左侧 err>0，右侧 err<0。正常巡线 vz_xun=-pid_out（err>0→pid>0→vz_xun<0→左转）。
 *
 * LINE_TRACK_INVERT_STEER_CMD：对最终下发 vz 再乘 ±1（含丢线 LostApplyOpposite、盲冲、自转），与 PID 同链。
 * 0：下发 vz = vz_xun（上式已与 Dcar 符号一致时用这个）。
 * 1：下发 vz = -vz_xun（仅当底盘与上式整段反号时开）。
 * 若仍反，再查 ir_grayscale.h 的 IR_GRAYSCALE_INVERT_DIGITAL（仅数字 0/1 与「0=黑」颠倒时）。
 */
#ifndef LINE_TRACK_INVERT_STEER_CMD
#define LINE_TRACK_INVERT_STEER_CMD  0
#endif

/* 1：路表 NAV_ROUTE_RADAR_ARM 触发后遇 T 字全黑则 USART4 雷达选支路（当前路表无 RADAR_ARM，详见 nav_route）；0：不参与编译 */
#ifndef LT_RADAR_BRANCH_ENABLE
#define LT_RADAR_BRANCH_ENABLE  1
#endif
/* 1：扫完选支路后 USART3 发两行——RADAR_PICK=L|R（ASCII）+「雷达:选择左/右边」UTF-8；0：不发送 */
#ifndef LT_RADAR_PRINT_CHOICE_LPUART1
#define LT_RADAR_PRINT_CHOICE_LPUART1  0
#endif
/* 雷达左右中位数与 sendrot 几何对齐：见 radar_obstacle.h 中 LT_RADAR_SWAP_LR_MEDIAN、LT_RADAR_SCAN_YAW_SIGN */

/* 上电后调用一次：重置里程与路线节点索引（发车前） */
void LineTracking_Init(void);

// 巡线单步执行，返回1表示本周期处理了新的传感器数据
u8 LineTracking_Step(void);

// 巡线主循环（阻塞，持续巡线直到停止）
void LineTracking_Run(void);

/* 1=已触发 nav_route 的 NAV_ROUTE_STOP，巡线任务结束（持续发 0 速） */
u8 LineTracking_IsHalted(void);

/* 地图 ms：+1 表定 / -1 镜像；默认不翻巡线 err，只影响迷宫横移与 8 字等（见 LT_MAP_DETECT_MIRROR_ERR_RAW） */
s8 LineTrack_GetMapMirror(void);
u8 LineTrack_IsMapDetectLocked(void);
/* 与判场/OLED 同单位：nav_odom 横向位移 cm（取整）；无 ODOM 时 0 */
long LineTrack_GetMapDetectXcm(void);
/* 1=已由 |xcm| 门槛或 x_ref 漂移闩住 mirror；0=可能尚未判出或仅靠 FALLBACK 超时锁定（见 LineTracking 内 s_map_odom_latched） */
u8 LineTrack_IsMapOdomLatched(void);

/* 雷达选支路完成后 OLED 保持该画面时长（ms），与 oled.c 周期无关 */
#ifndef LT_OLED_RADAR_PICK_SHOW_MS
#define LT_OLED_RADAR_PICK_SHOW_MS  2000u
#endif

/* OLED status (English 5x7 in oled.c). Only when USE_LINE_TRACKING && OLED_ENABLE */
typedef enum {
  LINE_TRACK_OLED_TRACK = 0,
  LINE_TRACK_OLED_LOST,
  LINE_TRACK_OLED_STEER_L,
  LINE_TRACK_OLED_STEER_R,
  LINE_TRACK_OLED_RADAR,
  LINE_TRACK_OLED_RADAR_PICK_L,
  LINE_TRACK_OLED_RADAR_PICK_R
} line_track_oled_disp_t;

line_track_oled_disp_t LineTrack_GetOledDisplayStatus(void);

#endif
