#ifndef RADAR_OBSTACLE_H
#define RADAR_OBSTACLE_H

#include "main.h"

/* 等底盘旋转完成帧超时（ms），超时则强行进入下一步，避免卡在 T 字 */
#ifndef RADAR_OBSTACLE_ROT_TIMEOUT_MS
#define RADAR_OBSTACLE_ROT_TIMEOUT_MS  12000u
#endif
/* 单侧雷达采样等满 10 点的超时（ms）；超时则用已采到的点算中位数，无数据则用默认距离 */
#ifndef RADAR_OBSTACLE_SAMPLE_TIMEOUT_MS
#define RADAR_OBSTACLE_SAMPLE_TIMEOUT_MS  8000u
#endif
#ifndef RADAR_OBSTACLE_SAMPLE_FALLBACK_M
#define RADAR_OBSTACLE_SAMPLE_FALLBACK_M  2.5f
#endif
/*
 * 单侧连续 10 帧距离为 0 时写入的中位等效距离（米）。
 * 旧参考工程用 5.0 表示「无障碍」；很多毫米波在极近/强反射/失锁时也输出 0，
 * 左支有障时会被误判为「左很空」而 PICK_LEFT。改为小值表示「极近/不可用」以走另一侧。
 */
#ifndef RADAR_OBSTACLE_ZEROS_IMPLY_BLOCKED_M
#define RADAR_OBSTACLE_ZEROS_IMPLY_BLOCKED_M  0.12f
#endif

/*
 * 第二次采样结束、车体回正(-45)之前，是否交换 left/right 中位数与零计数。
 * 设计意图：先 sendrot 左偏 45° 采「左支路」、再右偏 90° 采「右支路」。
 * 若底盘 SUB_CMD_ROTATION 的 rz 与 sendVel 的 Vz 符号相反，则第一次转动实际朝右，
 * 数据会进 left_median → lm_c/rm_c、obs、选支路全反。置 1 做几何对齐（实车不对再改 0）。
 */
#ifndef LT_RADAR_SWAP_LR_MEDIAN
#define LT_RADAR_SWAP_LR_MEDIAN  1
#endif
/* 三处探头 sendrot 的 rz 同乘此符号；1=与参考一致，-1=整体左右翻转（与 SWAP 择一调参即可） */
#ifndef LT_RADAR_SCAN_YAW_SIGN
#define LT_RADAR_SCAN_YAW_SIGN  1.0f
#endif

typedef enum {
	RADAR_SCAN_IDLE,
	RADAR_SCAN_TURN_LEFT_45,
	RADAR_SCAN_WAIT_LEFT_DONE,
	RADAR_SCAN_DELAY_LEFT,
	RADAR_SCAN_SAMPLING_LEFT,
	RADAR_SCAN_TURN_RIGHT_90,
	RADAR_SCAN_WAIT_RIGHT_DONE,
	RADAR_SCAN_DELAY_RIGHT,
	RADAR_SCAN_SAMPLING_RIGHT,
	RADAR_SCAN_TURN_BACK_45,
	RADAR_SCAN_COMPARE,
	RADAR_SCAN_DONE
} RadarScanState_t;

void Radar_StartScan(void);
void Radar_UpdateObstacleScan(void);
RadarScanState_t Radar_GetScanState(void);

float Radar_GetLeftDist(void);
float Radar_GetRightDist(void);
u8 Radar_GetLeftZeroCount(void);
u8 Radar_GetRightZeroCount(void);
void Radar_Reset(void);

#endif
