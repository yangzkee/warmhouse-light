/**
 * @file nav_sense.h
 * @brief 感知层：8 路红外 → 原始 pattern / 粗略路口类型（与 LineTracking 约定一致）
 *
 * 约定（与 ir_grayscale + LineTracking 一致）：
 * - IR_Data_number[i]：0=黑线，1=白底
 * - pattern = (L1<<7)|...|(L8)，某位为 0 表示该探头在黑线上
 */
#ifndef NAV_SENSE_H
#define NAV_SENSE_H

#include "main.h"

typedef enum {
	NAV_JUNC_NONE = 0,       /* 普通循迹段 */
	NAV_JUNC_LIKELY,         /* 多路见黑，疑似路口（需连续帧确认） */
	NAV_JUNC_LOST_ALL_WHITE  /* pattern==0xFF 全白，丢线 */
} nav_junc_hint_t;

/* 由 8 路数字量生成 8bit pattern（与 Track_Err 一致） */
u8 NavSense_BuildPattern(void);

/* 统计“见黑”探头数量（0=黑） */
u8 NavSense_BlackCount(u8 pattern);

/* 粗略路口提示：不替代 LineTracking 的 PID，仅供决策层参考 */
nav_junc_hint_t NavSense_JunctionHint(u8 pattern, u8 black_min);

/* 连续 need_frames 帧“疑似路口”才置 1（防抖），用于以后路口+拓扑决策 */
void NavSense_ResetJunctionFilter(void);
u8 NavSense_JunctionConfirmed(u8 black_min, u8 need_frames);

#endif /* NAV_SENSE_H */
