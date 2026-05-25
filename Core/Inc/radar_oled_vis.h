/**
 * @file radar_oled_vis.h
 * @brief 毫米波雷达 OLED 可视化：在仅有距离/角度输出时，生成「距离-时间」瀑布图（Waterfall）帧缓冲。
 *
 * 说明：赛规要求瀑布图或频谱图、且不得直接显示原始数据。本模块对协议解析后的测距量做滚动时域成像，
 * 输出 SSD1306 页格式位图；若硬件将来提供 IQ/FFT，可在此层替换为真实频谱图而不改 OLED 与 UART 边界。
 */
#ifndef RADAR_OLED_VIS_H
#define RADAR_OLED_VIS_H

#include "oled.h"
#include "main.h"

#ifndef RADAR_VIS_BITMAP_W
#define RADAR_VIS_BITMAP_W  128u
#endif
/* RADAR_VIS_BITMAP_H: default from oled.h (64 on 128x64) */
#if !defined(RADAR_VIS_BITMAP_H)
#define RADAR_VIS_BITMAP_H  32u
#endif
#ifndef RADAR_VIS_MAX_RANGE_M
/* 0~2.5m 映射全屏纵轴，近距离场裁更易拉开（原 6m 易挤在底部一条带） */
#define RADAR_VIS_MAX_RANGE_M  2.5f
#endif
#ifndef RADAR_VIS_MARK_HALF_PX
#define RADAR_VIS_MARK_HALF_PX  8
#endif
#ifndef RADAR_VIS_COL_THICK
#define RADAR_VIS_COL_THICK  5u
#endif

void RadarVis_Reset(void);

/**
 * 在 UART 完成帧解析后调用（可位于中断）：仅入队，不做瀑布图 memmove。
 */
void RadarVis_EnqueueFrame_ISR(float range_m, float angle_deg);

/** 主循环中调用：将队列中样本依次展开为瀑布图滚动。 */
void RadarVis_DrainPendingToWaterfall(void);

const uint8_t *RadarVis_GetBitmap(void);

#endif /* RADAR_OLED_VIS_H */
