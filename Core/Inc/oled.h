/**
 * @file oled.h
 * @brief SSD1315 / SSD1306 on I2C1 (PB6/PB7): centered status bitmaps.
 *
 * Target: common 1.3 inch I2C OLED — still 128x64 pixels (only glass is larger). Same init as 0.96".
 * For a 128x32 strip: #define OLED_PANEL_HEIGHT  32u before include.
 *
 * Many modules (incl. some 1.3") are SH1106-like (132 columns); right-edge garbage: default
 *   OLED_GDRAM_COL_START  2u
 * Pure SSD1306 with no offset: #define OLED_GDRAM_COL_START  0u before #include "oled.h".
 */
#ifndef OLED_H
#define OLED_H

#include "main.h"

#ifndef OLED_ENABLE
#define OLED_ENABLE  1
#endif

#ifndef OLED_PANEL_HEIGHT
#define OLED_PANEL_HEIGHT  64u
#endif

#if (OLED_PANEL_HEIGHT != 32u) && (OLED_PANEL_HEIGHT != 64u)
#error OLED_PANEL_HEIGHT must be 32 or 64
#endif

/* Radar waterfall height: match panel (64px panel → 64px bitmap, full-screen blit). */
#if !defined(RADAR_VIS_BITMAP_H)
#  if OLED_ENABLE
#    define RADAR_VIS_BITMAP_H  OLED_PANEL_HEIGHT
#  else
#    define RADAR_VIS_BITMAP_H  32u
#  endif
#endif

/* 全屏 I2C 刷新耗时长；加大可让主循环更多周期保持短节拍（LINE_TRACK_LOOP_MS），减轻巡线发僵 */
#ifndef OLED_UI_PERIOD_MS
#define OLED_UI_PERIOD_MS  200u
#endif

#ifndef OLED_GDRAM_COL_START
/* SH1106-style 132-column controllers often need 2 to align 128 visible columns. */
#define OLED_GDRAM_COL_START  2u
#endif

void Oled_Init(void);
void Oled_Tick(void);

#endif /* OLED_H */
