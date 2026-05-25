/**
 * @file radar_oled_vis.c
 * @brief 距离-时间瀑布图，SSD1306 列页格式；H=RADAR_VIS_BITMAP_H（见 oled.h，128x64 屏上为 64 全高）。
 *       新列在右侧；距→纵轴（0~RADAR_VIS_MAX_RANGE_M 映射全高）。亮纹加宽（±8px + 5 列同绘）。
 */
#include "radar_oled_vis.h"
#include <string.h>

#define RADAR_VIS_NPAGE  (RADAR_VIS_BITMAP_H / 8u)
/* 最大 8 页(64 像素)，避免与 H=32/64 两条路径重复定义不同缓冲大小 */
#define RADAR_VIS_BMP_BUFSZ  (RADAR_VIS_BITMAP_W * 8u)

#ifndef RADAR_VIS_ISR_QUEUE_DEPTH
#define RADAR_VIS_ISR_QUEUE_DEPTH  24u
#endif

typedef struct
{
  float r_m;
  float a_deg;
} RadarVisSample_t;

static volatile uint8_t s_qh;
static volatile uint8_t s_qt;
static RadarVisSample_t s_q[RADAR_VIS_ISR_QUEUE_DEPTH];

/* SSD1306 页主序：index = page * 128 + x；缓冲按最大 64px 高预置 */
static uint8_t s_bmp[RADAR_VIS_BMP_BUFSZ];

/* 亮纹：±RADAR_VIS_MARK_HALF_PX 像素高，全屏 64px 时更易辨认 */
static void radar_vis_col_mark_thick(uint8_t *col, unsigned row)
{
  int d;
  for (d = -RADAR_VIS_MARK_HALF_PX; d <= RADAR_VIS_MARK_HALF_PX; d++)
  {
    int rr = (int)row + d;
    unsigned pg, bit;
    if (rr < 0 || (unsigned)rr >= RADAR_VIS_BITMAP_H)
    {
      continue;
    }
    pg = (unsigned)rr / 8u;
    bit = (unsigned)rr % 8u;
    col[pg] = (uint8_t)(col[pg] | (uint8_t)(1u << bit));
  }
}

static unsigned radar_vis_range_to_row(float range_m)
{
  float t;
  unsigned r;

  if (range_m <= 0.0f)
  {
    return RADAR_VIS_BITMAP_H;
  }
  if (range_m >= RADAR_VIS_MAX_RANGE_M)
  {
    t = 0.0f;
  }
  else
  {
    t = (1.0f - (range_m / RADAR_VIS_MAX_RANGE_M)) * (float)(RADAR_VIS_BITMAP_H - 1u);
  }
  r = (unsigned)(t + 0.5f);
  if (r >= RADAR_VIS_BITMAP_H)
  {
    r = RADAR_VIS_BITMAP_H - 1u;
  }
  return r;
}

void RadarVis_Reset(void)
{
  s_qh = 0u;
  s_qt = 0u;
  (void)memset(s_bmp, 0, sizeof(s_bmp));
}

void RadarVis_EnqueueFrame_ISR(float range_m, float angle_deg)
{
  uint8_t next;
  uint8_t qh = s_qh;

  next = (uint8_t)((qh + 1u) % RADAR_VIS_ISR_QUEUE_DEPTH);
  if (next == s_qt)
  {
    s_qt = (uint8_t)((s_qt + 1u) % RADAR_VIS_ISR_QUEUE_DEPTH);
  }
  s_q[qh].r_m = range_m;
  s_q[qh].a_deg = angle_deg;
  s_qh = next;
}

static void radar_vis_apply_one_frame(float range_m, float angle_deg)
{
  uint8_t col[RADAR_VIS_NPAGE];
  unsigned p;
  unsigned row;

  (void)angle_deg;

  for (p = 0u; p < RADAR_VIS_NPAGE; p++)
  {
    uint8_t *base = &s_bmp[p * RADAR_VIS_BITMAP_W];
    (void)memmove(&base[0], &base[1], (size_t)(RADAR_VIS_BITMAP_W - 1u));
  }

  (void)memset(col, 0, sizeof(col));

  row = radar_vis_range_to_row(range_m);
  if (row < RADAR_VIS_BITMAP_H)
  {
    radar_vis_col_mark_thick(col, row);
  }

  for (p = 0u; p < RADAR_VIS_NPAGE; p++)
  {
    uint8_t b;
    unsigned c;

    b = col[p];
    /* 最右 RADAR_VIS_COL_THICK 列同值：水平加粗 */
    for (c = 0u; c < RADAR_VIS_COL_THICK; c++)
    {
      unsigned x = (unsigned)RADAR_VIS_BITMAP_W - 1u - c;
      s_bmp[p * RADAR_VIS_BITMAP_W + x] = b;
    }
  }
}

void RadarVis_DrainPendingToWaterfall(void)
{
  for (;;)
  {
    uint8_t qt;
    uint8_t qh;
    RadarVisSample_t s;

    qh = s_qh;
    qt = s_qt;
    if (qt == qh)
    {
      break;
    }
    s = s_q[qt];
    s_qt = (uint8_t)((qt + 1u) % RADAR_VIS_ISR_QUEUE_DEPTH);
    radar_vis_apply_one_frame(s.r_m, s.a_deg);
  }
}

const uint8_t *RadarVis_GetBitmap(void)
{
  return s_bmp;
}
