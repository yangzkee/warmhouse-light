/**
 * @file oled.c
 * @brief SSD1315/SSD1306 on I2C1 (hi2c1): 128x64/128x32; 5x7; status line 2x on h>=48; radar full-screen 64px.
 */
#include "oled.h"

#if OLED_ENABLE

#include "app_ds3231.h"
#include "i2c.h"
#include "LineTracking.h"
#include "nav_odom.h"
#include "radar_oled_vis.h"
#include <stdio.h>
#include <string.h>

#define SSD1315_CTRL_CMD    0x00u
#define SSD1315_CTRL_DATA   0x40u

/* Longer I2C timeout for GDDRAM bursts (weak pull-ups / long wires). */
#define SSD1315_I2C_TIMEOUT_MS  200u

#define SSD1315_W           128u
#define SSD1315_H           OLED_PANEL_HEIGHT
#define SSD1315_NPAGE       (SSD1315_H / 8u)
#define SSD1315_BUFLEN      (SSD1315_W * SSD1315_NPAGE)

/* HUD: time y0-7; track 16px high top-right (y0-15); distance label 16px at y16-31; path m on bottom page of that row */
#if SSD1315_H >= 48u
#define OLED_HUD_DIST_OY     16u
#define OLED_HUD_RES_TOP_PX  32u
#endif

static uint8_t s_fb[SSD1315_BUFLEN];
static uint8_t s_inited;

/* Adafruit glcdfont: 5 columns (LSB = top row), 6th column blank */
static const uint8_t s_oled_font5[10][6] = {
  { 0x3Eu, 0x51u, 0x49u, 0x45u, 0x3Eu, 0x00u },
  { 0x00u, 0x42u, 0x7Fu, 0x40u, 0x00u, 0x00u },
  { 0x42u, 0x61u, 0x51u, 0x49u, 0x46u, 0x00u },
  { 0x21u, 0x41u, 0x45u, 0x4Bu, 0x31u, 0x00u },
  { 0x18u, 0x14u, 0x12u, 0x7Fu, 0x10u, 0x00u },
  { 0x27u, 0x45u, 0x45u, 0x45u, 0x39u, 0x00u },
  { 0x3Cu, 0x4Au, 0x49u, 0x49u, 0x30u, 0x00u },
  { 0x01u, 0x71u, 0x09u, 0x05u, 0x03u, 0x00u },
  { 0x36u, 0x49u, 0x49u, 0x49u, 0x36u, 0x00u },
  { 0x06u, 0x49u, 0x49u, 0x29u, 0x1Eu, 0x00u }
};

/* 5x7, tools/gen_oled5_ascii.py — column LSB=top, +1 pad, same as digits */
static const uint8_t s_oled_cap[26][6] = {
  {0x7Eu,0x09u,0x09u,0x09u,0x7Eu,0x00u},
  {0x7Fu,0x49u,0x49u,0x49u,0x36u,0x00u},
  {0x3Eu,0x41u,0x41u,0x41u,0x22u,0x00u},
  {0x7Fu,0x41u,0x41u,0x41u,0x3Eu,0x00u},
  {0x7Fu,0x49u,0x49u,0x49u,0x41u,0x00u},
  {0x7Fu,0x09u,0x09u,0x09u,0x01u,0x00u},
  {0x3Eu,0x41u,0x49u,0x49u,0x3Bu,0x00u},
  {0x7Fu,0x08u,0x08u,0x08u,0x7Fu,0x00u},
  {0x00u,0x41u,0x7Fu,0x41u,0x00u,0x00u},
  {0x20u,0x40u,0x41u,0x5Fu,0x20u,0x00u},
  {0x7Fu,0x08u,0x14u,0x22u,0x41u,0x00u},
  {0x7Fu,0x40u,0x40u,0x40u,0x40u,0x00u},
  {0x7Fu,0x02u,0x04u,0x02u,0x7Fu,0x00u},
  {0x7Fu,0x02u,0x04u,0x08u,0x7Fu,0x00u},
  {0x3Eu,0x41u,0x41u,0x41u,0x3Eu,0x00u},
  {0x7Fu,0x09u,0x09u,0x09u,0x06u,0x00u},
  {0x3Eu,0x41u,0x51u,0x21u,0x5Eu,0x00u},
  {0x7Fu,0x09u,0x19u,0x29u,0x46u,0x00u},
  {0x66u,0x49u,0x49u,0x59u,0x33u,0x00u},
  {0x00u,0x01u,0x7Fu,0x01u,0x00u,0x00u},
  {0x3Fu,0x40u,0x40u,0x40u,0x3Fu,0x00u},
  {0x1Fu,0x20u,0x40u,0x20u,0x1Fu,0x00u},
  {0x3Fu,0x40u,0x38u,0x40u,0x3Fu,0x00u},
  {0x63u,0x14u,0x08u,0x14u,0x63u,0x00u},
  {0x03u,0x04u,0x78u,0x04u,0x03u,0x00u},
  {0x71u,0x49u,0x45u,0x43u,0x41u,0x00u}
};
static const uint8_t s_oled_font_colon[6] = { 0x00u, 0x36u, 0x36u, 0x00u, 0x00u, 0x00u };
static const uint8_t s_oled_font_dot[6] = { 0x00u, 0x00u, 0x00u, 0x00u, 0x60u, 0x00u }; /* '.' 底行点 */
static const uint8_t s_oled_font_minus[6] = { 0x08u, 0x08u, 0x08u, 0x08u, 0x08u, 0x00u };
/* HAL expects 7-bit addr shifted left; probe 0x3C then 0x3D at runtime. */
static uint16_t s_ssd1315_i2c_addr8 = ((uint16_t)(0x3Cu << 1));

static void oled_pick_i2c_addr8(void)
{
  const uint16_t cand[2] = { (uint16_t)(0x3Cu << 1), (uint16_t)(0x3Du << 1) };
  unsigned i;

  s_ssd1315_i2c_addr8 = cand[0];
  for (i = 0u; i < 2u; i++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, cand[i], 4u, 20u) == HAL_OK)
    {
      s_ssd1315_i2c_addr8 = cand[i];
      return;
    }
  }
}

static HAL_StatusTypeDef ssd1315_tx(const uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Master_Transmit(&hi2c1, s_ssd1315_i2c_addr8, (uint8_t *)buf, len, SSD1315_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef ssd1315_cmd1(uint8_t cmd)
{
  uint8_t b[2];
  b[0] = SSD1315_CTRL_CMD;
  b[1] = cmd;
  return ssd1315_tx(b, 2u);
}

static HAL_StatusTypeDef ssd1315_cmd_burst(const uint8_t *cmds, uint16_t ncmd)
{
  uint8_t tmp[48];
  if (ncmd == 0u || ncmd > (sizeof(tmp) - 1u))
  {
    return HAL_ERROR;
  }
  tmp[0] = SSD1315_CTRL_CMD;
  (void)memcpy(&tmp[1], cmds, ncmd);
  return ssd1315_tx(tmp, (uint16_t)(ncmd + 1u));
}

static HAL_StatusTypeDef ssd1315_hw_init(void)
{
  HAL_StatusTypeDef st;
  /* 128x32: MUX=31, COM=0x02; 128x64: MUX=63, COM=0x12 (Adafruit-style SSD1306). */
#if SSD1315_H == 32u
  static const uint8_t kInit[] = {
    0xAEu,
    0x2Eu,
    0xD5u, 0x80u,
    0xA8u, 0x1Fu,
    0xD3u, 0x00u,
    0x40u,
    0x8Du, 0x14u,
    0x20u, 0x00u,
    0xA1u,
    0xC8u,
    0xDAu, 0x02u,
    0x81u, 0xCFu,
    0xD9u, 0xF1u,
    0xDBu, 0x40u,
    0xA4u,
    0xA6u
  };
#elif SSD1315_H == 64u
  static const uint8_t kInit[] = {
    0xAEu,
    0x2Eu,
    0xD5u, 0x80u,
    0xA8u, 0x3Fu,
    0xD3u, 0x00u,
    0x40u,
    0x8Du, 0x14u,
    0x20u, 0x00u,
    0xA1u,
    0xC8u,
    0xDAu, 0x12u,
    0x81u, 0xCFu,
    0xD9u, 0xF1u,
    0xDBu, 0x40u,
    0xA4u,
    0xA6u
  };
#else
#error SSD1315_H must be 32 or 64
#endif

  HAL_Delay(120);
  st = ssd1315_cmd_burst(kInit, (uint16_t)sizeof(kInit));
  return st;
}

/* Page mode: write 128 zeros per page to clear power-on GDDRAM; then horizontal mode again. */
static HAL_StatusTypeDef ssd1315_clear_gddram_page_mode(void)
{
  HAL_StatusTypeDef st;
  uint8_t zpage[129];
  unsigned p;

  (void)memset(&zpage[1], 0, 128);
  zpage[0] = SSD1315_CTRL_DATA;

  {
    static const uint8_t kMemPage[] = { 0x20u, 0x02u };
    st = ssd1315_cmd_burst(kMemPage, (uint16_t)sizeof(kMemPage));
  }
  if (st != HAL_OK)
  {
    return st;
  }

  for (p = 0u; p < SSD1315_NPAGE; p++)
  {
    uint8_t ps[4];
    uint8_t cs = (uint8_t)OLED_GDRAM_COL_START;

    ps[0] = SSD1315_CTRL_CMD;
    ps[1] = (uint8_t)(0xB0u | (uint8_t)p);
    ps[2] = (uint8_t)(0x00u | (cs & 0x0Fu));
    ps[3] = (uint8_t)(0x10u | ((cs >> 4) & 0x0Fu));
    st = ssd1315_tx(ps, 4u);
    if (st != HAL_OK)
    {
      return st;
    }
    st = ssd1315_tx(zpage, 129u);
    if (st != HAL_OK)
    {
      return st;
    }
  }

  {
    static const uint8_t kMemH[] = { 0x20u, 0x00u };
    st = ssd1315_cmd_burst(kMemH, (uint16_t)sizeof(kMemH));
  }
  return st;
}

static void fb_clear(void)
{
  (void)memset(s_fb, 0, sizeof(s_fb));
}

/* src: SSD1306 page layout, w*h/8 bytes; oy must be multiple of 8; overwrites s_fb columns ox..ox+w-1 */
static void fb_blit_pages(const uint8_t *src, uint16_t w, uint8_t h, unsigned ox, unsigned oy)
{
  unsigned np;
  unsigned p;
  unsigned x;
  unsigned base_page;

  if ((h % 8u) != 0u || (oy % 8u) != 0u || w == 0u)
  {
    return;
  }
  np = (unsigned)(h / 8u);
  base_page = oy / 8u;
  for (p = 0u; p < np; p++)
  {
    if (base_page + p >= SSD1315_NPAGE)
    {
      break;
    }
    for (x = 0u; x < w; x++)
    {
      unsigned dx = ox + x;
      if (dx < SSD1315_W)
      {
        s_fb[(base_page + p) * SSD1315_W + dx] = src[p * (unsigned)w + x];
      }
    }
  }
}

static void fb_write_glyph5_1page(unsigned page, unsigned x, const uint8_t cols6[6])
{
  unsigned i;

  if (page >= SSD1315_NPAGE)
  {
    return;
  }
  for (i = 0u; i < 6u; i++)
  {
    unsigned xx = x + i;
    if (xx < SSD1315_W)
    {
      s_fb[page * SSD1315_W + xx] = cols6[i];
    }
  }
}

static const uint8_t *oled_glyph_ch(char c);

#if SSD1315_H >= 48u
/* Pixel in screen coords (0..H-1 top-down); for 2x glyphs. */
static void oled_fb_set_px(unsigned x, unsigned y)
{
  unsigned pg;
  unsigned b;

  if (x >= SSD1315_W)
  {
    return;
  }
  if (y >= SSD1315_H)
  {
    return;
  }
  pg = y / 8u;
  b = (unsigned)(y % 8u);
  s_fb[pg * SSD1315_W + x] = (uint8_t)(s_fb[pg * SSD1315_W + x] | (uint8_t)(1u << b));
}

/* 5x7 -> 10x14 (first 5 columns only; col 5 is pad) */
static void fb_write_glyph5_2x(unsigned x0, unsigned y0, const uint8_t *cols6)
{
  unsigned ci, r, xx, yy;

  for (ci = 0u; ci < 5u; ci++)
  {
    uint8_t cby = cols6[ci];
    for (r = 0u; r < 7u; r++)
    {
      if ((cby & (1u << r)) == 0u)
      {
        continue;
      }
      for (yy = 0u; yy < 2u; yy++)
      {
        for (xx = 0u; xx < 2u; xx++)
        {
          oled_fb_set_px(x0 + ci * 2u + xx, y0 + r * 2u + yy);
        }
      }
    }
  }
}

static unsigned oled_str5_2x_str_width(const char *s)
{
  unsigned w = 0u;
  const char *p;

  for (p = s; *p != '\0'; p++)
  {
    if (*p == ' ')
    {
      w += 8u; /* 4px*2, matches 1x spacing feel */
    }
    else
    {
      w += 12u; /* 6*2 */
    }
  }
  return w;
}

/* y0: top of 2x band (7 source rows -> 14 px on screen). Horizontally centered. */
static void oled_str5_2x(unsigned y0, const char *s)
{
  unsigned x;
  unsigned tot;
  const char *p;

  tot = oled_str5_2x_str_width(s);
  if (tot >= SSD1315_W)
  {
    x = 0u;
  }
  else
  {
    x = (unsigned)((SSD1315_W - tot) / 2u);
  }
  for (p = s; *p != '\0'; p++)
  {
    if (*p == ' ')
    {
      x += 8u;
      if (x >= SSD1315_W)
      {
        break;
      }
      continue;
    }
    {
      const uint8_t *g = oled_glyph_ch(*p);
      if (g != NULL)
      {
        if (x + 12u > SSD1315_W)
        {
          break;
        }
        fb_write_glyph5_2x(x, y0, g);
        x += 12u;
      }
    }
  }
}
#endif /* SSD1315_H >= 48u */

static const uint8_t *oled_glyph_ch(char c)
{
  if (c >= 'a' && c <= 'z')
  {
    c = (char)(c - 32);
  }
  if (c >= '0' && c <= '9')
  {
    return s_oled_font5[(unsigned)(c - '0')];
  }
  if (c >= 'A' && c <= 'Z')
  {
    return s_oled_cap[(unsigned)(c - 'A')];
  }
  if (c == ':')
  {
    return s_oled_font_colon;
  }
  if (c == '-')
  {
    return s_oled_font_minus;
  }
  if (c == '.')
  {
    return s_oled_font_dot;
  }
  return NULL;
}

/* Space = 4px; supports A–Z, 0–9, :. -, space */
static void oled_str5_page(unsigned page, unsigned x, const char *s)
{
  const char *p;

  for (p = s; *p != '\0'; p++)
  {
    if (*p == ' ')
    {
      x += 4u;
      continue;
    }
    {
      const uint8_t *g = oled_glyph_ch(*p);
      if (g != NULL)
      {
        fb_write_glyph5_1page(page, x, g);
        x += 6u;
      }
    }
    if (x >= SSD1315_W)
    {
      break;
    }
  }
}

#if SSD1315_H >= 48u
static void oled_draw_hud(void)
{
  DS3231_Time_t rt;
  HAL_StatusTypeDef trc;
  char tbuf[16];
  char odo_line[24];
  unsigned trx;
  const uint8_t *gmap;

  trc = DS3231_GetTime(&rt);
  if (trc == HAL_OK)
  {
    (void)sprintf(tbuf, "%02u:%02u:%02u",
                   (unsigned)(rt.hour % 24u), (unsigned)(rt.min % 60u), (unsigned)(rt.sec % 60u));
  }
  else
  {
    (void)sprintf(tbuf, "--:--:--");
  }

  (void)sprintf(odo_line, "PATH %.2fM", (double)NavOdom_GetTotalDistanceM());

  oled_str5_page(0u, 0u, tbuf);

  if (LineTrack_IsMapDetectLocked() == 0u)
  {
    gmap = oled_glyph_ch('?');
  }
  else if (LineTrack_GetMapMirror() > 0)
  {
    gmap = oled_glyph_ch('L');
  }
  else if (LineTrack_GetMapMirror() < 0)
  {
    gmap = oled_glyph_ch('R');
  }
  else
  {
    gmap = oled_glyph_ch('?');
  }
  trx = 114u;
  if (gmap != NULL)
  {
    fb_write_glyph5_1page(0u, trx, gmap);
  }

  /* 64px(8 page): 2x status 占 page2~3，PATH 放第5行 避免叠字。48/32 在下方分支。 */
#if SSD1315_H >= 64u
  oled_str5_page(4u, 0u, odo_line);
#elif SSD1315_H >= 48u
  oled_str5_page(4u, 0u, odo_line);
#else
  oled_str5_page(2u, 0u, odo_line);
#endif
}
#endif /* SSD1315_H >= 48u */

/*
 * Same idea as oled.txt OLED_Refresh(): page addressing 0x20/0x02, per page B0+i + column low/high,
 * then one 0x40 + 128 bytes (column-major page strip from s_fb). Avoids horizontal 0x21 drift on some modules.
 */
static HAL_StatusTypeDef ssd1315_flush(void)
{
  HAL_StatusTypeDef st;
  unsigned p;
  uint8_t cs;
  uint8_t pg_hdr[4];
  uint8_t line[129];

  (void)ssd1315_cmd1(0x2Eu);

  {
    static const uint8_t kPageAddr[] = { 0x20u, 0x02u };
    st = ssd1315_cmd_burst(kPageAddr, (uint16_t)sizeof(kPageAddr));
  }
  if (st != HAL_OK)
  {
    return st;
  }

  cs = (uint8_t)OLED_GDRAM_COL_START;
  for (p = 0u; p < SSD1315_NPAGE; p++)
  {
    pg_hdr[0] = SSD1315_CTRL_CMD;
    pg_hdr[1] = (uint8_t)(0xB0u | (uint8_t)p);
    pg_hdr[2] = (uint8_t)(0x00u | (cs & 0x0Fu));
    pg_hdr[3] = (uint8_t)(0x10u | ((cs >> 4) & 0x0Fu));
    st = ssd1315_tx(pg_hdr, 4u);
    if (st != HAL_OK)
    {
      return st;
    }
    line[0] = SSD1315_CTRL_DATA;
    (void)memcpy(&line[1], &s_fb[p * SSD1315_W], 128u);
    st = ssd1315_tx(line, 129u);
    if (st != HAL_OK)
    {
      return st;
    }
  }

  {
    static const uint8_t kHoriz[] = { 0x20u, 0x00u };
    st = ssd1315_cmd_burst(kHoriz, (uint16_t)sizeof(kHoriz));
  }
  return st;
}

void Oled_Init(void)
{
  HAL_StatusTypeDef st;
  HAL_StatusTypeDef stf;
  HAL_StatusTypeDef stc;

  s_inited = 0u;
  oled_pick_i2c_addr8();
  st = ssd1315_hw_init();
  if (st != HAL_OK)
  {
    printf("[OLED] FAIL hw_init HAL=%d addr8=0x%02X i2c_err=0x%lx (NACK/wiring PB6 SCL PB7 SDA/VCC/RES)\r\n",
           (int)st, (unsigned)s_ssd1315_i2c_addr8, (unsigned long)hi2c1.ErrorCode);
    return;
  }
  stc = ssd1315_clear_gddram_page_mode();
  if (stc != HAL_OK)
  {
    printf("[OLED] WARN clear_gddram HAL=%d (lower half may show noise)\r\n", (int)stc);
  }
  st = ssd1315_cmd1(0xAFu);
  if (st != HAL_OK)
  {
    printf("[OLED] FAIL display_on HAL=%d\r\n", (int)st);
    return;
  }
  fb_clear();
  stf = ssd1315_flush();
  if (stf != HAL_OK)
  {
    printf("[OLED] FAIL flush HAL=%d addr8=0x%02X i2c_err=0x%lx\r\n",
           (int)stf, (unsigned)s_ssd1315_i2c_addr8, (unsigned long)hi2c1.ErrorCode);
    return;
  }
  s_inited = 1u;
  printf("[OLED] OK %ux%u addr7=0x%02X I2C1_AF6\r\n",
         (unsigned)SSD1315_W, (unsigned)SSD1315_H, (unsigned)(s_ssd1315_i2c_addr8 >> 1));
}

void Oled_Tick(void)
{
  static uint32_t s_last_ms;
  static u8 s_radar_max_contrast;
  uint32_t now;
  line_track_oled_disp_t disp;

  if (!s_inited)
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - s_last_ms) < OLED_UI_PERIOD_MS)
  {
    return;
  }
  s_last_ms = now;

  disp = LineTrack_GetOledDisplayStatus();

  fb_clear();

  /* Radar phase: full-panel waterfall only (no HUD). Boost contrast in this mode. */
  if (disp == LINE_TRACK_OLED_RADAR)
  {
    uint8_t rbh = (uint8_t)SSD1315_H;
    if (rbh > (uint8_t)RADAR_VIS_BITMAP_H)
    {
      rbh = (uint8_t)RADAR_VIS_BITMAP_H;
    }
    if (s_radar_max_contrast == 0u)
    {
      (void)ssd1315_cmd1(0x81u);
      (void)ssd1315_cmd1(0xFFu);
      s_radar_max_contrast = 1u;
    }
    fb_blit_pages(RadarVis_GetBitmap(), RADAR_VIS_BITMAP_W, rbh, 0u, 0u);
    (void)ssd1315_flush();
    return;
  }
  if (s_radar_max_contrast != 0u)
  {
    (void)ssd1315_cmd1(0x81u);
    (void)ssd1315_cmd1(0xCFu);
    s_radar_max_contrast = 0u;
  }

  /* Status: 2x centered in band rows 17..30 (H>=48). 32px: 1x, manual x. */
  if (SSD1315_NPAGE >= 4u)
  {
    const char *st;

    switch (disp)
    {
    case LINE_TRACK_OLED_LOST:
      st = "LOST";
      break;
    case LINE_TRACK_OLED_STEER_L:
      st = "LEFT";
      break;
    case LINE_TRACK_OLED_STEER_R:
      st = "RIGHT";
      break;
    case LINE_TRACK_OLED_RADAR_PICK_L:
      st = "GO LEFT";
      break;
    case LINE_TRACK_OLED_RADAR_PICK_R:
      st = "GO RIGHT";
      break;
    case LINE_TRACK_OLED_TRACK:
    default:
      st = "FOLLOW";
      break;
    }
#if SSD1315_H >= 48u
    oled_str5_2x(17u, st);
#else
    switch (disp)
    {
    case LINE_TRACK_OLED_LOST:
      oled_str5_page(2u, 55u, "LOST");
      break;
    case LINE_TRACK_OLED_STEER_L:
      oled_str5_page(2u, 55u, "LEFT");
      break;
    case LINE_TRACK_OLED_STEER_R:
      oled_str5_page(2u, 49u, "RIGHT");
      break;
    case LINE_TRACK_OLED_RADAR_PICK_L:
      oled_str5_page(2u, 32u, "GO LEFT");
      break;
    case LINE_TRACK_OLED_RADAR_PICK_R:
      oled_str5_page(2u, 25u, "GO RIGHT");
      break;
    case LINE_TRACK_OLED_TRACK:
    default:
      oled_str5_page(2u, 46u, "FOLLOW");
      break;
    }
#endif
  }

#if SSD1315_H >= 48u
  oled_draw_hud();
#endif

  (void)ssd1315_flush();
}

#else /* !OLED_ENABLE */

void Oled_Init(void) { }
void Oled_Tick(void) { }

#endif /* OLED_ENABLE */
