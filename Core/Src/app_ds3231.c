#include "app_ds3231.h"
#include "i2c.h"
#include <stdio.h>

#define DS3231_I2C_ADDR           (0x68u << 1)
#define DS3231_REG_SEC            0x00u
#define DS3231_REG_CONTROL_STATUS 0x0Fu

static uint8_t bcd_to_dec(uint8_t bcd)
{
  return (uint8_t)(((bcd >> 4) * 10u) + (bcd & 0x0Fu));
}

static uint8_t dec_to_bcd(uint8_t dec)
{
  return (uint8_t)(((dec / 10u) << 4) | (dec % 10u));
}

static uint8_t weekday_zeller(uint16_t year, uint8_t month, uint8_t day)
{
  uint16_t y = year;
  uint8_t m = month;
  if (m < 3u)
  {
    m = (uint8_t)(m + 12u);
    y--;
  }
  {
    uint16_t w = (uint16_t)((day + 2u * m + (3u * (m + 1u)) / 5u + y + y / 4u - y / 100u + y / 400u) % 7u);
    return (uint8_t)(w + 1u);
  }
}

static HAL_StatusTypeDef read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2, DS3231_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100u);
}

static HAL_StatusTypeDef write_regs(uint8_t reg, const uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Write(&hi2c2, DS3231_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 100u);
}

static HAL_StatusTypeDef set_time_internal(const DS3231_Time_t *t)
{
  uint8_t raw[7];
  raw[0] = dec_to_bcd(t->sec % 60u);
  raw[1] = dec_to_bcd(t->min % 60u);
  raw[2] = dec_to_bcd(t->hour % 24u);
  raw[3] = dec_to_bcd((t->week >= 1u && t->week <= 7u) ? t->week : 1u);
  raw[4] = dec_to_bcd((t->day >= 1u && t->day <= 31u) ? t->day : 1u);
  raw[5] = dec_to_bcd((t->month >= 1u && t->month <= 12u) ? t->month : 1u);
  raw[6] = dec_to_bcd(t->year % 100u);
  return write_regs(DS3231_REG_SEC, raw, sizeof(raw));
}

HAL_StatusTypeDef DS3231_GetTime(DS3231_Time_t *t)
{
  uint8_t raw[7];
  HAL_StatusTypeDef st = read_regs(DS3231_REG_SEC, raw, sizeof(raw));
  if (st != HAL_OK)
  {
    return st;
  }
  t->sec = bcd_to_dec((uint8_t)(raw[0] & 0x7Fu));
  t->min = bcd_to_dec((uint8_t)(raw[1] & 0x7Fu));
  t->hour = bcd_to_dec((uint8_t)(raw[2] & 0x3Fu));
  t->week = bcd_to_dec((uint8_t)(raw[3] & 0x07u));
  t->day = bcd_to_dec((uint8_t)(raw[4] & 0x3Fu));
  t->month = bcd_to_dec((uint8_t)(raw[5] & 0x1Fu));
  t->year = bcd_to_dec(raw[6]);
  return HAL_OK;
}

static uint8_t month_from_date_str(const char *m)
{
  static const char *kMon[12] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  uint8_t i;
  for (i = 0u; i < 12u; i++)
  {
    if ((m[0] == kMon[i][0]) && (m[1] == kMon[i][1]) && (m[2] == kMon[i][2]))
    {
      return (uint8_t)(i + 1u);
    }
  }
  return 1u;
}

static void build_time_from_compile(DS3231_Time_t *t)
{
  const char *d = __DATE__;
  const char *tm = __TIME__;
  uint16_t year;
  t->month = month_from_date_str(d);
  t->day = (uint8_t)(((d[4] == ' ') ? 0 : (d[4] - '0')) * 10 + (d[5] - '0'));
  year = (uint16_t)((d[7] - '0') * 1000 + (d[8] - '0') * 100 + (d[9] - '0') * 10 + (d[10] - '0'));
  t->year = (uint8_t)(year % 100u);
  t->hour = (uint8_t)((tm[0] - '0') * 10 + (tm[1] - '0'));
  t->min = (uint8_t)((tm[3] - '0') * 10 + (tm[4] - '0'));
  t->sec = (uint8_t)((tm[6] - '0') * 10 + (tm[7] - '0'));
  t->week = weekday_zeller(year, t->month, t->day);
}

void DS3231_InitAndSyncToBuildTime(void)
{
  uint8_t status = 0u;
  DS3231_Time_t t;

  if (read_regs(DS3231_REG_CONTROL_STATUS, &status, 1u) != HAL_OK)
  {
    printf("[DS3231] read status failed\r\n");
    return;
  }

  build_time_from_compile(&t);
  if (set_time_internal(&t) != HAL_OK)
  {
    printf("[DS3231] set time failed\r\n");
    return;
  }

  status = (uint8_t)(status & (uint8_t)(~0x80u));
  (void)write_regs(DS3231_REG_CONTROL_STATUS, &status, 1u);
  printf("[DS3231] time set to build time: 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
         t.year, t.month, t.day, t.hour, t.min, t.sec);
}

void DS3231_PrintTimeOnce(void)
{
  DS3231_Time_t t;
  if (DS3231_GetTime(&t) == HAL_OK)
  {
    printf("[DS3231] 20%02u-%02u-%02u %02u:%02u:%02u W%u\r\n",
           t.year, t.month, t.day, t.hour, t.min, t.sec, t.week);
  }
  else
  {
    printf("[DS3231] read time failed\r\n");
  }
}
