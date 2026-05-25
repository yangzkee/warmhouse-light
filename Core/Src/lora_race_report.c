/**
 * @file lora_race_report.c
 * At 9.60m / 20.10m: one UTF-8 line on USART3 (LoRa).
 *
 * Wire format (ASCII + pipes + ISO-like time; only 队名 is CJK, \\x-escaped in source for Keil AC5):
 *   LORA|01|张邹建宇|2026-04-23T15:33:50|R+0m45s
 *   字段: 头|编号|队名|DS3231 当前时间(CLK)|比赛用时 R+{分}m{秒}s (上电起算)
 * 全角标点易在 VOFA+「Abc/JustFloat」里被错切字节；故分隔符用 | 与 ASCII，避免 [?:ef] 类乱码。
 * 接收端选 UTF-8；纯文本模式比 Abc 更适合本行（若用 Abc 请仅作 Raw 看字符串）。
 */
#include "lora_race_report.h"
#include "app_ds3231.h"
#include "DF_Communication.h"
#include "usart.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

#define LORA_REPORT_DIST_M_960  9.60f
#define LORA_REPORT_DIST_M_2010 20.10f

static const char s_team_id[] = "01";
/* "张邹建宇" UTF-8 */
static const char s_team_name[] = "\xe5\xbc\xa0\xe9\x82\xb9\xe5\xbb\xba\xe5\xae\x87";

static void send_one_line(void)
{
	DS3231_Time_t t;
	int n;
	char buf[220];
	u32 t_ms;
	u32 el_s;
	u32 el_min;
	u32 el_sec;
	u16 y_full;
	uint8_t y, mon, d, h, mi, s;

	t_ms = HAL_GetTick();
	el_s = t_ms / 1000U;
	el_min = el_s / 60U;
	el_sec = el_s % 60U;

	if (DS3231_GetTime(&t) == HAL_OK) {
		y = t.year;
		mon = t.month;
		d = t.day;
		h = t.hour;
		mi = t.min;
		s = t.sec;
		y_full = (u16)(2000u + (u16)y);
		n = sprintf(buf,
			"LORA|%s|%s|%u-%02u-%02uT%02u:%02u:%02u|R+%um%us\r\n",
			s_team_id, s_team_name, (unsigned)y_full, (unsigned)mon, (unsigned)d,
			(unsigned)h, (unsigned)mi, (unsigned)s,
			(unsigned)el_min, (unsigned)el_sec);
	} else {
		n = sprintf(buf,
			"LORA|%s|%s|CLK--|R+%um%us\r\n",
			s_team_id, s_team_name, (unsigned)el_min, (unsigned)el_sec);
	}
	if (n > 0 && n < (int)sizeof(buf))
		(void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 80);
}

void LoraRaceReport_Try(float total_dist_m)
{
	static u8 s_done_960;
	static u8 s_done_2010;

	if (!Odom_IsValid())
		return;

	if (s_done_960 == 0U && total_dist_m >= LORA_REPORT_DIST_M_960) {
		s_done_960 = 1u;
		send_one_line();
	}
	if (s_done_2010 == 0U && total_dist_m >= LORA_REPORT_DIST_M_2010) {
		s_done_2010 = 1u;
		send_one_line();
	}
}
