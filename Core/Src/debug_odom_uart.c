 /**
 * @file debug_odom_uart.c
 *
 * Keil microlib：部分工程里 snprintf 未实现或恒失败，会表现为 n<=0 从不发送。
 * 周期输出改用 sprintf；上电自检用纯常量 HAL_UART_Transmit，不依赖任何 printf。
 */
#include "debug_odom_uart.h"

#if DEBUG_ODOM_UART_ENABLE

#include "DF_Communication.h"
#include "LineTracking.h"
#include "nav_odom.h"
#include "usart.h"
#include <stdio.h>
#if LT_RADAR_BRANCH_ENABLE
#include "radar_obstacle.h"
#include "radar_uart.h"
#endif

extern UART_HandleTypeDef huart3;

void DebugUsart3_BootPing(void)
{
	static const uint8_t k[] = "LORA_USART3_OK\r\n";
	(void)HAL_UART_Transmit(&huart3, k, (uint16_t)(sizeof(k) - 1U), 80);
}

void DebugOdomUart_Tick(void)
{
	static u32 s_last_ms;
	u32 t;
	int n;
	char buf[160];
	char tch;
	char c_radar;
#if LT_RADAR_BRANCH_ENABLE
	float lmf, rmf;
	long lm_c, rm_c;
	const long obs_dead_c = 15L;
#endif

	t = HAL_GetTick();
	if ((t - s_last_ms) < DEBUG_ODOM_UART_PERIOD_MS)
		return;
	s_last_ms = t;

	/* px/py：与 NavOdom_GetX/GetY 同源（默认 1:1），避免与累计路程分层重复换算 */

	switch (NavOdom_GetTurnHint()) {
	case NAV_TURN_LEFT:     tch = 'L'; break;
	case NAV_TURN_RIGHT:    tch = 'R'; break;
	case NAV_TURN_STRAIGHT: tch = '-'; break;
	default:                tch = '?'; break;
	}

	c_radar = '-';
#if LT_RADAR_BRANCH_ENABLE
	lm_c = 0;
	rm_c = 0;
	if (Radar_GetScanState() == RADAR_SCAN_DONE) {
		lmf = Radar_GetLeftDist();
		rmf = Radar_GetRightDist();
		lm_c = (long)(lmf * 100.0f + (lmf >= 0.0f ? 0.5f : -0.5f));
		rm_c = (long)(rmf * 100.0f + (rmf >= 0.0f ? 0.5f : -0.5f));
		if (lm_c + obs_dead_c < rm_c)
			c_radar = 'L';
		else if (rm_c + obs_dead_c < lm_c)
			c_radar = 'R';
	}
#endif

	/* m=累计弧长 m；px/py 为 m，与协议 N_Pos 同单位 */
	n = sprintf(buf,
		"OXY,m=%.4f,px=%.4f,py=%.4f,ms=%d,mlk=%u,%c%c\r\n",
		(double)NavOdom_GetTotalDistanceM(),
		(double)NavOdom_GetX(), (double)NavOdom_GetY(),
		(int)LineTrack_GetMapMirror(), (unsigned)LineTrack_IsMapDetectLocked(),
		tch, c_radar);
	if (n > 0 && n < (int)sizeof(buf))
		(void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 40);
}

#else /* !DEBUG_ODOM_UART_ENABLE */

void DebugOdomUart_Tick(void) { }
void DebugUsart3_BootPing(void) { }

#endif
