/**
 * @file odom_telemetry.c
 */
#include "odom_telemetry.h"
#include "DF_Communication.h"
#include "LineTracking.h"
#include "nav_odom.h"
#include "usart.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

void OdomTelemetry_Tick(float total_dist_m)
{
#if ODOM_TELEMETRY_ENABLE
	static u32 s_last_ms;
	u32 t = HAL_GetTick();
	char buf[220];
	int n;

	if ((t - s_last_ms) < ODOM_TELEMETRY_PERIOD_MS)
		return;
	s_last_ms = t;

	/* 与 Parse_OdomData1 一致；不输出 gyro_z；末四列与 OXY 同义的地图判别（ms/mlk/xcm/odl） */
	n = sprintf(buf,
		"ODOM,%.3f,%.4f,%.4f,%.4f,%.4f,%u,%.3f,%.4f,%.4f,%d,%u,%ld,%u\r\n",
		(double)g_odom.yaw,
		(double)g_odom.pos_x,
		(double)g_odom.pos_y,
		(double)g_odom.vel_x,
		(double)g_odom.vel_y,
		(unsigned)g_odom.valid,
		(double)total_dist_m,
		(double)NavOdom_GetX(),
		(double)NavOdom_GetY(),
		(int)LineTrack_GetMapMirror(),
		(unsigned)LineTrack_IsMapDetectLocked(),
		(long)LineTrack_GetMapDetectXcm(),
		(unsigned)LineTrack_IsMapOdomLatched());
	if (n > 0 && n < (int)sizeof(buf))
		(void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 30);
#else
	(void)total_dist_m;
#endif
}
