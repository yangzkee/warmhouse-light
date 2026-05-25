/**
 * @file radar_uart.c
 */
#include "radar_uart.h"
#include "radar_oled_vis.h"

float radar_distance;
float radar_angle;
volatile u8 radar_data_ready;
volatile u8 radar_parsing_enabled;

static u8 s_rx_buf[RADAR_UART_FRAME_LEN];
static u8 s_rx_idx;

void RadarUart_OnRxByte(u8 b)
{
	if (!radar_parsing_enabled) {
		s_rx_idx = 0;
		return;
	}

	s_rx_buf[s_rx_idx++] = b;

	if (s_rx_idx == 1u && s_rx_buf[0] != 0xAAu) {
		s_rx_idx = 0;
		return;
	}
	if (s_rx_idx == 2u && s_rx_buf[1] != 0x55u) {
		s_rx_idx = 0;
		return;
	}
	if (s_rx_idx < RADAR_UART_FRAME_LEN)
		return;

	if (s_rx_buf[2] == 0x06u && s_rx_buf[3] == 0xA2u) {
		u8 sum = 0;
		u8 i;

		for (i = 0; i < 12u; i++)
			sum = (u8)(sum + s_rx_buf[i]);
		if (sum == s_rx_buf[12]) {
			u16 dist_raw = (u16)(s_rx_buf[6] | ((u16)s_rx_buf[7] << 8));
			u16 angle_raw = (u16)(s_rx_buf[8] | ((u16)s_rx_buf[9] << 8));

			radar_distance = dist_raw * 0.01f;
			radar_angle = (float)((s16)angle_raw) * 0.01f;
			radar_data_ready = 1u;
			RadarVis_EnqueueFrame_ISR(radar_distance, radar_angle);
		}
	}
	s_rx_idx = 0;
}
