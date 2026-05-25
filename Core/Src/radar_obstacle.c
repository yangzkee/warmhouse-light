/**
 * @file radar_obstacle.c
 * @brief 左右探头 + 雷达采样（自 check_radar，底盘旋转改为 DF sendrot 异步完成检测）
 */
#include "radar_obstacle.h"
#include "DF_Communication.h"
#include "radar_oled_vis.h"
#include "radar_uart.h"

static RadarScanState_t radar_scan_state = RADAR_SCAN_IDLE;
static float left_median_dist;
static float right_median_dist;
static u8 left_zero_count;
static u8 right_zero_count;
static float radar_samples[10];
static u8 sample_count;
static u32 delay_start_tick;
static u32 last_blink_tick;
static u32 s_sampling_t0;

static void bubble_sort(float arr[], int n)
{
	int i, j;

	for (i = 0; i < n - 1; i++) {
		for (j = 0; j < n - i - 1; j++) {
			if (arr[j] > arr[j + 1]) {
				float t = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = t;
			}
		}
	}
}

void Radar_StartScan(void)
{
	if (radar_scan_state == RADAR_SCAN_IDLE) {
		left_zero_count = 0;
		right_zero_count = 0;
		left_median_dist = 0.0f;
		right_median_dist = 0.0f;
		radar_scan_state = RADAR_SCAN_TURN_LEFT_45;
	}
}

void Radar_Reset(void)
{
	radar_scan_state = RADAR_SCAN_IDLE;
	left_zero_count = 0;
	right_zero_count = 0;
	left_median_dist = 0.0f;
	right_median_dist = 0.0f;
	RadarVis_Reset();
}

RadarScanState_t Radar_GetScanState(void)
{
	return radar_scan_state;
}

float Radar_GetLeftDist(void)
{
	return left_median_dist;
}

float Radar_GetRightDist(void)
{
	return right_median_dist;
}

u8 Radar_GetLeftZeroCount(void)
{
	return left_zero_count;
}

u8 Radar_GetRightZeroCount(void)
{
	return right_zero_count;
}

static float radar_median_from_buf(float *arr, u8 n)
{
	if (n == 0u)
		return RADAR_OBSTACLE_SAMPLE_FALLBACK_M;
	bubble_sort(arr, (int)n);
	if ((n & 1u) != 0u)
		return arr[n / 2u];
	return (arr[n / 2u - 1u] + arr[n / 2u]) * 0.5f;
}

/* 右采结束、回正前：把变量名与真实「左/右支路」对齐（见 LT_RADAR_SWAP_LR_MEDIAN） */
static void radar_swap_lr_medians_if_cfg(void)
{
#if LT_RADAR_SWAP_LR_MEDIAN
	{
		float tf;
		u8 tz;

		tf = left_median_dist;
		left_median_dist = right_median_dist;
		right_median_dist = tf;
		tz = left_zero_count;
		left_zero_count = right_zero_count;
		right_zero_count = tz;
	}
#endif
}

void Radar_UpdateObstacleScan(void)
{
	if (radar_scan_state == RADAR_SCAN_IDLE)
		return;

	if (radar_scan_state == RADAR_SCAN_TURN_LEFT_45) {
		(void)sendrot_AsyncBegin(0.0f, 0.0f, (-45.0f * LT_RADAR_SCAN_YAW_SIGN), 20.0f);
		delay_start_tick = HAL_GetTick();
		radar_scan_state = RADAR_SCAN_WAIT_LEFT_DONE;
	} else if (radar_scan_state == RADAR_SCAN_WAIT_LEFT_DONE) {
		if (DF_RotationAsyncTryConsumeDone(delay_start_tick)) {
			delay_start_tick = HAL_GetTick();
			radar_scan_state = RADAR_SCAN_DELAY_LEFT;
		} else if ((HAL_GetTick() - delay_start_tick) >= RADAR_OBSTACLE_ROT_TIMEOUT_MS) {
			DF_RotationAsyncForceClear();
			delay_start_tick = HAL_GetTick();
			radar_scan_state = RADAR_SCAN_DELAY_LEFT;
		}
	} else if (radar_scan_state == RADAR_SCAN_DELAY_LEFT) {
		if (HAL_GetTick() - delay_start_tick >= 500u) {
			radar_parsing_enabled = 1u;
			sample_count = 0;
			s_sampling_t0 = HAL_GetTick();
			radar_scan_state = RADAR_SCAN_SAMPLING_LEFT;
		}
	} else if (radar_scan_state == RADAR_SCAN_SAMPLING_LEFT) {
		if ((HAL_GetTick() - s_sampling_t0) >= RADAR_OBSTACLE_SAMPLE_TIMEOUT_MS) {
			left_median_dist = radar_median_from_buf(radar_samples, sample_count);
			radar_parsing_enabled = 0;
			radar_scan_state = RADAR_SCAN_TURN_RIGHT_90;
		} else if (radar_data_ready) {
			radar_data_ready = 0;
			if (radar_distance == 0.0f) {
				left_zero_count++;
				if (left_zero_count >= 10u) {
					left_median_dist = RADAR_OBSTACLE_ZEROS_IMPLY_BLOCKED_M;
					radar_parsing_enabled = 0;
					radar_scan_state = RADAR_SCAN_TURN_RIGHT_90;
				}
			} else if (sample_count < 10u) {
				radar_samples[sample_count++] = radar_distance;
				if (sample_count >= 10u) {
					left_median_dist = radar_median_from_buf(radar_samples, 10);
					radar_parsing_enabled = 0;
					radar_scan_state = RADAR_SCAN_TURN_RIGHT_90;
				}
			}
		}
	} else if (radar_scan_state == RADAR_SCAN_TURN_RIGHT_90) {
		(void)sendrot_AsyncBegin(0.0f, 0.0f, (90.0f * LT_RADAR_SCAN_YAW_SIGN), 20.0f);
		delay_start_tick = HAL_GetTick();
		radar_scan_state = RADAR_SCAN_WAIT_RIGHT_DONE;
	} else if (radar_scan_state == RADAR_SCAN_WAIT_RIGHT_DONE) {
		if (DF_RotationAsyncTryConsumeDone(delay_start_tick)) {
			delay_start_tick = HAL_GetTick();
			radar_scan_state = RADAR_SCAN_DELAY_RIGHT;
		} else if ((HAL_GetTick() - delay_start_tick) >= RADAR_OBSTACLE_ROT_TIMEOUT_MS) {
			DF_RotationAsyncForceClear();
			delay_start_tick = HAL_GetTick();
			radar_scan_state = RADAR_SCAN_DELAY_RIGHT;
		}
	} else if (radar_scan_state == RADAR_SCAN_DELAY_RIGHT) {
		if (HAL_GetTick() - delay_start_tick >= 500u) {
			radar_parsing_enabled = 1u;
			sample_count = 0;
			s_sampling_t0 = HAL_GetTick();
			radar_scan_state = RADAR_SCAN_SAMPLING_RIGHT;
		}
	} else if (radar_scan_state == RADAR_SCAN_SAMPLING_RIGHT) {
		if ((HAL_GetTick() - s_sampling_t0) >= RADAR_OBSTACLE_SAMPLE_TIMEOUT_MS) {
			right_median_dist = radar_median_from_buf(radar_samples, sample_count);
			radar_parsing_enabled = 0;
			radar_swap_lr_medians_if_cfg();
			radar_scan_state = RADAR_SCAN_TURN_BACK_45;
		} else if (radar_data_ready) {
			radar_data_ready = 0;
			if (radar_distance == 0.0f) {
				right_zero_count++;
				if (right_zero_count >= 10u) {
					right_median_dist = RADAR_OBSTACLE_ZEROS_IMPLY_BLOCKED_M;
					radar_parsing_enabled = 0;
					radar_swap_lr_medians_if_cfg();
					radar_scan_state = RADAR_SCAN_TURN_BACK_45;
				}
			} else if (sample_count < 10u) {
				radar_samples[sample_count++] = radar_distance;
				if (sample_count >= 10u) {
					right_median_dist = radar_median_from_buf(radar_samples, 10);
					radar_parsing_enabled = 0;
					radar_swap_lr_medians_if_cfg();
					radar_scan_state = RADAR_SCAN_TURN_BACK_45;
				}
			}
		}
	} else if (radar_scan_state == RADAR_SCAN_TURN_BACK_45) {
		(void)sendrot_AsyncBegin(0.0f, 0.0f, (-45.0f * LT_RADAR_SCAN_YAW_SIGN), 20.0f);
		delay_start_tick = HAL_GetTick();
		radar_scan_state = RADAR_SCAN_COMPARE;
	} else if (radar_scan_state == RADAR_SCAN_COMPARE) {
		if (DF_RotationAsyncTryConsumeDone(delay_start_tick))
			radar_scan_state = RADAR_SCAN_DONE;
		else if ((HAL_GetTick() - delay_start_tick) >= RADAR_OBSTACLE_ROT_TIMEOUT_MS) {
			DF_RotationAsyncForceClear();
			radar_scan_state = RADAR_SCAN_DONE;
		}
	} else if (radar_scan_state == RADAR_SCAN_DONE) {
		if (left_median_dist >= right_median_dist) {
			if (HAL_GetTick() - last_blink_tick >= 1000u)
				last_blink_tick = HAL_GetTick();
		}
	}
}
