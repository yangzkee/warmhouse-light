/**
 * @file nav_odom.c
 * @brief pos 默认按 NAV_ODOM_LINEAR_SCALE_* 缩放后与协议一致；累计路程与相邻帧 (x,y) 同单位。
 */
#include "nav_odom.h"
#include "DF_Communication.h"
#include <math.h>

static float s_linear_scale = NAV_ODOM_LINEAR_SCALE_STEPPER;
static float s_max_step_m = NAV_ODOM_MAX_STEP_M_STEPPER;
static NavOdomProfile_t s_profile = NAV_ODOM_PROFILE_STEPPER;

static float s_total_dist_m;
static float s_last_x;
static float s_last_y;
static u8 s_inited;
static float s_seg_start_dist; /* 累计路程上的一段起点 */
static u32 s_last_odom_tick;     /* 上次已参与积分的 g_odom.update_tick */
static float s_prev_yaw_deg;     /* 上一 ODOM 帧 yaw，用于 Δyaw */
static float s_last_yaw_delta_deg;

static float unwrap_delta_deg(float a_deg, float b_deg)
{
	float d = b_deg - a_deg;
	while (d > 180.0f)
		d -= 360.0f;
	while (d < -180.0f)
		d += 360.0f;
	return d;
}

void NavOdom_ResetTrajectory(void)
{
	s_total_dist_m = 0.0f;
	s_last_x = 0.0f;
	s_last_y = 0.0f;
	s_inited = 0;
	s_seg_start_dist = 0.0f;
	s_last_odom_tick = 0U;
	s_prev_yaw_deg = 0.0f;
	s_last_yaw_delta_deg = 0.0f;
}

void NavOdom_SetProfile(NavOdomProfile_t profile)
{
	switch (profile) {
	case NAV_ODOM_PROFILE_ENCODER520:
		s_linear_scale = NAV_ODOM_LINEAR_SCALE_ENCODER520;
		s_max_step_m = NAV_ODOM_MAX_STEP_M_ENCODER520;
		break;
	default:
		profile = NAV_ODOM_PROFILE_STEPPER;
		/* fallthrough */
	case NAV_ODOM_PROFILE_STEPPER:
		s_linear_scale = NAV_ODOM_LINEAR_SCALE_STEPPER;
		s_max_step_m = NAV_ODOM_MAX_STEP_M_STEPPER;
		break;
	}
	s_profile = profile;
	NavOdom_ResetTrajectory();
}

NavOdomProfile_t NavOdom_GetProfile(void)
{
	return s_profile;
}

float NavOdom_GetLinearScale(void)
{
	return s_linear_scale;
}

void NavOdom_UpdateStep(void)
{
	if (!Odom_IsValid())
		return;

	/* ODOM 约 10Hz，主循环 250Hz：仅在新解析帧上积分，避免对同一 (x,y) 反复 sqrt */
	if (g_odom.update_tick == s_last_odom_tick)
		return;

	if (!s_inited) {
		s_last_x = g_odom.pos_x * s_linear_scale;
		s_last_y = g_odom.pos_y * s_linear_scale;
		s_prev_yaw_deg = g_odom.yaw;
		s_last_yaw_delta_deg = 0.0f;
		s_inited = 1;
		s_last_odom_tick = g_odom.update_tick;
		return;
	}

	{
		float x = g_odom.pos_x * s_linear_scale;
		float y = g_odom.pos_y * s_linear_scale;
		float dx = x - s_last_x;
		float dy = y - s_last_y;
		float step = sqrtf(dx * dx + dy * dy);

		if (step <= s_max_step_m)
			s_total_dist_m += step;

		s_last_yaw_delta_deg = unwrap_delta_deg(s_prev_yaw_deg, g_odom.yaw);
		s_prev_yaw_deg = g_odom.yaw;

		s_last_x = x;
		s_last_y = y;
		s_last_odom_tick = g_odom.update_tick;
	}
}

float NavOdom_GetTotalDistanceM(void)
{
	return s_total_dist_m;
}

float NavOdom_GetX(void)
{
	return g_odom.pos_x * s_linear_scale;
}

float NavOdom_GetY(void)
{
	return g_odom.pos_y * s_linear_scale;
}

float NavOdom_GetYawDeg(void)
{
	return Odom_GetYaw();
}

float NavOdom_GetLastYawDeltaDeg(void)
{
	if (!Odom_IsValid() || !s_inited)
		return 0.0f;
	return s_last_yaw_delta_deg;
}

NavOdomTurnHint_t NavOdom_GetTurnHint(void)
{
	float d;

	if (!Odom_IsValid() || !s_inited)
		return NAV_TURN_UNKNOWN;

	d = s_last_yaw_delta_deg * NAV_ODOM_YAW_LEFT_SIGN;
	if (d > NAV_ODOM_YAW_DELTA_DEADZONE_DEG)
		return NAV_TURN_LEFT;
	if (d < -NAV_ODOM_YAW_DELTA_DEADZONE_DEG)
		return NAV_TURN_RIGHT;
	return NAV_TURN_STRAIGHT;
}

void NavOdom_MarkSegmentStart(void)
{
	s_seg_start_dist = s_total_dist_m;
}

float NavOdom_GetDeltaSinceLastMarkM(void)
{
	return s_total_dist_m - s_seg_start_dist;
}
