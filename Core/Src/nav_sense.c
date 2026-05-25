/**
 * @file nav_sense.c
 */
#include "nav_sense.h"
#include "ir_grayscale.h"

u8 NavSense_BuildPattern(void)
{
	u8 *x = IR_Data_number;
	return (u8)((x[0] << 7) | (x[1] << 6) | (x[2] << 5) | (x[3] << 4)
	       | (x[4] << 3) | (x[5] << 2) | (x[6] << 1) | x[7]);
}

u8 NavSense_BlackCount(u8 pattern)
{
	u8 i, cnt = 0;
	for (i = 0; i < 8; i++) {
		if (((pattern >> i) & 1u) == 0u)
			cnt++;
	}
	return cnt;
}

nav_junc_hint_t NavSense_JunctionHint(u8 pattern, u8 black_min)
{
	if (pattern == 0xFFu)
		return NAV_JUNC_LOST_ALL_WHITE;
	if (NavSense_BlackCount(pattern) >= black_min)
		return NAV_JUNC_LIKELY;
	return NAV_JUNC_NONE;
}

static u8 s_junc_streak;

void NavSense_ResetJunctionFilter(void)
{
	s_junc_streak = 0;
}

u8 NavSense_JunctionConfirmed(u8 black_min, u8 need_frames)
{
	u8 p = NavSense_BuildPattern();

	if (NavSense_JunctionHint(p, black_min) == NAV_JUNC_LIKELY) {
		if (s_junc_streak < 255u)
			s_junc_streak++;
	} else {
		s_junc_streak = 0;
	}
	return (s_junc_streak >= need_frames) ? 1u : 0u;
}
