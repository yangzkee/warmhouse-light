/**
 * @file nav_route.c
 */
#include "nav_route.h"
#include "nav_path.h"
#include "nav_odom.h"

/*
 * 路表 s_route_table[].trigger_dist_m：与 NavOdom_GetTotalDistanceM() **同一套「米」**——从同一零点起的累计弧长。
 * TryFire 用 total_dist_m（即 GetTotalDistanceM）与 (trigger − lead) 比较；SCALE 默认 1 时表内数字即实车应读到的路程。
 * 若某行 trigger_dist_m **不大于**上一已触发行的距离（路表暂未按里程递增重排），则**自动跳过**该行，不触发、不重算路径拓扑。
 *
 * 为何常要整表重标：
 *   历史上若曾对里程乘 10、或对路表乘 NAV_ROUTE_ENCODER520_ODOM_SCALE 等，表内米数与「当前」积分不一定同含义；
 *   现协议 N_Pos 与积分为米、SCALE=1 后，应以**实车**为准重填本表，不能假定旧数仍成立。
 *
 * 建议标定流程（正图/镜像场各做一套或按规则镜像）：
 *   1) 与正式发车一致上电、同一出发点；确保 NavRoute_Init/Reset 与里程零点与比赛一致。
 *   2) 开 DEBUG_ODOM_UART，看 OXY 行 **m=**（即 GetTotalDistanceM，米）。
 *   3) 车到各特征点（过线、8 字入口、雷达武装线等）时记下当前 **m**；将该值写入对应 idx 的 trigger_dist_m（可略提前以留 ROUTE_TRIGGER_LEAD_M 余量）。
 *   4) idx STOP：触发仍含 lead，见 LineTracking 中 ROUTE_TRIGGER_LEAD_M；表内 16m 等仅为占位，须按终点线实测改。
 *   5) guide_ms 等非距离列一般不必因「米」改动，除非策略变化。
 *
 * NAV_ROUTE_ENCODER520_ODOM_SCALE：仅当将来又出现「表内米」与「积分米」两套单位时启用；默认可 1。
 */
#ifndef NAV_ROUTE_ENCODER520_ODOM_SCALE
#define NAV_ROUTE_ENCODER520_ODOM_SCALE  1.0f
#endif

/* 下列距离为历史占位；SCALE=1 后请按上节流程用实车 m= 逐条替换 */
static const NavRouteNodeDef_t s_route_table[] = {
	/* idx 0~5：IGNORE=仅过线/提示，不注入 PID；路口策略见注释 */
	{ 3.00f,  NAV_ROUTE_IGNORE,     0 },
	{ 7.30f,  NAV_ROUTE_IGNORE,     0 }, /* 过线提示；须能继续到 10.1/14.45，勿在此 STOP */
	/* idx 3~5 */
	{ 9.60f,  NAV_ROUTE_IGNORE,     0 }, /* 标定点：仅打点，不注入引导 */
	{ 10.10f, NAV_ROUTE_IGNORE,     0 },
	{ 14.45f, NAV_ROUTE_IGNORE,     0 }, /* idx4：里程提示；终点 idx11=20.2m STOP（勿在此处 STOP 否则到不了后段） */
	{ 14.64f, NAV_ROUTE_IGNORE,     0 }, /* idx5 正图：左拐入8字圆弧（圆外弧）；武装8字；弱引导 LT_FIG8_* */
	{ 15.30f, NAV_ROUTE_IGNORE,     0 }, /* idx6：8字顶弧；武装 Fig8；正图左出迷宫依赖尾段光电+LineTracking */
	{ 16.20f, NAV_ROUTE_IGNORE,     0 }, /* 提示：近终点前路段 */
	{ 16.65f, NAV_ROUTE_RADAR_ARM,  0 }, /* 武装雷达：等 T 字全黑后扫描选边（须为 RADAR_ARM，仅用 IGNORE 不会进 LT_RADAR_WAIT_T） */
	{ 18.42f, NAV_ROUTE_IGNORE,       0u }, /* idx9：仅过点；交口循线 PID+光电，不注入 ROUTE_GUIDE */
	{ 19.15f, NAV_ROUTE_IGNORE,       0 }, /* idx10：与 LineTracking 盲走圆弧触发距一致（不注入导向） */
 	{ 20.10f, NAV_ROUTE_STOP,         0 }, /* idx11：终点停车 */
};

#define ROUTE_CNT ((u8)(sizeof(s_route_table) / sizeof(s_route_table[0])))

static float route_effective_trigger_m(u8 idx)
{
	float base;

	if (idx >= ROUTE_CNT)
		return 0.0f;
	base = s_route_table[idx].trigger_dist_m;
	if (NavOdom_GetProfile() == NAV_ODOM_PROFILE_ENCODER520)
		return base * NAV_ROUTE_ENCODER520_ODOM_SCALE;
	return base;
}

static u8 s_next_idx;
  /** 上一已成功 TryFire 的路点的有效触发距离（米）；用于跳过表中「小于等于该值」的后续占位行 */
static float s_last_fired_trig_m;

static u8 map_action_to_nav_edge(NavRouteAction_t a)
{
	switch (a) {
	case NAV_ROUTE_LEFT:     return (u8)NAV_ACT_LEFT;
	case NAV_ROUTE_RIGHT:    return (u8)NAV_ACT_RIGHT;
	case NAV_ROUTE_STRAIGHT: return (u8)NAV_ACT_STRAIGHT;
	case NAV_ROUTE_IGNORE:   return (u8)NAV_ACT_STRAIGHT;
	case NAV_ROUTE_STOP:     return (u8)NAV_ACT_STRAIGHT;
	case NAV_ROUTE_RADAR_ARM: return (u8)NAV_ACT_STRAIGHT;
	default:                 return (u8)NAV_ACT_STRAIGHT;
	}
}

static void build_linear_graph(void)
{
	u8 i;

	NavPath_Init();
	for (i = 0; i < ROUTE_CNT; i++) {
		u8 act = map_action_to_nav_edge(s_route_table[i].act);
		(void)NavPath_AddEdge((s16)i, (s16)(i + 1), 0.0f, act);
	}
	NavPath_BfsFromGoal((s16)ROUTE_CNT);
}

void NavRoute_Init(void)
{
	s_next_idx = 0;
	s_last_fired_trig_m = -1.0f;
	NavOdom_ResetTrajectory();
	build_linear_graph();
}

void NavRoute_Reset(void)
{
	s_next_idx = 0;
	s_last_fired_trig_m = -1.0f;
	NavOdom_ResetTrajectory();
}

u8 NavRoute_GetCount(void)
{
	return ROUTE_CNT;
}

u8 NavRoute_GetNextIndex(void)
{
	return s_next_idx;
}

const NavRouteNodeDef_t *NavRoute_GetTable(void)
{
	return s_route_table;
}

float NavRoute_GetTriggerDistM(u8 idx)
{
	return route_effective_trigger_m(idx);
}

float NavRoute_PhysicalMToOdomTotalM(float physical_m)
{
	if (NavOdom_GetProfile() == NAV_ODOM_PROFILE_ENCODER520)
		return physical_m * NAV_ROUTE_ENCODER520_ODOM_SCALE;
	return physical_m;
}

float NavRoute_GetRemainingToNextM(float total_dist_m, float lead_m)
{
	u8 idx;
	float lead_odom;
	float th;

	if (s_next_idx >= ROUTE_CNT)
		return 9999.0f;
	lead_odom = NavRoute_PhysicalMToOdomTotalM(lead_m);

	idx = s_next_idx;
	while (idx < ROUTE_CNT && route_effective_trigger_m(idx) <= s_last_fired_trig_m)
		idx++;
	if (idx >= ROUTE_CNT)
		return 9999.0f;
	th = route_effective_trigger_m(idx);
	return (th - lead_odom) - total_dist_m;
}

u8 NavRoute_TryFire(float total_dist_m, float lead_m,
		    NavRouteAction_t *out_act, u16 *out_guide_ms, u8 *out_fired_index)
{
	float trig;
	float lead_odom;
	float th;
	u8 fire_idx;

	lead_odom = NavRoute_PhysicalMToOdomTotalM(lead_m);

	while (s_next_idx < ROUTE_CNT) {
		th = route_effective_trigger_m(s_next_idx);
		if (th <= s_last_fired_trig_m) {
			s_next_idx++;
			continue;
		}
		trig = th - lead_odom;
		if (total_dist_m < trig)
			return 0;

		fire_idx = s_next_idx;
		if (out_act)
			*out_act = s_route_table[fire_idx].act;
		if (out_guide_ms)
			*out_guide_ms = s_route_table[fire_idx].guide_ms;
		if (out_fired_index)
			*out_fired_index = fire_idx;

		s_last_fired_trig_m = th;
		s_next_idx++;
		return 1;
	}
	return 0;
}
