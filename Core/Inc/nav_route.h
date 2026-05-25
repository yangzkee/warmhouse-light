/**
 * @file nav_route.h
 * @brief 赛道里程节点表 + 与 NavPath 线性拓扑同步（实车测试用）
 */
#ifndef NAV_ROUTE_H
#define NAV_ROUTE_H

#include "main.h"

typedef enum {
	NAV_ROUTE_LEFT = 1,
	NAV_ROUTE_RIGHT = 2,
	NAV_ROUTE_STRAIGHT = 3,
	NAV_ROUTE_IGNORE = 4,
	NAV_ROUTE_STOP = 5,
	NAV_ROUTE_RADAR_ARM = 6
} NavRouteAction_t;

typedef struct {
	/**
	 * 从发车/计时零点起，沿行驶轨迹的累计弧长（米），与 NavOdom_GetTotalDistanceM() 同单位。
	 * NAV_ROUTE_ENCODER520_ODOM_SCALE=1 时直接可比；换底盘或改积分后须按实车 OXY 的 m= 重标此列。
	 */
	float trigger_dist_m;
	NavRouteAction_t act;
	u16 guide_ms;
} NavRouteNodeDef_t;

void NavRoute_Init(void);
void NavRoute_Reset(void);

u8 NavRoute_GetCount(void);
u8 NavRoute_GetNextIndex(void);

u8 NavRoute_TryFire(float total_dist_m, float lead_m,
		    NavRouteAction_t *out_act, u16 *out_guide_ms, u8 *out_fired_index);

const NavRouteNodeDef_t *NavRoute_GetTable(void);

/** 与 NavOdom_GetTotalDistanceM() 同单位的触发阈值（520 profile 下可乘 NAV_ROUTE_ENCODER520_ODOM_SCALE，默认 1） */
float NavRoute_GetTriggerDistM(u8 idx);

/**
 * 物理米 → 与累计路程同单位的阈值。
 * 默认：底盘里程为米、NAV_ROUTE_ENCODER520_ODOM_SCALE=1 时恒等映射。
 */
float NavRoute_PhysicalMToOdomTotalM(float physical_m);

float NavRoute_GetRemainingToNextM(float total_dist_m, float lead_m);

#endif /* NAV_ROUTE_H */
