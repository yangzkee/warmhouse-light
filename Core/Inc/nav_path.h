/**
 * @file nav_path.h
 * @brief 决策层：拓扑边表 + 从终点反向 BFS + 访问计数防绕圈（骨架）
 *
 * 使用方式（两阶段）：
 * 1) 探索阶段：在路口根据 ODOM 近似位置建节点/边（或手动填表）
 * 2) 行驶阶段：NavPath_BfsFromGoal(end_id) 后，在路口用 NavPath_PickActionFromEdge()
 *
 * 注意：F103 RAM 有限，MAX_NODES / MAX_EDGES 保持较小；若赛道固定，可改为“纯静态表”。
 */
#ifndef NAV_PATH_H
#define NAV_PATH_H

#include "main.h"

/* 须 ≥ 路线顶点数+1：nav_route 线性图为 0..ROUTE_CNT（含终点顶点），16 个路点需顶点 0..16 共 17 个 */
#define NAV_PATH_MAX_NODES 24
#define NAV_PATH_MAX_EDGES 40

/* 边上记录的“在路口应执行”的相对动作（相对车头） */
typedef enum {
	NAV_ACT_STRAIGHT = 0,
	NAV_ACT_RIGHT    = 1,
	NAV_ACT_BACK     = 2, /* 预留 */
	NAV_ACT_LEFT     = 3
} nav_act_rel_t;

typedef struct {
	s16 from;
	s16 to;
	float length_m; /* 可选：用于加权最短路，当前 BFS 仅跳数 */
	u8 act_at_to;   /* nav_act_rel_t：到达 to 前在路口执行的动作 */
} nav_edge_t;

typedef struct {
	float x_m;
	float y_m;
	u8 visit_cnt; /* 防绕圈：同一节点多次访问可改变策略 */
	u8 used;      /* 1=该槽位已录入坐标 */
} nav_node_t;

void NavPath_Init(void);
void NavPath_SetNode(s16 id, float x_m, float y_m);

/* 手动添加边（适合已知地图一次性录入） */
u8 NavPath_AddEdge(s16 from, s16 to, float length_m, u8 act_at_to);

/* 从终点反向 BFS，得到每个节点到终点的“下一步边”索引（-1 表示不可达） */
void NavPath_BfsFromGoal(s16 goal_id);

/* 当前在 node_id，返回应沿哪条边离开（bfs 树中的边），否则 -1 */
s16 NavPath_GetNextEdgeIndex(s16 node_id);

const nav_edge_t *NavPath_GetEdge(s16 ei);

void NavPath_NodeVisit(s16 node_id);
void NavPath_NodeResetVisit(s16 node_id);
nav_node_t *NavPath_GetNode(s16 id);

/* 占位：用坐标匹配最近节点（±2cm 需按实测调） */
s16 NavPath_FindNearestNode(float x, float y, float gate_m);

#endif /* NAV_PATH_H */
