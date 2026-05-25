/**
 * @file nav_path.c
 */
#include "nav_path.h"
#include <string.h>
#include <math.h>

static nav_node_t s_nodes[NAV_PATH_MAX_NODES];
static nav_edge_t s_edges[NAV_PATH_MAX_EDGES];
static u8 s_edge_count;

static s16 s_bfs_dist[NAV_PATH_MAX_NODES];
static s16 s_bfs_next_edge[NAV_PATH_MAX_NODES]; /* 从该节点出发应走的边索引 */

void NavPath_Init(void)
{
	memset(s_nodes, 0, sizeof(s_nodes));
	memset(s_edges, 0, sizeof(s_edges));
	memset(s_bfs_dist, -1, sizeof(s_bfs_dist));
	memset(s_bfs_next_edge, -1, sizeof(s_bfs_next_edge));
	s_edge_count = 0;
}

void NavPath_SetNode(s16 id, float x_m, float y_m)
{
	if (id < 0 || id >= NAV_PATH_MAX_NODES)
		return;
	s_nodes[id].x_m = x_m;
	s_nodes[id].y_m = y_m;
	s_nodes[id].used = 1;
}

u8 NavPath_AddEdge(s16 from, s16 to, float length_m, u8 act_at_to)
{
	if (s_edge_count >= NAV_PATH_MAX_EDGES)
		return 0;
	if (from < 0 || to < 0 || from >= NAV_PATH_MAX_NODES || to >= NAV_PATH_MAX_NODES)
		return 0;
	s_edges[s_edge_count].from = from;
	s_edges[s_edge_count].to = to;
	s_edges[s_edge_count].length_m = length_m;
	s_edges[s_edge_count].act_at_to = act_at_to;
	s_edge_count++;
	return 1;
}

nav_node_t *NavPath_GetNode(s16 id)
{
	if (id < 0 || id >= NAV_PATH_MAX_NODES)
		return 0;
	return &s_nodes[id];
}

void NavPath_NodeVisit(s16 node_id)
{
	if (node_id >= 0 && node_id < NAV_PATH_MAX_NODES)
		s_nodes[node_id].visit_cnt++;
}

void NavPath_NodeResetVisit(s16 node_id)
{
	if (node_id >= 0 && node_id < NAV_PATH_MAX_NODES)
		s_nodes[node_id].visit_cnt = 0;
}

s16 NavPath_FindNearestNode(float x, float y, float gate_m)
{
	s16 best = -1;
	float best_d2 = gate_m * gate_m;
	s16 i;

	for (i = 0; i < NAV_PATH_MAX_NODES; i++) {
		float dx, dy, d2;
		if (!s_nodes[i].used)
			continue;
		dx = x - s_nodes[i].x_m;
		dy = y - s_nodes[i].y_m;
		d2 = dx * dx + dy * dy;
		if (d2 <= best_d2) {
			best_d2 = d2;
			best = i;
		}
	}
	return best;
}

void NavPath_BfsFromGoal(s16 goal_id)
{
	s16 q[NAV_PATH_MAX_NODES];
	u8 qh = 0, qt = 0;
	s16 i;

	for (i = 0; i < NAV_PATH_MAX_NODES; i++) {
		s_bfs_dist[i] = -1;
		s_bfs_next_edge[i] = -1;
	}

	if (goal_id < 0 || goal_id >= NAV_PATH_MAX_NODES)
		return;

	s_bfs_dist[goal_id] = 0;
	q[qt++] = (u8)goal_id;

	while (qh < qt) {
		s16 u = q[qh++];
		u8 e;

		for (e = 0; e < s_edge_count; e++) {
			/* 反向：若存在边 v->u，则从 u 扩散到 v */
			if (s_edges[e].to == u) {
				s16 v = s_edges[e].from;
				if (v >= 0 && v < NAV_PATH_MAX_NODES && s_bfs_dist[v] < 0) {
					s_bfs_dist[v] = s_bfs_dist[u] + 1;
					s_bfs_next_edge[v] = e;
					q[qt++] = (u8)v;
				}
			}
		}
	}
}

s16 NavPath_GetNextEdgeIndex(s16 node_id)
{
	if (node_id < 0 || node_id >= NAV_PATH_MAX_NODES)
		return -1;
	return s_bfs_next_edge[node_id];
}

const nav_edge_t *NavPath_GetEdge(s16 ei)
{
	if (ei < 0 || ei >= (s16)s_edge_count)
		return 0;
	return &s_edges[ei];
}
