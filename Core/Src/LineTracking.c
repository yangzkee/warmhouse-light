 /**
 * @file LineTracking.c
 * @brief 8路灰度巡线逻辑，PID+左偏/右偏模式，吸附式循迹
 *
 * 分层：PID 巡线为主；几何上牢控 L4/L5 压黑，直道(345/456)强化中路，直角(12345/45678)抬 err，全黑 T 字配合里程。
 * 另：nav_odom + nav_route + 拐角状态机 + 迷宫锁 + 流程图（横+竖→巡线；横在竖无→直角；丢线→等预设或快自转）
 *
 * 传感器：L1最左、L8最右，0=黑线、1=白底；pattern=(L1<<7)|…|(L8<<0)，与 nav_sense / IR_Data_number[0..7] 一致。
 *
 * sendVel(V_x,V_y,V_z) 与底盘约定（须与此处控制量一致）：
 * - V_x>0：车体向右横移，车头朝向不变；V_y>0：前进；V_z>0：绕竖轴顺时针转角(度，×SCALE_FACTOR_VELOCITY 下发)。
 * - 左转=逆时针=V_z<0；右转=顺时针=V_z>0。
 * - 本文件 err：线在左侧 err>0。参考实现 vz=-pid_out×…；LINE_TRACK_INVERT_STEER_CMD 时再取反以适配底盘。
 *
 * PID吸附原理：
 * - 居中时err≈0，PID输出≈0，Vz≈0，直行
 * - 偏离时err≠0，PID输出≠0，Vz使车头转向黑线，自动拉回
 * - Vz不是默认90°，而是由PID实时计算，限幅在±VZ_ANGLE_MAX内
 *
 * PC13 LED：持续收到 ODOM（update_tick 新鲜）时常亮；曾解析但断流时快双闪；未解析有串口慢闪。
 */
#include "LineTracking.h"
#include "ir_grayscale.h"
#include "DF_Communication.h"
#include "nav_odom.h"
#include "nav_sense.h"
#include "nav_route.h"
#include "odom_telemetry.h"
#include "debug_odom_uart.h"
#include "lora_race_report.h"
#include <math.h>
#include <stdio.h>
#if LT_RADAR_BRANCH_ENABLE
#include "radar_obstacle.h"
#include "radar_uart.h"
#endif

/*
 * 巡线核心（Track_Err / PID / 丢线 LostChooseVz 等）与 XUNXIANGIT 工程对齐；
 * nav_route.c 路径表、地图判定、迷宫光电状态机仍由本文件调用当前工程数据。
 * 若底盘 Vz 与参考工程相反：在 LineTracking.h 设 LINE_TRACK_INVERT_STEER_CMD=1。
 */
static float LineTrack_SteerSignApply(float vz_xun)
{
#if LINE_TRACK_INVERT_STEER_CMD
	return -vz_xun;
#else
	return vz_xun;
#endif
}

static void LineTrack_SendVelFull(float vx, float vy, float vz);

extern UART_HandleTypeDef huart3;

/* 迷宫段里程：累计 Δyaw 与路表 L/R 校核；仅 LT_MAZE_YAW_USART3_LOG=1 时经 USART3 打 MAZE 行。 */
#ifndef LT_MAZE_YAW_USART3_LOG
#define LT_MAZE_YAW_USART3_LOG  0
#endif
#ifndef LT_MAZE_YAW_CHK_ENABLE
#define LT_MAZE_YAW_CHK_ENABLE  1
#endif
#ifndef LT_MAZE_CHK_IDX_LO
#define LT_MAZE_CHK_IDX_LO  255u /* 无迷宫里程点时不对 ODOM 节点做迷宫 yaw 校验 */
#endif
#ifndef LT_MAZE_CHK_IDX_HI
#define LT_MAZE_CHK_IDX_HI  255u
#endif
#ifndef MAZE_CHK_WINDOW_MS
#define MAZE_CHK_WINDOW_MS  850u /* ODOM ~10Hz，约 8～9 帧；可调大更易过判 */
#endif
#ifndef MAZE_CHK_PASS_DEG
#define MAZE_CHK_PASS_DEG  3.5f /* 窗口内净转角超过此视为与 L/R 一致 */
#endif
#ifndef MAZE_CHK_STRAIGHT_MAX_DEG
#define MAZE_CHK_STRAIGHT_MAX_DEG  7.0f
#endif

#if LT_MAZE_YAW_CHK_ENABLE
static u8 s_maze_chk_active;
#if LT_MAZE_YAW_USART3_LOG
static u8 s_maze_chk_idx;
#endif
static NavRouteAction_t s_maze_chk_exp;
static u32 s_maze_chk_deadline;
static u32 s_maze_chk_odom_tick;
static float s_maze_chk_sum_deg;

static void LineTrack_MazeYawChk_Finalize(const char *tail)
{
#if LT_MAZE_YAW_USART3_LOG
	{
		char buf[96];
		int n;
		char expc = '?';
		long sc;

		if (s_maze_chk_exp == NAV_ROUTE_LEFT)
			expc = 'L';
		else if (s_maze_chk_exp == NAV_ROUTE_RIGHT)
			expc = 'R';
		else if (s_maze_chk_exp == NAV_ROUTE_STRAIGHT)
			expc = 'S';
		else if (s_maze_chk_exp == NAV_ROUTE_IGNORE)
			expc = 'I';
		else if (s_maze_chk_exp == NAV_ROUTE_STOP)
			expc = 'T';
		else if (s_maze_chk_exp == NAV_ROUTE_RADAR_ARM)
			expc = 'D';

		sc = (long)(s_maze_chk_sum_deg * 100.0f
			    + (s_maze_chk_sum_deg >= 0.0f ? 0.5f : -0.5f));
		n = sprintf(buf, "MAZE,idx=%u,exp=%c,sum_cdeg=%+ld,%s\r\n",
			(unsigned)(s_maze_chk_idx + 1u), expc, sc, tail);
		if (n > 0 && n < (int)sizeof(buf))
			(void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 30);
	}
#else
	(void)tail;
#endif
	s_maze_chk_active = 0;
}

static void LineTrack_MazeYawChk_EvalClose(void)
{
	int ok;
	float sum;

	if (!Odom_IsValid()) {
		LineTrack_MazeYawChk_Finalize("FAIL,no_odom");
		return;
	}
	sum = s_maze_chk_sum_deg;
	ok = 0;
	if (s_maze_chk_exp == NAV_ROUTE_LEFT)
		ok = (sum >= MAZE_CHK_PASS_DEG);
	else if (s_maze_chk_exp == NAV_ROUTE_RIGHT)
		ok = (sum <= -MAZE_CHK_PASS_DEG);
	else if (s_maze_chk_exp == NAV_ROUTE_STRAIGHT) {
		if (sum >= -MAZE_CHK_STRAIGHT_MAX_DEG && sum <= MAZE_CHK_STRAIGHT_MAX_DEG)
			ok = 1;
	} else if (s_maze_chk_exp == NAV_ROUTE_STOP)
		ok = 1;
	else if (s_maze_chk_exp == NAV_ROUTE_RADAR_ARM)
		ok = 1;
	else
		ok = 1; /* IGNORE：不判 */

	LineTrack_MazeYawChk_Finalize(ok ? "OK" : "FAIL");
}

static void LineTrack_MazeYawChk_Process(void)
{
	u32 now;

	if (!s_maze_chk_active)
		return;
	now = HAL_GetTick();
	if (Odom_IsValid() && g_odom.update_tick != s_maze_chk_odom_tick) {
		s_maze_chk_odom_tick = g_odom.update_tick;
		s_maze_chk_sum_deg += NavOdom_GetLastYawDeltaDeg() * NAV_ODOM_YAW_LEFT_SIGN;
	}
	if ((s32)(now - s_maze_chk_deadline) >= 0)
		LineTrack_MazeYawChk_EvalClose();
}

static void LineTrack_MazeYawChk_Arm(u8 fired_idx, NavRouteAction_t act)
{
	if (fired_idx < LT_MAZE_CHK_IDX_LO || fired_idx > LT_MAZE_CHK_IDX_HI)
		return;
	if (s_maze_chk_active) {
#if LT_MAZE_YAW_USART3_LOG
		{
			char buf[48];
			int n;

			n = sprintf(buf, "MAZE,idx=%u,OVL\r\n", (unsigned)(s_maze_chk_idx + 1u));
			if (n > 0 && n < (int)sizeof(buf))
				(void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 25);
		}
#endif
	}
	s_maze_chk_active = 1;
#if LT_MAZE_YAW_USART3_LOG
	s_maze_chk_idx = fired_idx;
#endif
	s_maze_chk_exp = act;
	s_maze_chk_sum_deg = 0.0f;
	s_maze_chk_deadline = HAL_GetTick() + MAZE_CHK_WINDOW_MS;
	if (Odom_IsValid()) {
		s_maze_chk_odom_tick = g_odom.update_tick;
	} else {
		s_maze_chk_odom_tick = 0u;
		LineTrack_MazeYawChk_Finalize("SKIP,no_odom");
	}
}
#else
static void LineTrack_MazeYawChk_Process(void) { }
static void LineTrack_MazeYawChk_Arm(u8 fired_idx, NavRouteAction_t act)
{
	(void)fired_idx;
	(void)act;
}
#endif /* LT_MAZE_YAW_CHK_ENABLE */

/* ==================== 参数配置 ==================== */
#ifndef LINE_TRACK_VY_BASE
#define LINE_TRACK_VY_BASE  14.0f /* 巡线基准 vy；可工程里 #define 覆盖 */
#endif
#define LINE_SPEED       LINE_TRACK_VY_BASE
#define VZ_ANGLE_MAX     34.0f   /* 急弯/S 末端接直道略抬上限，防修正顶死 */
#define VZ_ANGLE_MIN     0.22f   /* 输出死区略收，小舵量也能跟上 S 弯 */

#define PID_KP           0.66f   /* 直道基准；弯里由 LT_PID_*_CURVE_SCALE 放大 P/D */
#define PID_KI           0.0029f
#define PID_KD           3.55f
#define PID_INTEGRAL_MAX 76.0f   /* 无 yaw 时略收，防弯中积分顶满 */
/* 弯越大(s_curve_blend→1)略增 Kp/Kd：参考巡线「弯区加强横向」常见做法；直道 blend≈0 等价原参数 */
#ifndef LT_PID_KP_CURVE_SCALE
#define LT_PID_KP_CURVE_SCALE  0.24f /* 弯/S 末端横向响应更快 */
#endif
#ifndef LT_PID_KD_CURVE_SCALE
#define LT_PID_KD_CURVE_SCALE  0.18f /* 偏差变化快时 D 跟上，抑冲出 */
#endif
#define PID_DT           (LINE_TRACK_LOOP_MS / 1000.0f)
#define D_FILTER_ALPHA   0.09f   /* 略加快 D 跟随；仍低通抑抖 */

#define LT_ERR_SOFT_DEADZONE 0.82f /* 小偏差少打舵，抑直道蛇形 */
#define LT_ERR_SOFT_SCALE    0.38f
#define LT_ARC_ERR_MIN       0.85f  /* 更早进入圆弧增强 */
#define LT_ARC_ERR_MAX       16.0f
#define LT_ARC_ERR_BOOST     1.40f /* 圆弧/连续弯横向更贴轨迹 */
#define LT_LARGE_ERR_BOOST   1.18f /* |err| 很大时再略抬，减少甩出赛道 */
#define LT_VZ_SLEW_MAX       3.35f /* 弯里再加 LT_VZ_SLEW_CURVE_ADD；直道会减 LT_VZ_SLEW_STRAIGHT_SUB */
#ifndef LT_VZ_SLEW_STRAIGHT_SUB
#define LT_VZ_SLEW_STRAIGHT_SUB  0.98f /* 直道进一步限 vz 变化率，抑蛇形 */
#endif
/* 直道：略降 P、略抬 D + D 低通更钝 + 航向角速度低通（与 s_straight_blend 联动） */
#ifndef LT_STRAIGHT_PID_KP_REDUCE
#define LT_STRAIGHT_PID_KP_REDUCE  0.22f /* 直道进一步减 P，减晃 */
#endif
#ifndef LT_STRAIGHT_PID_KD_EXTRA
#define LT_STRAIGHT_PID_KD_EXTRA  0.14f
#endif
#ifndef LT_D_FILTER_STRAIGHT_ALPHA
#define LT_D_FILTER_STRAIGHT_ALPHA  0.052f /* D 更钝，抑光电量化抖动 */
#endif
#ifndef LT_STRAIGHT_VZ_LP_ALPHA
#define LT_STRAIGHT_VZ_LP_ALPHA  0.28f /* vz 低通更强，直道摆头更柔 */
#endif

/* 弯/直角：优先把横向(Vz)打足、纵向略让路；vy 输出限斜率，减少「一顿一顿」 */
#ifndef LT_CURVE_ERR_ENTER
#define LT_CURVE_ERR_ENTER  1.42f /* 稍易进弯模式，圆弧/直角更跟线 */
#endif
#ifndef LT_CURVE_BLEND_ALPHA
#define LT_CURVE_BLEND_ALPHA  0.28f /* s_curve_blend 更快跟上 want，S 接直道少滞后 */
#endif
/* 急 S：相邻两帧 |Δerr| 大 → 强制抬高弯权重，避免当中间 |err| 变小时掉到「直道+ vz 低通」 */
#ifndef LT_S_CURVE_ERR_JUMP_TH
#define LT_S_CURVE_ERR_JUMP_TH  1.55f /* 略降阈值，更早锁高弯权 */
#endif
#ifndef LT_S_CURVE_JUMP_WANT_MIN
#define LT_S_CURVE_JUMP_WANT_MIN  0.82f
#endif
#ifndef LT_STRAIGHT_DERR_MAX
 #define LT_STRAIGHT_DERR_MAX  1.12f /* 偏差变化快时不判直道加速，防 S 弯中段误加速 */
#endif
#ifndef LT_STRAIGHT_VZ_LP_CURVE_BLEND_MAX
#define LT_STRAIGHT_VZ_LP_CURVE_BLEND_MAX  0.15f /* 略收紧：弯里少开 vz 低通，转向更跟线 */
#endif
#ifndef LT_TIGHT_S_VY_SCALE
#define LT_TIGHT_S_VY_SCALE  0.91f /* 急变弯+已在弯模式时略降 vy，减全白丢线 */
#endif
#ifndef LT_CURVE_STEER_PRIOR
#define LT_CURVE_STEER_PRIOR  0.21f /* 弯里横向打足，贴弧 */
#endif
#ifndef LT_CURVE_VY_EXTRA_PENALTY
#define LT_CURVE_VY_EXTRA_PENALTY  0.20f /* blend=1 时再乘 (1-K)，相对直道略降纵向 */
#endif
#ifndef LT_VZ_SLEW_CURVE_ADD
#define LT_VZ_SLEW_CURVE_ADD  2.42f /* 弯里 vz 斜率更大，急变线跟得上 */
#endif
#ifndef LT_CURVE_VZ_DEADZONE_SCALE
#define LT_CURVE_VZ_DEADZONE_SCALE  0.48f /* 弯里更小舵死区，细修贴弧 */
#endif
#ifndef LT_VY_SLEW_UP_PT
#define LT_VY_SLEW_UP_PT   1.12f /* 每周期 vy 最大增加量（与 LINE_SPEED 同量纲） */
#endif
#ifndef LT_VY_SLEW_DN_PT
#define LT_VY_SLEW_DN_PT   2.05f /* 入弯允许 vy 较快下降 */
#endif

/* 直道略提速：|err| 小、非弯、非迷宫/路口时再乘系数（与 s_curve_blend 互斥） */
#ifndef LT_STRAIGHT_ERR_MAX
#define LT_STRAIGHT_ERR_MAX  1.58f /* 很居中才算「直道」略提速，减少边界抖 */
#endif
#ifndef LT_STRAIGHT_BLEND_ALPHA
#define LT_STRAIGHT_BLEND_ALPHA  0.18f
#endif
#ifndef LT_STRAIGHT_SPEED_BOOST
#define LT_STRAIGHT_SPEED_BOOST  0.045f /* 居中直道在 vy 上再乘 (1+K*s_straight_blend) */
#endif
#ifndef LT_STRAIGHT_VY_CAP_K
#define LT_STRAIGHT_VY_CAP_K  1.08f /* 相对 LINE_SPEED 封顶，与弯道拉开差距 */
#endif

/* 迷宫/阶梯框等复杂区：再压一点纵向，优先贴中路+里程节点找出口（须标定 LINE_TRACK_MAZE_*） */
#ifndef LT_MAZE_COMPLEX_VY_K
#define LT_MAZE_COMPLEX_VY_K  0.88f
#endif
#ifndef LT_MAZE_LOST_FORWARD_K
#define LT_MAZE_LOST_FORWARD_K  0.24f /* 迷宫内全白时比 LT_LOST_FORWARD_K 更慢 */
#endif
#ifndef LT_MAZE_LOST_VZ_SCALE
#define LT_MAZE_LOST_VZ_SCALE  0.45f /* 丢线时弱转向找线，避免大转钻进旁路 */
#endif

#define LINE_LOST        0xFF    /* 全白=丢线 */
#define SENSOR_CENTER     3.5f    /* 质心法中心 */
#define ERR_SCALE         18.0f   /* 质心→err缩放 */

/* 里程触发路线状态机参数
 * 触发：累计路程 ≥ 有效触发里程 - ROUTE_TRIGGER_LEAD_M（与 nav_route 中路表米、NAV_ROUTE_ENCODER520_ODOM_SCALE 同单位；默认 1:1）。
 * 执行：命中后 s_route_guide_ticks≈guide_ms/LOOP，每周期用 ROUTE_GUIDE_ERR 把 err 拉向目标方向，PID 连续算 vz，
 *       即持续一小段时间的转向引导，不是「数值一到就只转一下」。
 */
#define ROUTE_TRIGGER_LEAD_M  0.08f   /* 默认提前8cm开始给PID转向偏置 */
#define ROUTE_GUIDE_ERR       28.0f   /* 给PID的指导级偏差，进一步增大 */
#define ROUTE_GUIDE_SPEED_K   0.55f   /* 路线节点引导期间降速到55%（非关键弯） */
#ifndef ROUTE_GUIDE_KEY_SPEED_K
#define ROUTE_GUIDE_KEY_SPEED_K  0.48f /* idx2~4 + 迷宫尾虚拟节点：再慢一点，多判光电 */
#endif
#ifndef ROUTE_APPROACH_SLOW_M
#define ROUTE_APPROACH_SLOW_M  0.30f /* 距下一触发线前约此距离内预减速（仍无引导帧时） */
#endif
#ifndef ROUTE_APPROACH_VY_K
#define ROUTE_APPROACH_VY_K  0.80f
#endif
#ifndef ROUTE_GUIDE_PATTERN_SLOW_K
#define ROUTE_GUIDE_PATTERN_SLOW_K  0.90f /* 引导中且已识别宽T/650左形态时再乘一次 */
#endif
#define ROUTE_RELEASE_ERR     2.5f    /* 回正释放阈值：误差足够小就结束上一个引导 */
#define ROUTE_RELEASE_CNT     4       /* 连续4帧居中后，结束上一个引导 */
/* 迷宫光电虚拟下标（非 nav_route 表内）；尾段右转出迷宫见 LtRoute_MazeTailL78OnlyOk（宏可调严/松） */
#ifndef LT_MAZE_VIRT_ENTRY_T_RIGHT_IDX
#define LT_MAZE_VIRT_ENTRY_T_RIGHT_IDX  252u /* T 字 8 路全黑 → 右转 */
#endif
#ifndef LT_MAZE_VIRT_ENTRY_L123_LEFT_IDX
#define LT_MAZE_VIRT_ENTRY_L123_LEFT_IDX 253u /* L1~L3 黑 → 左转 */
#endif
#ifndef LT_MAZE_VIRT_LEFT_IDX
#define LT_MAZE_VIRT_LEFT_IDX   250u /* 尾段 L12345 左转 */
#endif
#ifndef LT_MAZE_VIRT_RIGHT_IDX
#define LT_MAZE_VIRT_RIGHT_IDX  251u /* 尾段 L7/L8 右转出 */
#endif
#ifndef LT_MAZE_OPTICAL_PRE_ARM_MS
#define LT_MAZE_OPTICAL_PRE_ARM_MS  0u /* 非 0 时首判 T 前整段不采样，易错过短全黑；0=立即判 */
#endif
#ifndef LT_MAZE_OPTICAL_COOLDOWN_MS
#define LT_MAZE_OPTICAL_COOLDOWN_MS  180u /* 缩短冷却，出迷宫光电判定更快；过小易抖 */
#endif
/* 两对对称：进迷宫 先右(T)→再左(L123)；尾段 先左(L12345)→再右(L78)。两对之间同一间隔。 */
#ifndef LT_MAZE_COOLDOWN_BETWEEN_PAIR_MS
#define LT_MAZE_COOLDOWN_BETWEEN_PAIR_MS  120u
#endif
#ifndef LT_MAZE_COOLDOWN_BEFORE_L78_MS
#define LT_MAZE_COOLDOWN_BEFORE_L78_MS  LT_MAZE_COOLDOWN_BETWEEN_PAIR_MS /* 与上同，旧名保留 */
#endif
/* 迷宫：T 后 sendVel 横移 V_x；尾段对称横移 + 直走 */
#ifndef LT_MAZE_ENTRY_VX
#define LT_MAZE_ENTRY_VX  20.0f /* 右横移为正，左横移取负；与底盘 sendVel 一致 */
#endif
/* 进迷宫：右横移前盲冲；出迷宫：左横移前盲冲 + 左横移后盲冲，三处同距（更激进可略加大本宏） */
#ifndef LT_MAZE_BLIND_PRE_LAT_M
#define LT_MAZE_BLIND_PRE_LAT_M  0.10f
#endif
#ifndef LT_MAZE_TAIL_FWD_AFTER_LAT_M
#define LT_MAZE_TAIL_FWD_AFTER_LAT_M  LT_MAZE_BLIND_PRE_LAT_M /* 左横移后盲冲与上同；勿再单独写 0.05 */
#endif
#ifndef LT_MAZE_ENTRY_LAT_MAX_M
#define LT_MAZE_ENTRY_LAT_MAX_M  4.0f /* 右横移最久未出现 12345黑/678白 则强制结束并记弧长（防卡死） */
#endif
#ifndef LT_MAZE_T_LATERAL_LINE_STREAK
#define LT_MAZE_T_LATERAL_LINE_STREAK  2u /* 678 全白(1) 连续帧；须先全黑稳定再横移且 s_maze_t_seen_00_in_lat */
#endif
#ifndef LT_MAZE_TAIL_LAT_MIN_M
#define LT_MAZE_TAIL_LAT_MIN_M  0.05f /* 未记到弧长时的默认左移量 */
#endif
/* 1=左横移仅按「进迷宫右横移」历时结束（与右横移同 Vx 节拍，完全镜像）；0=ODOM 或历时先到（旧逻辑） */
#ifndef LT_MAZE_TAIL_LAT_TIME_ONLY
#define LT_MAZE_TAIL_LAT_TIME_ONLY  1u
#endif
/* 1=出迷宫「左横移前盲走」与进迷宫「右横移前盲走」同距、同时（记进时实际 ODOM 段长+历时，出时按镜像结束） */
#ifndef LT_MAZE_MIRROR_PRE_FWD
#define LT_MAZE_MIRROR_PRE_FWD  1u
#endif
#ifndef LT_MAZE_T_PRE_FULLBLACK_STREAK
#define LT_MAZE_T_PRE_FULLBLACK_STREAK  1u /* 1 帧全黑即可与 score 并行；更敏感时易误触发 */
#endif
#ifndef LT_MAZE_T_OVERSHOOT_BACK_VY_K
#define LT_MAZE_T_OVERSHOOT_BACK_VY_K  0.35f /* 非全黑时倒车 vy=LINE_SPEED*K */
#endif
#ifndef LT_MAZE_ENTRY_VX_SLOW_K
#define LT_MAZE_ENTRY_VX_SLOW_K  0.45f /* 横移减慢，便于 678 变白 */
#endif
#ifndef LT_MAZE_TAIL_L1234_STREAK_FRAMES
#define LT_MAZE_TAIL_L1234_STREAK_FRAMES  1u /* 连续 1 帧即计 streak；另见 score */
#endif
#ifndef LT_MAZE_TAIL_L1234_SCORE_ADD
#define LT_MAZE_TAIL_L1234_SCORE_ADD  2u
#endif
#ifndef LT_MAZE_TAIL_L1234_SCORE_SUB
#define LT_MAZE_TAIL_L1234_SCORE_SUB  0u /* 0=非 L1234 帧不扣分，积分更黏 */
#endif
#ifndef LT_MAZE_TAIL_L1234_SCORE_TRIGGER
#define LT_MAZE_TAIL_L1234_SCORE_TRIGGER  2u
#endif
#ifndef LT_MAZE_T_LOOSE_MIN_BLACK
#define LT_MAZE_T_LOOSE_MIN_BLACK  7u /* 与 8 路全黑并列：近全黑(≥7 路黑)也算「见到 T」 */
#endif
#ifndef LT_MAZE_L1234_LOOSE_MIN_BLACK
#define LT_MAZE_L1234_LOOSE_MIN_BLACK  3u /* L1~L4 至少几路黑（严=4 路） */
#endif
/* 出迷宫等 L1234：1=L1 为黑(0)即通过（加强左前探线）；镜像场对称为 L8 为黑 */
#ifndef LT_MAZE_TAIL_EXIT_L12_IMMEDIATE
#define LT_MAZE_TAIL_EXIT_L12_IMMEDIATE  1u
#endif
#ifndef LT_MAZE_TAIL_L1234_FORCE_MS
#define LT_MAZE_TAIL_L1234_FORCE_MS  3200u /* 再等不到 L1234 则强制左横移盲走，缩短尾段空等 */
#endif
#ifndef LT_MAZE_TAIL_RECOVER_BACK_VY_K
#define LT_MAZE_TAIL_RECOVER_BACK_VY_K  0.32f /* 第4段丢线时倒车 vy=-LINE_SPEED*K */
#endif
#ifndef LT_MAZE_BLIND_FWD_VY_K
#define LT_MAZE_BLIND_FWD_VY_K  0.62f
#endif
/* PRE_MAZE_LEFT(idx3) 门限前不跑迷宫光电：代码里用 NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT)，勿再用固定米数 */
#ifndef LT_MAZE_OPTICAL_MIN_TOTAL_M
#define LT_MAZE_OPTICAL_MIN_TOTAL_M  10.10f /* 仅作文档；逻辑已改用 NavRoute_GetTriggerDistM */
#endif
/* 须与 nav_route.c 一致：idx1=7.30m；idx2=9.60m(标定点)；idx3=10.10m（PRE_MAZE_LEFT）；idx4=14.45m（MAZE_EXIT_RIGHT） */
#ifndef LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT
#define LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT  3u
#endif
#ifndef LT_NAV_ROUTE_IDX_MAZE_EXIT_RIGHT
#define LT_NAV_ROUTE_IDX_MAZE_EXIT_RIGHT  4u /* 名沿历史；正图语义见 nav_route idx4 注释 */
#endif
/* 须与 nav_route.c 一致：idx9=18.40m；idx10=19.15m(盲走圆弧触发距)；idx11=20.2m STOP */
#ifndef LT_NAV_ROUTE_IDX_CIRCLE_OUTER_ARC
#define LT_NAV_ROUTE_IDX_CIRCLE_OUTER_ARC  5u
#endif
/* idx4(出迷宫)～idx5(圆外弧入8字) 弯段：略降纵向 + 按地图轻推弯向（右图左拐/左图右拐） */
#ifndef LT_IDX45_CURVE_GUIDE_ENABLE
#define LT_IDX45_CURVE_GUIDE_ENABLE  1
#endif
#ifndef LT_IDX45_CURVE_VY_K
#define LT_IDX45_CURVE_VY_K  0.84f /* 弯段纵向略降，减轻丢线后 PIVOT */
#endif
#ifndef LT_IDX45_CURVE_GUIDE_SOFT_K
#define LT_IDX45_CURVE_GUIDE_SOFT_K  0.36f
#endif
#ifndef LT_IDX45_CURVE_GUIDE_BLEND
#define LT_IDX45_CURVE_GUIDE_BLEND  0.24f
#endif
#ifndef LT_IDX45_CURVE_SUPPRESS_GEO_CORNER
#define LT_IDX45_CURVE_SUPPRESS_GEO_CORNER  1 /* 1=该窗内禁用 L/R 直角钳位，避免判反打 180° */
#endif
#ifndef LT_NAV_ROUTE_IDX_FIG8_POST_1041
#define LT_NAV_ROUTE_IDX_FIG8_POST_1041  6u
#endif
/* Post950 窗口与 nav_route 下标一致：LtPost950_InOdomWindow 要求 total≥idx4、且 next_idx 为 idx5～idx6（见 nav_route 触发距）。
 * 本下标仅用于 LtPost950_InOdomWindow：next_idx>=FINISH_APPROACH(7) 时关闭 Post950 强转窗口。 */
#ifndef LT_NAV_ROUTE_IDX_FINISH_APPROACH
#define LT_NAV_ROUTE_IDX_FINISH_APPROACH  7u
#endif
/* idx9(18.40m)：路表过点；交口仅循线 PID + 光电（不注入 idx9 里程 ROUTE_GUIDE） */
#ifndef LT_NAV_ROUTE_IDX_RADAR_POST_HINT
#define LT_NAV_ROUTE_IDX_RADAR_POST_HINT  9u
#endif
/* idx10：nav_route 表内该点距离；累计里程达到 NavRoute_GetTriggerDistM(idx10) 时发盲走圆弧（与表一致） */
#ifndef LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT
#define LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT  10u
#endif
/* idx11(20.2m) 终点 STOP：TryFire 后先盲直 LT_FINAL_STOP_BLIND_FWD_M 再完全停机 */
#ifndef LT_NAV_ROUTE_IDX_FINAL_STOP
#define LT_NAV_ROUTE_IDX_FINAL_STOP  11u
#endif
#ifndef LT_FINAL_STOP_BLIND_FWD_M
#define LT_FINAL_STOP_BLIND_FWD_M  0.6f
#endif
#ifndef LT_FINAL_STOP_BLIND_FWD_VY_K
#define LT_FINAL_STOP_BLIND_FWD_VY_K  1.0f
#endif
#ifndef LT_POSMAP_STRONG_LEFT_K
#define LT_POSMAP_STRONG_LEFT_K  1.24f /* idx9 正图：在 LEFT650 上再乘 */
#endif
#ifndef LT_POSMAP_STRONG_LEFT_GUIDE_VY_K
#define LT_POSMAP_STRONG_LEFT_GUIDE_VY_K  0.46f /* idx9 强左/强右引导期纵向（共用） */
#endif
#ifndef LT_POSMAP_STRONG_RIGHT_K
#define LT_POSMAP_STRONG_RIGHT_K  1.24f /* idx9 镜像：在 WIDE_R 上再乘 */
#endif
/* idx8 触发距起贴内弧窗（约 +LT_ARC_KEEP_SEGMENT_LEN_M，至 idx9 前） */
#ifndef LT_NAV_ROUTE_IDX_ARC_KEEP_START
#define LT_NAV_ROUTE_IDX_ARC_KEEP_START  9u
#endif
#ifndef LT_ARC_KEEP_SEGMENT_LEN_M
#define LT_ARC_KEEP_SEGMENT_LEN_M  0.72f /* 物理米：自 idx8 触发点起窗长，至 idx9 前略有余量 */
#endif
#ifndef LT_ARC_KEEP_BASE_BIAS
#define LT_ARC_KEEP_BASE_BIAS  3.2f
#endif
#ifndef LT_ARC_KEEP_OUTSIDE_BOOST
#define LT_ARC_KEEP_OUTSIDE_BOOST  9.0f /* L678 或 L123 全白（弧外侧）时追加 */
#endif
/* idx10 盲走：同步 sendArcDisplacement（阻塞至完成回传）；参数与 (0.09f, ±190, 10) 一致；触发距 = nav_route idx10 */
#ifndef LT_BLIND_ARC_RADIUS_M
#define LT_BLIND_ARC_RADIUS_M  0.09f
#endif
#ifndef LT_BLIND_ARC_ANGLE_MAG_DEG
#define LT_BLIND_ARC_ANGLE_MAG_DEG  190.0f
#endif
#ifndef LT_BLIND_ARC_SPEED
#define LT_BLIND_ARC_SPEED  10.0f
#endif
/* 仅当 LT_POST_950_CORNER_GUIDE_ENABLE=1：过 idx3 后～idx6 前 678/123 强转（见下方 #if 块） */
#ifndef LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX
#define LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX  244u
#endif
#ifndef LT_POST_950_CORNER_GUIDE_MS
#define LT_POST_950_CORNER_GUIDE_MS  420u
#endif
#ifndef LT_POST_950_CORNER_STREAK
#define LT_POST_950_CORNER_STREAK  2u
#endif
/* 0=idx3(10.10m) 后不做光电强转，与 idx2(9.60m) 同（仅路表 IGNORE）；1=恢复旧逻辑：约 idx3～idx6 里程段内 678/123 强转一次（见 LtPost950_InOdomWindow） */
#ifndef LT_POST_950_CORNER_GUIDE_ENABLE
#define LT_POST_950_CORNER_GUIDE_ENABLE  0u
#endif
#ifndef LT_MAZE_VIRT_FIG8_LEFT_IDX
#define LT_MAZE_VIRT_FIG8_LEFT_IDX  249u /* 图1：10.80m 顶弧后拉左进直道 */
#endif
#ifndef LT_FIG8_BRANCH_GUIDE_MS
#define LT_FIG8_BRANCH_GUIDE_MS  300u /* idx6(15.30m) 顶弧支：出弧后早停强引导，减轻丢线 */
#endif
#ifndef LT_FIG8_BRANCH_JUNC_STREAK
#define LT_FIG8_BRANCH_JUNC_STREAK  2u
#endif
#ifndef LT_FIG8_BRANCH_MAX_SEARCH_M
#define LT_FIG8_BRANCH_MAX_SEARCH_M  3.5f /* 顶弧后找路口距离略增，避免未出弧就超时 */
#endif
/* idx4 圆外弧弱引导；idx5 顶弧支须能转出 8 字，分支单独加宏 */
#ifndef LT_FIG8_CIRCLE_OUTER_GUIDE_MS
#define LT_FIG8_CIRCLE_OUTER_GUIDE_MS  160u
#endif
#ifndef LT_FIG8_CIRCLE_OUTER_GUIDE_SOFT_K
#define LT_FIG8_CIRCLE_OUTER_GUIDE_SOFT_K  0.48f /* idx4 武装后圆外弧段 */
#endif
#ifndef LT_FIG8_BRANCH_GUIDE_SOFT_K
#define LT_FIG8_BRANCH_GUIDE_SOFT_K  0.62f /* idx5 顶弧支：出弧后宁可弱引导，避免扯丢线 */
#endif
#ifndef LT_FIG8_BRANCH_GUIDE_ERR_K
#define LT_FIG8_BRANCH_GUIDE_ERR_K  0.94f
#endif
#ifndef LT_FIG8_BRANCH_GUIDE_BLEND
#define LT_FIG8_BRANCH_GUIDE_BLEND  0.40f
#endif
/* 仅顶弧后「左出 8 字」支路：在 BRANCH_SOFT_K×ERR_K 之后再乘，并单独 blend */
#ifndef LT_FIG8_BRANCH_LEFT_GUIDE_EXTRA_K
#define LT_FIG8_BRANCH_LEFT_GUIDE_EXTRA_K  1.02f
#endif
#ifndef LT_FIG8_BRANCH_LEFT_GUIDE_BLEND
#define LT_FIG8_BRANCH_LEFT_GUIDE_BLEND  0.48f
#endif
#ifndef LT_FIG8_CIRCLE_OUTER_GUIDE_BLEND
#define LT_FIG8_CIRCLE_OUTER_GUIDE_BLEND  0.28f /* 圆外弧段每帧向 guide 靠拢 */
#endif
#ifndef LT_FIG8_CIRCLE_OUTER_CURVE_WANT_FLOOR
#define LT_FIG8_CIRCLE_OUTER_CURVE_WANT_FLOOR  0.40f /* 引导期弯权重下限 */
#endif
#ifndef LT_FIG8_CIRCLE_OUTER_GUIDE_VY_K
#define LT_FIG8_CIRCLE_OUTER_GUIDE_VY_K  0.86f /* 圆形外弧引导再压一点纵向 */
#endif
#ifndef LT_MAZE_VIRT_FIG8_RIGHT_IDX
#define LT_MAZE_VIRT_FIG8_RIGHT_IDX  248u /* 图2：10.80m 后拉右进直道 */
#endif
/*
 * 1：仅按 s_map_mirror 选 8 字虚拟左/右支（不依赖地图判别是否锁定）。
 * 0：默认；若 LT_MAP_DETECT_ENABLE=1 且已锁定 mirror，仍会随 s_map_mirror 选支路（与路径表镜像一致，无需强开本宏）。
 */
#ifndef LT_FIG8_VIRT_USE_MAP_MIRROR
#define LT_FIG8_VIRT_USE_MAP_MIRROR  0
#endif
/* ODOM 判左右场：0=关；1=按 xcm 判 ms（默认不翻 err，见 LT_MAP_DETECT_MIRROR_ERR_RAW） */
#ifndef LT_MAP_DETECT_ENABLE
#define LT_MAP_DETECT_ENABLE  1
#endif
#ifndef LT_MAP_DETECT_ARM_DIST_M
#define LT_MAP_DETECT_ARM_DIST_M  0.60f /* 至少走此路程后再记 x_ref，避免起步震荡 */
#endif
#ifndef LT_MAP_DETECT_DX_TH
#define LT_MAP_DETECT_DX_TH  0.10f /* m；|pos_x - x_ref| 超过此再锁存 */
#endif
#ifndef LT_MAP_DETECT_FALLBACK_DIST_M
#define LT_MAP_DETECT_FALLBACK_DIST_M  4.0f /* 累计路程超此仍未达 ±DX_TH 则用 LT_MAP_DETECT_FALLBACK_MIRROR */
#endif
/*
 * 地图左右变体（与 ODOM pos_x 漂移判别配合）：
 * LT_MAP_MIRROR_FORCE：非 0 时上电即锁定，不再自动判别。+1=图1，-1=图2。
 * LT_MAP_DETECT_FALLBACK_MIRROR：判别锁定前初值 + 超时未判出时的默认（切换「默认图」主要改这里）。
 */
#ifndef LT_MAP_MIRROR_FORCE
#define LT_MAP_MIRROR_FORCE  0
#endif
#ifndef LT_MAP_DETECT_FALLBACK_MIRROR
/*
 * +1：图1 / L1b 单张默认图（8 字虚拟支路走「强左」与箭头一致）。
 * -1：仅用于「对称第二场」且外弧后须进**右**横支时；在单图上会与光电左弧冲突，表现为突然右转。
 */
#define LT_MAP_DETECT_FALLBACK_MIRROR  1
#endif
/*
 * 1：纯 |N_PosX| 门槛（米）：|px| 过 LT_MAP_DETECT_ABS_X_M 当帧即锁定 mirror，不看累计路程。
 * 0：LT_MAP_DETECT_ARM_DIST_M + x_ref 漂移 + LT_MAP_DETECT_FALLBACK_DIST_M 兜底（旧逻辑）。
 */
#ifndef LT_MAP_DETECT_USE_ABS_XCM
#define LT_MAP_DETECT_USE_ABS_XCM  1
#endif
#ifndef LT_MAP_DETECT_ABS_X_M
#define LT_MAP_DETECT_ABS_X_M  0.5f /* 米；与 g_odom.pos_x / VOFA px 同单位；|x| 超过即判左/右场 */
#endif
/* 1：锁定后 err_raw×s_map_mirror（整条 PID 随场左右翻，易在弯中突然反向打角）；0：仅迷宫 V_x、8 字虚拟支路等用 ms，巡线 err 不翻 */
#ifndef LT_MAP_DETECT_MIRROR_ERR_RAW
#define LT_MAP_DETECT_MIRROR_ERR_RAW  0
#endif
/* 过 PRE_MAZE 门限(idx2) 后至迷宫光电 MAZE_OPT_DONE 前：统一降纵向，增加判线时间；结束后不再乘 */
#ifndef LT_MAZE_CORRIDOR_VY_K
#define LT_MAZE_CORRIDOR_VY_K  0.68f
#endif
/* 右横移结束后仅凭 ODOM 累计直行此弧长(米)即进入第 4 段（不再用十字路口光电计数） */
#ifndef LT_MAZE_POST_RLAT_STRAIGHT_M
#define LT_MAZE_POST_RLAT_STRAIGHT_M  1.1f
#endif
/* 第 3 段 TAIL_COUNT_CROSS：略弱横向 PID。正场 / 镜像可分别标定（镜像不是「和正场强制同一数值」） */
#ifndef LT_MAZE_EXIT_TAIL3_PID_ERR_SCALE
#define LT_MAZE_EXIT_TAIL3_PID_ERR_SCALE  0.52f
#endif
#ifndef LT_MAZE_EXIT_TAIL3_PID_ERR_SCALE_MIRROR
#define LT_MAZE_EXIT_TAIL3_PID_ERR_SCALE_MIRROR  LT_MAZE_EXIT_TAIL3_PID_ERR_SCALE
#endif
/* 第 4 段：等尾段稳定黑；正图左出迷宫须强横向跟随（原 0 会关掉 err） */
#ifndef LT_MAZE_EXIT_L1234_PID_ERR_SCALE
#define LT_MAZE_EXIT_L1234_PID_ERR_SCALE  0.88f
#endif
#ifndef LT_MAZE_EXIT_L1234_PID_ERR_SCALE_MIRROR
#define LT_MAZE_EXIT_L1234_PID_ERR_SCALE_MIRROR  LT_MAZE_EXIT_L1234_PID_ERR_SCALE
#endif
/* 尾段第 3 段（右横移后至满直行门前）：纵向系数，默认 1.0 不额外缩放 */
#ifndef LT_MAZE_TAIL_STRAIGHT_RUN_VY_K
#define LT_MAZE_TAIL_STRAIGHT_RUN_VY_K  1.0f
#endif
/* 第 4 段：等 L1234 稳定黑左横移 */
#ifndef LT_MAZE_TAIL_SECOND_CROSS_VY_K
#define LT_MAZE_TAIL_SECOND_CROSS_VY_K  0.82f /* 等 L1234/左出迷宫：略提纵向，配合加强 err */
#endif
#ifndef LT_MAZE_T_FULL_NEED_FRAMES
#define LT_MAZE_T_FULL_NEED_FRAMES  1u /* 连续 1 帧全黑即可；另见 score 与误触发风险 */
#endif
#ifndef LT_MAZE_T_ENTRY_SCORE_ADD
#define LT_MAZE_T_ENTRY_SCORE_ADD  2u /* 见全黑 +2 */
#endif
#ifndef LT_MAZE_T_ENTRY_SCORE_SUB
#define LT_MAZE_T_ENTRY_SCORE_SUB  0u /* 0=非 T 帧不扣分；配合「近全黑」易达标 */
#endif
#ifndef LT_MAZE_T_ENTRY_SCORE_TRIGGER
#define LT_MAZE_T_ENTRY_SCORE_TRIGGER  2u /* 与 ADD=2：一帧严/近全黑即触发 */
#endif
#ifndef LT_MAZE_L123_ENTRY_NEED_FRAMES
#define LT_MAZE_L123_ENTRY_NEED_FRAMES  4u
#endif
#ifndef LT_MAZE_L78_NEED_FRAMES
#define LT_MAZE_L78_NEED_FRAMES  3u /* L7/L8 见黑连续帧；命名保留，尾段另见 LT_MAZE_TAIL_L78_FAST_* */
#endif
/* 尾段：LtRoute_MazeTailL78OnlyOk 连续满此帧数即快启尾盲（先于纯 ODOM 门限/等 L1234），减轻 PID 久跟右转线车头右偏 */
#ifndef LT_MAZE_TAIL_L78_FAST_BLIND_ENABLE
#define LT_MAZE_TAIL_L78_FAST_BLIND_ENABLE  1
#endif
#ifndef LT_MAZE_TAIL_L78_FAST_BLIND_FRAMES
#define LT_MAZE_TAIL_L78_FAST_BLIND_FRAMES  1u /* 1 帧即快启尾盲（激进） */
#endif
/* LtRoute_MazeTailL78OnlyOk：0=须 L7+L8 双黑；1=仅须 L7/L8 至少一路黑（更激进） */
#ifndef LT_MAZE_TAIL_L78_RIGHT_OR_ONE_BLACK
#define LT_MAZE_TAIL_L78_RIGHT_OR_ONE_BLACK  1
#endif
/* 0=须 L1~L5 全白（严）；1=仅须 L1~L4 全白，L5 可黑（更激进） */
#ifndef LT_MAZE_TAIL_L78_LOOSE_L5
#define LT_MAZE_TAIL_L78_LOOSE_L5  1
#endif
#ifndef LT_MAZE_TAIL_GUIDE_LEFT_MS
#define LT_MAZE_TAIL_GUIDE_LEFT_MS  320u
#endif
#ifndef LT_MAZE_TAIL_GUIDE_RIGHT_MS
#define LT_MAZE_TAIL_GUIDE_RIGHT_MS 380u
#endif
#ifndef LT_ROUTE_GUIDE_LEFT650_K
#define LT_ROUTE_GUIDE_LEFT650_K  1.12f
#endif
/* idx8 专用刹停（ms）；0=不刹（可与 LT_ROUTE_NODE_DEBUG_PAUSE_MS 每节点暂停叠加） */
#ifndef LT_IDX8_PAUSE_MS
#define LT_IDX8_PAUSE_MS  0u
#endif
#ifndef LT_ROUTE_GUIDE_WIDE_R_K
#define LT_ROUTE_GUIDE_WIDE_R_K  1.42f
#endif
#ifndef ROUTE_WIDE_RELEASE_MAX_BLACK
#define ROUTE_WIDE_RELEASE_MAX_BLACK  4u /* 宽横线时黑块仍多则不按「小 err」提前结束引导 */
#endif

/* 与 ODOM_TELEMETRY 共用 USART3 时务必为 0，否则无线串口被调试信息淹没 */
#define IR_DEBUG_USART3  0
#define IR_DEBUG_CNT     500
/*
 * 1：全白丢线当帧经 USART3 打一行 LOST（s_err_last_seen、LostChooseVz 原始值、上一帧 vz、本次下发 vz、fsm）。
 *    若「线在左却 out_vz>0」：先查 es 符号是否反（IR_GRAYSCALE_INVERT_DIGITAL / 地图镜像），再查 LINE_TRACK_INVERT_STEER_CMD。
 */
#ifndef LT_DEBUG_LINE_LOST_USART3
#define LT_DEBUG_LINE_LOST_USART3  0
#endif

/* ==================== 光电拐角状态机（参考 corner_logic_desc，无摄像头版） ==================== */
typedef enum {
	LT_FSM_TRACKING = 0,    /* 正常循迹 PID */
	LT_FSM_PRE_CORNER,      /* 疑似路口：减速，消抖可退回 TRACKING */
	LT_FSM_BLIND_SPRINT,    /* 竖线感消失：短距离盲冲越过交叉口 */
	LT_FSM_PIVOTING         /* 间歇旋转寻线 */
} LtFsmState_t;

/* 路口判定：排除全黑0x00（长方形穿心/宽块），否则易一直 PRE_CORNER 吸进侧线 */
#define LT_JUNC_BLACK_MIN       6u     /* 6~7 路见黑才像“真 T 字横线”，8 路全黑不算 */
#define LT_PRE_ENTER_FRAMES     3u     /* 略增，抑密集路口抖进 PRE */
#define LT_PRE_EXIT_FRAMES      5u     /* 连续 5 帧无路口且 err 小 → 退回 TRACKING */
#define LT_PRE_ERR_MAX          3.0f   /* PRE 内判“线较正”的 |err| 阈值 */
#define LT_BLIND_FRAMES         8u     /* 盲冲帧数略短，减少纯盲冲距离 */
#define LT_LOST_TO_BLIND_FRAMES 24u    /* 连续丢线更久才盲冲→PIVOT，S 弯先慢转找线 */
#define LT_FSM_COOLDOWN_M       0.18f  /* PIVOT/盲冲结束后 18cm 内不再进 PRE/盲冲 */
/* 流程图「预设 vs 自转」：下一里程节点很近 = 走预设，延长丢线容忍；直角且预设远 = 快自转 */
#define LT_FLOW_PRESET_NEAR_M   0.35f
#define LT_FLOW_FREEPIVOT_FAR_M 0.42f
#define LT_LOST_PRESET_PATIENCE 26u    /* 等 ODOM 节点触发 */
#define LT_LOST_CORNER_FAST      14u   /* 直角锁定时略晚再快盲冲 */
#define LT_PIVOT_SEG_MS         260u
#define LT_PIVOT_WZ             13.0f /* 原 20 易原地甩 180°，略减角速度 */
#define LT_PIVOT_TIMEOUT_MS     4000u

/*
 * 迷宫段（两侧有出口、中间多矩形交错）：光电极易沿框边绕圈。
 * 在 [LINE_TRACK_MAZE_DIST0_M, LINE_TRACK_MAZE_DIST1_M] 内：
 *   - 关掉拐角 FSM；歧义 pattern 强制“贴竖直中线”；
 *   - 丢线时慢速直行，不原地大转，避免钻进侧向出口。
 * 与 nav_route 起点累计里程一致，用卷尺+实车标定 dist0/dist1；dist1<=dist0 表示关闭。
 *
 * 第二段：图上大区 1.4（横向框 + T 字前），与 1.3 独立标定。详见 ROUTE_13_POINTS.md。
 * 填好后：直道加速/弯权重自动在区外生效；区内 LT_MAZE_COMPLEX_VY_K 压速 + 中路缩放，利于找出口。
 */
#define LINE_TRACK_MAZE_DIST0_M   0.0f   /* 迷宫1（1.3 阶梯框）起点 m，默认 0=关 */
#define LINE_TRACK_MAZE_DIST1_M     0.0f   /* 迷宫1 终点 m */
#define LINE_TRACK_MAZE2_DIST0_M   0.0f   /* 迷宫2（1.4 横向框）起点 m */
#define LINE_TRACK_MAZE2_DIST1_M   0.0f   /* 迷宫2 终点 m */
#define LT_MAZE_ERR_SCALE        0.42f  /* 非歧义时横向误差缩放，减小甩尾 */
#define LT_MAZE_VZ_ABS_MAX       12.0f  /* 迷宫内限制转角，防拐进旁路 */

/*
 * 中间两路（L4、L5）优先：与查表 err 融合，目标常态下 L4、L5 均压黑线，
 * 车身对准跑道正中，利于后续里程/节点与既定路程衔接。
 */
#define LINE_TRACK_MID_HOLD_BLEND       0.62f  /* 略减中路拖曳，S 末端接直道更快跟横向 err */
#define LINE_TRACK_MID_HOLD_BLEND_BOTH  0.90f
#define LINE_TRACK_MID_HOLD_MAZE_PLUS   0.10f
#define LINE_TRACK_MID_PAIR_GAIN        4.05f  /* 仅 L4 或仅 L5：更早把线拉回中路 */
#define LINE_TRACK_MID_FINE_GAIN        1.88f
#define LINE_TRACK_MID_HOLD_WMAX        0.90f

/* 8 路质心 err 与查表融合：软化相邻 pattern 跳变、补 default 分支 */
#ifndef LT_ERR_CENTROID_BLEND
#define LT_ERR_CENTROID_BLEND  0.36f /* 略增质心权重，缓查表跳变，利于 S 弯/连续弯 */
#endif
/* 几何巡线：直道向 L4/L5 中路融合权重；直角最小 |err| 地板，便于 PID 果断转向 */
#ifndef LT_GEO_STRAIGHT_MID_BLEND
#define LT_GEO_STRAIGHT_MID_BLEND  0.78f
#endif
#ifndef LT_GEO_CORNER_ERR_FLOOR
#define LT_GEO_CORNER_ERR_FLOOR  6.5f
#endif
/*
 * 大 U 外弧/急左转时，宽幅黑区可能短时满足「L4~L8 全黑」→ 被判成右直角，err 钳到负 → vz>0 顺时针甩出弧外
 * （与赛题图右上 U 顶红箭头一致）。若 ODOM 上一帧 Δyaw 已为左转，则跳过该右直角钳位，交给下方 345/456 中路融合。
 */
#ifndef LT_GEO_RIGHT_CORNER_SUPPRESS_ON_LEFT_YAW_ENABLE
#define LT_GEO_RIGHT_CORNER_SUPPRESS_ON_LEFT_YAW_ENABLE  1
#endif
#ifndef LT_GEO_RIGHT_CORNER_SUPPRESS_LEFT_YAW_DEG
#define LT_GEO_RIGHT_CORNER_SUPPRESS_LEFT_YAW_DEG  0.32f
#endif
/* 正常巡线见光电直角：弯权更快拉满（无里程引导时） */
#ifndef LT_GEO_RA_CURVE_BLEND_ALPHA
#define LT_GEO_RA_CURVE_BLEND_ALPHA  0.42f
#endif

/* ODOM Δyaw 叠到 err：默认关闭（易与光电/ODOM 不同步时反向干扰） */
#ifndef LT_YAW_ALIGN_ASSIST_ENABLE
#define LT_YAW_ALIGN_ASSIST_ENABLE  0
#endif
#ifndef LT_YAW_ALIGN_GAIN
#define LT_YAW_ALIGN_GAIN  0.38f /* 基准；直道小 err 时会再乘系数，弯里会放大 */
#endif
#ifndef LT_YAW_ALIGN_ABSMAX
#define LT_YAW_ALIGN_ABSMAX  4.5f
#endif

/* 丢线(全白)时前进比例：弯弧上略低，减少「只冲不转」飞出跑道 */
#ifndef LT_LOST_FORWARD_K
#define LT_LOST_FORWARD_K  0.26f
#endif
#ifndef LT_LOST_VZ_SOFT
#define LT_LOST_VZ_SOFT    0.68f  /* ref 接近 0 且 yaw 也无明确弯向时的 vz 比例 */
#endif
#ifndef LT_LOST_TRACK_VZ_CAP
#define LT_LOST_TRACK_VZ_CAP  21.0f /* 非节点引导丢线寻线 vz 上限(度)，避免满舵甩尾 */
#endif
/* 全白丢线：与上一帧 vy/vz 反向（前→后、右转→左转），并与 LineTrack_LostChooseVz(ODOM/yaw) 融合 */
#ifndef LT_LOST_BACK_K
#define LT_LOST_BACK_K  0.38f
#endif
#ifndef LT_LOST_VY_OPP_THRESH
#define LT_LOST_VY_OPP_THRESH  0.16f
#endif
#ifndef LT_LOST_VZ_OPP_THRESH
#define LT_LOST_VZ_OPP_THRESH  0.22f
#endif
#ifndef LT_LOST_VZ_OPP_GAIN
#define LT_LOST_VZ_OPP_GAIN  0.80f
#endif
#ifndef LT_LOST_VZ_OPP_BLEND
#define LT_LOST_VZ_OPP_BLEND  0.0f /* 默认完全不用「上一帧 vz 取反」，避免与 err 冲突；需老逻辑时再调大 */
#endif
/* |ref| 超过此门限即按 err 符号给转向，不再交给 Δyaw（弯中丢线时 ODOM 常把方向判反） */
#ifndef LT_LOST_REF_SIGN_GATE
#define LT_LOST_REF_SIGN_GATE  0.018f
#endif
/* 1：丢线寻线允许用 Δyaw/TurnHint；0：仅用 s_err_last_seen/里程节点（光编不稳时建议 0） */
#ifndef LT_LOST_USE_ODOM_YAW_HINT
#define LT_LOST_USE_ODOM_YAW_HINT  0
#endif
#ifndef LT_MAZE_LOST_BACK_K_SCALE
#define LT_MAZE_LOST_BACK_K_SCALE  0.62f /* 迷宫内倒车略慢 */
#endif

/* 无新光电包时仍发上一帧 vy/vz，避免底盘指令断档（与仅在有新包时算 PID 配套） */
#ifndef LT_LINE_RESEND_LAST_VEL
#define LT_LINE_RESEND_LAST_VEL  1
#endif

/* 里程节点 LED 指示（DF_Communication LED_Set：1=亮） */
#define LT_LED_ROUTE_DEBUG       1
#define LT_LED_ROUTE_PULSE_MS    130u
#define LT_LED_ROUTE_GAP_MS      220u

#if LT_LED_ROUTE_DEBUG
typedef enum {
	LT_LED_ST_IDLE = 0,
	LT_LED_ST_ON,
	LT_LED_ST_GAP
} LtLedRouteSt_t;

static LtLedRouteSt_t s_route_led_st = LT_LED_ST_IDLE;
static u8 s_route_led_rem = 0;
static u32 s_route_led_next_tick = 0;

static void LineTrack_LedRoute_Reset(void)
{
	s_route_led_st = LT_LED_ST_IDLE;
	s_route_led_rem = 0;
	LED_Set(0);
}

/* fired_idx 为 0..ROUTE_CNT-1 → 闪 1..N 次 */
static void LineTrack_LedRoute_OnNodeFired(u8 fired_idx)
{
	u8 n;

	if (s_route_led_st != LT_LED_ST_IDLE)
		return; /* 上一段还在闪，忽略，避免次数错乱 */
	n = (u8)(fired_idx + 1u);
	if (n > 16u)
		n = 16u;
	s_route_led_rem = n;
	s_route_led_st = LT_LED_ST_ON;
	LED_Set(1);
	s_route_led_next_tick = HAL_GetTick() + LT_LED_ROUTE_PULSE_MS;
}

static void LineTrack_LedRoute_Process(void)
{
	u32 now;

	if (s_route_led_st == LT_LED_ST_IDLE)
		return;
	now = HAL_GetTick();
	if ((s32)(now - s_route_led_next_tick) < 0)
		return;
	switch (s_route_led_st) {
	case LT_LED_ST_ON:
		LED_Set(0);
		s_route_led_rem--;
		if (s_route_led_rem == 0u) {
			s_route_led_st = LT_LED_ST_IDLE;
			return;
		}
		s_route_led_st = LT_LED_ST_GAP;
		s_route_led_next_tick = now + LT_LED_ROUTE_GAP_MS;
		break;
	case LT_LED_ST_GAP:
		LED_Set(1);
		s_route_led_st = LT_LED_ST_ON;
		s_route_led_next_tick = now + LT_LED_ROUTE_PULSE_MS;
		break;
	default:
		break;
	}
}

static u8 LineTrack_LedRoute_IsIdle(void)
{
	return (s_route_led_st == LT_LED_ST_IDLE) ? 1u : 0u;
}
#else
static u8 LineTrack_LedRoute_IsIdle(void)
{
	return 1u;
}
#endif

/*
 * 与「里程判定点连闪」共用 PC13；执行顺序为 LineTrack_LedRoute_Process 后再调本函数。
 * - ODOM 已解析：常亮（优先于节点闪灯，避免误判「解析断了」）
 * - 未解析但 USART1 近期有字节：慢闪
 * - 无数据：灭
 * 未解析且节点在闪时：本函数提前 return，由 Route 独占灯效。
 *
 * 注意：ODOM 常为突发帧，两帧之间可能静默数百 ms～1s；若「近期有字节」窗口过短（原 500ms），
 * 会在帧间隙误判为无数据 → 「先慢闪一会又一直灭」。故放宽「链路活跃」判定。
 */
#define LT_USART1_ACTIVITY_MS       3000u  /* 距上次 RX 字节小于此 → 慢闪（有流量但未解析出 ODOM） */
#define LT_ODOM_UNPARSE_BLINK_MS     220u
/* 10Hz ODOM 约 100ms 一帧；超过此时间无新解析 → 数据陈旧（电机干扰/掉帧时常如此） */
#define LT_ODOM_FRESH_MS             400u

static void LineTrack_OdomLinkLed_Tick(void)
{
	u32 now;
	u32 last;
	u32 gap;
	u32 age;

	now = HAL_GetTick();

	/* valid 只表示「曾经成功解析」；电机一转灯灭多为新 ODOM 帧进不来，而非 valid 被清 0。 */
	if (Odom_IsValid()) {
		age = (u32)(now - g_odom.update_tick);
		if (age < LT_ODOM_FRESH_MS) {
			LED_Set(1);
			return;
		}
		/* 曾成功过，但长时间无新帧：快双闪，与「从未解析」区分 */
		LED_Set(((now / 120u) & 1u) ? 1u : 0u);
		return;
	}
	if (!LineTrack_LedRoute_IsIdle())
		return;
	last = Uart1_GetLastRxTick();
	gap = (u32)(now - last);
	if (Uart1_RxEver() && gap < LT_USART1_ACTIVITY_MS)
		LED_Set(((now / LT_ODOM_UNPARSE_BLINK_MS) & 1u) ? 1u : 0u);
	else
		LED_Set(0);
}

/* ==================== 静态变量 ==================== */
static float s_err = 0;
static float s_err_last = 0;
static float s_err_integral = 0;
static float s_d_filtered = 0;   /* 微分低通滤波 */
#if IR_DEBUG_USART3
static u16 s_debug_cnt = 0;
#endif
static u8 s_sensor_valid = 0;
/* 首次 LineTracking_Step 时刻；用于无有效 $D 帧时的超时解锁（ms） */
static u32 s_lt_step_epoch_ms;
#ifndef LT_IR_BOOTSTRAP_VALID_MS
#define LT_IR_BOOTSTRAP_VALID_MS  1200u
#endif

/* 路口动作见 nav_route.h（NavRouteAction_t） */
static u16 s_force_action_ticks = 0;  /* 极短越线防抖 */
static u16 s_route_guide_ticks = 0;   /* PID偏向持续时间 */
static u8 s_route_release_cnt = 0;    /* 回正释放计数 */
static NavRouteAction_t s_current_act = NAV_ROUTE_STRAIGHT;
static NavRouteAction_t s_last_junction_act = NAV_ROUTE_STRAIGHT;
static u8 s_last_route_node_idx = 255u; /* 最近一次触发的 nav_route 表下标；255=无效 */
static u8 s_fig8_branch_armed;         /* 已过圆形外弧或 idx5(10.80m) 节点，正在等路口条件 */
static u8 s_fig8_branch_done;          /* 已触发左引导或已超时放弃 */
static u8 s_fig8_junc_streak;
static u8 s_fig8_circle_outer_l12;     /* 1=圆形外弧段：仅 L1+L2 见黑触发强左；0=8 字原 T/横线/左直角逻辑 */
#if LT_POST_950_CORNER_GUIDE_ENABLE != 0u
static u8 s_post_950_corner_streak;
static u8 s_post_950_forced_once;    /* Post950 强转已用；Init 清零 */
#endif
#if LT_ROUTE_NODE_DEBUG_PAUSE_MS > 0
static u32 s_route_debug_pause_until_ms; /* 非 0：任意节点 TryFire 后调试刹停，直到该 tick */
#endif
static u8 s_line_track_halted;         /* 终点盲直结束或其它 STOP：巡线停，持续发速度 0 */
static u8 s_final_stop_blind_active;   /* 1：idx11 STOP 已触发，正盲直至 LT_FINAL_STOP_BLIND_FWD_M */
#if LT_RADAR_BRANCH_ENABLE
typedef enum {
	LT_RADAR_OFF = 0,
	LT_RADAR_WAIT_T,
	LT_RADAR_SCANNING,
	LT_RADAR_BRANCH_ROT,
	LT_RADAR_CREEP
} LtRadarPhase_t;
static LtRadarPhase_t s_lt_radar_phase;
static u8 s_lt_radar_t_streak;
static u32 s_lt_radar_motion_start;
static u8 s_lt_radar_go_left;
static u8 s_lt_radar_pick_valid; /* 1=已完成雷达选边，idx9 盲转方向跟雷达、光电仅作触发 */
static u16 s_lt_radar_creep_ticks;
static u32 s_oled_radar_pick_deadline_ms; /* 0=不显示选边结果；非 0 时 tick<此值则 OLED 雷达选左/右字模 */
static u8 s_oled_radar_pick_is_left;
#ifndef LT_RADAR_TJUNC_STREAK
#define LT_RADAR_TJUNC_STREAK  1u /* 与迷宫入口 T 同判据时 1 帧即可进扫描 */
#endif
#ifndef LT_RADAR_PICK_ZERO_BLOCKED_MIN
#define LT_RADAR_PICK_ZERO_BLOCKED_MIN  7u /* 原 10：一侧连续全 0 判「极近」；略降更灵敏 */
#endif
#ifndef LT_RADAR_CREEP_TICKS
#define LT_RADAR_CREEP_TICKS  18u
#endif
#ifndef LT_RADAR_BRANCH_ROT_DEG
#define LT_RADAR_BRANCH_ROT_DEG  45.0f
#endif
/* idx9～idx10 口字出口：sendrot_AsyncBegin（与雷达选支路后 BRANCH_ROT 相同），转角 |rz|=LT_NAV_IDX9_EXIT_SENDROT_DEG */
#ifndef LT_NAV_IDX9_EXIT_SENDROT_DEG
#define LT_NAV_IDX9_EXIT_SENDROT_DEG  40.0f /* 度；左负右正，与底盘 sendrot 约定一致 */
#endif
#ifndef LT_NAV_IDX9_EXIT_SENDROT_OMEGA
#define LT_NAV_IDX9_EXIT_SENDROT_OMEGA  20.0f /* sendrot 第 4 参 r_max，与 LT_RADAR_BRANCH_ROT 一致 */
#endif
#ifndef LT_NAV_IDX9_EXIT_SENDROT_EDGE_STREAK
#define LT_NAV_IDX9_EXIT_SENDROT_EDGE_STREAK  1u /* 1=尽快触发；需更稳可改 2 */
#endif
#ifndef LT_NAV_IDX9_WINDOW_USE_LEAD
#define LT_NAV_IDX9_WINDOW_USE_LEAD  1 /* 1：窗起点=t9-ROUTE_TRIGGER_LEAD，与 TryFire 对齐 */
#endif
#ifndef LT_NAV_IDX9_EXIT_FORCED_AFTER_M
#define LT_NAV_IDX9_EXIT_FORCED_AFTER_M  0.40f /* 窗内光电未判到缘时，过 t9+此值里程兜底 sendrot */
#endif
#ifndef LT_NAV_IDX9_SUPPRESS_LOST_PIVOT
#define LT_NAV_IDX9_SUPPRESS_LOST_PIVOT  1 /* 1：idx9 窗内未完成盲转时勿进 BLIND_SPRINT/PIVOT */
#endif
/* 旋转结束后先沿当前朝向盲走一段（里程弧长），再恢复 PID */
#ifndef LT_NAV_IDX9_POST_ROT_BLIND_FWD_M
#define LT_NAV_IDX9_POST_ROT_BLIND_FWD_M  0.04f
#endif
#ifndef LT_NAV_IDX9_POST_ROT_BLIND_FWD_VY_K
#define LT_NAV_IDX9_POST_ROT_BLIND_FWD_VY_K  LT_MAZE_BLIND_FWD_VY_K
#endif
static u8 s_nav_idx9_exit_rot_done;      /* 本窗已执行过一次出口盲转（含 sendrot 完成） */
static u8 s_nav_idx9_exit_rot_edge_streak;
static u8 s_nav_idx9_sendrot_win_int_cleared;
static u8 s_nav_idx9_exit_rot_wait;      /* 1=等待 sendrot 完成（勿发巡线速，同雷达 BRANCH_ROT） */
static u32 s_nav_idx9_exit_rot_motion_start;
static u8 s_nav_idx9_post_rot_blind_fwd_active; /* 1=正执行转后盲直，满 LT_NAV_IDX9_POST_ROT_BLIND_FWD_M 后清 */
#endif /* LT_RADAR_BRANCH_ENABLE */
static u32 s_last_action_tick = 0;    /* 节点触发 / 进入 PIVOTING 时刻，用于超时保护 */

static LtFsmState_t s_fsm = LT_FSM_TRACKING;
static u8 s_junc_enter_streak = 0;
static u8 s_pre_exit_streak = 0;
static u8 s_blind_ticks = 0;
static s8 s_pivot_dir = 1;             /* +1 vz>0 右转找线，-1 左转 */
static u16 s_pivot_seg_timer = 0;      /* 当前半周期计数 */
static u8 s_pivot_rot_phase = 1;       /* 1=旋转中 0=停顿 */
static u8 s_lost_streak = 0;           /* 连续全白帧数，防误判盲冲 */
static float s_fsm_cool_until_dist = -1.0f; /* <0 表示未启用；≥0 时里程未到该值则抑制 FSM 激进行为 */
static float s_err_last_seen = 0.0f;     /* 最近一次非丢线时的 err，供盲冲选转向 */
static float s_vz_slew_prev = 0.0f;      /* 转向变化率限制，丝滑输出 */
static float s_vz_straight_lp = 0.0f;    /* 直道 vz 一阶低通，抑高频晃 */
static float s_vy_smoothed = 0.0f;       /* 纵向速度一阶限斜率，丝滑加减速 */
static float s_curve_blend = 0.0f;     /* 0=直道 1=弯/直角需求强，低通平滑 */
static float s_straight_blend = 0.0f;  /* 0~1 直道加速权重，与弯互斥 */
static float s_last_send_vx = 0.0f;
static float s_last_send_vy = 0.0f;
static float s_last_send_vz = 0.0f;
static u32 s_last_ir_tick_ms = 0u;     /* 上一包光电解析时刻，用于 PID 积分 dt */
static float s_prev_err_raw_sc = 0.0f; /* 上一包 err_raw，用于急 S 弯 |Δerr| */
static u8 s_blind_arc_done;              /* 本局已跑过 idx10 盲走则不再触发 */
static u8 s_flow_hnov_streak = 0;        /* 横线感在而竖带不在，连续帧 */
static u8 s_flow_right_angle_latched = 0; /* 判定为「直角：竖线消失」后，优先自转或等预设 */

/* 下一里程点为出迷宫点：T→…→右横移→直行(可 L78 快判)→等 L1234 黑→盲走→左横移(对称距)→短盲直→DONE；详见 LineTrack_MazeOptical_Update */
#define MAZE_OPT_IDLE              0u
#define MAZE_OPT_ENTRY_WAIT_T      1u
#define MAZE_OPT_TAIL_COUNT_CROSS  3u /* 右横移后仅 ODOM 直行门限（名保留兼容） */
#define MAZE_OPT_TAIL_WAIT_L1234   4u /* 等 L1~L4 稳定黑再左移 */
#define MAZE_OPT_DONE              6u
#define MAZE_OPT_BLIND_RUN         7u
#define MAZE_BLIND_SUB_ENTRY_LAT_R      2u /* 慢右横移，记弧长 */
#define MAZE_BLIND_SUB_TAIL_LAT_L         3u
#define MAZE_BLIND_SUB_TAIL_FWD           4u
#define MAZE_BLIND_SUB_T_PRE_STABILIZE    5u /* 全黑稳定/倒车 */
#define MAZE_BLIND_SUB_ENTRY_PRE_FWD      6u /* 全黑稳定后、右横移前盲走 */
#define MAZE_BLIND_SUB_TAIL_PRE_FWD       8u /* L1234 就绪后、左横移前盲走 */
static u8 s_maze_opt_phase;
static u8 s_maze_entry_t_streak;
static u32 s_maze_opt_cd_until;
static u32 s_maze_opt_arm_until;
static u8 s_maze_blind_sub;
static u8 s_maze_blind_next_phase;
static float s_maze_lat_saved_m;       /* 第2步右横移：NavOdom 段长→存此；第4步左横移按同距离对称（必须保留 ODOM） */
static u32 s_maze_lat_saved_ms;        /* 第2步右横移历时；第4步左横移见 LT_MAZE_TAIL_LAT_TIME_ONLY */
static u32 s_maze_entry_lat_start_tick; /* 右横移段起点 tick */
static u32 s_maze_tail_lat_start_tick;  /* 左横移段起点 tick */
static float s_maze_mirror_pre_fwd_m;   /* 进迷宫：右横移前盲走结束时 NavOdom 段长；出迷宫左横移前盲走镜像用 */
static u32 s_maze_mirror_pre_fwd_ms;    /* 进迷宫：右横移前盲走历时 ms */
static u32 s_maze_entry_pre_fwd_start_tick;
static u32 s_maze_tail_pre_fwd_start_tick;
static u8 s_maze_t_exit_line_streak;   /* 第2步 678 全白结束条件连续帧 */
static u8 s_maze_t_fullblack_streak;   /* T 后全黑稳定帧（预横移） */
static u8 s_maze_t_seen_00_in_lat;     /* 右横移前已在全黑稳定，横移中曾再见到 0x00 则保持为 1 */
static u8 s_maze_tail_l1234_streak;    /* 尾段 L1~L4 稳定黑 */
static u32 s_maze_tail_l1234_enter_tick; /* 进入 MAZE_OPT_TAIL_WAIT_L1234 时刻 */
static u8 s_maze_entry_t_score;        /* T 字：+ADD/-SUB，补短全黑/闪白间隙 */
static u8 s_maze_tail_l1234_score;
static u8 s_maze_t_pre_score;          /* 盲走 T 前全黑稳定：同上 */
static u8 s_maze_tail_l78_exit_streak; /* L78 尾出形态连续帧，满则快启尾盲 */

static s8 s_map_mirror = (s8)LT_MAP_DETECT_FALLBACK_MIRROR; /* 初值见 LT_MAP_DETECT_FALLBACK_MIRROR */
#if LT_MAP_DETECT_ENABLE
static u8 s_map_detect_locked;
static u8 s_map_odom_latched; /* 1=已由 xcm/x_ref 门槛闩住 mirror，不再改；0=可能仅 fallback，允许后续 xcm 覆盖 */
#if !LT_MAP_DETECT_USE_ABS_XCM
static u8 s_map_x_ref_armed;
static float s_map_x_ref;
#endif
#endif

static void LineTrack_MapDetect_Reset(void)
{
#if LT_MAP_MIRROR_FORCE != 0
	s_map_mirror = (s8)LT_MAP_MIRROR_FORCE;
#if LT_MAP_DETECT_ENABLE
	s_map_detect_locked = 1u;
	s_map_odom_latched = 1u;
#if !LT_MAP_DETECT_USE_ABS_XCM
	s_map_x_ref_armed = 0u;
	s_map_x_ref = 0.0f;
#endif
#endif
#else
	s_map_mirror = (s8)LT_MAP_DETECT_FALLBACK_MIRROR;
#if LT_MAP_DETECT_ENABLE
	s_map_detect_locked = 0u;
	s_map_odom_latched = 0u;
#if !LT_MAP_DETECT_USE_ABS_XCM
	s_map_x_ref_armed = 0u;
	s_map_x_ref = 0.0f;
#endif
#endif
#endif
}

static void LineTrack_MapDetect_Tick(void)
{
#if LT_MAP_MIRROR_FORCE != 0
	return;
#elif !LT_MAP_DETECT_ENABLE
	return;
#else
	if (s_map_odom_latched)
		return;
	if (!Odom_IsValid())
		return;

#if LT_MAP_DETECT_USE_ABS_XCM
	{
		float xm;
		float th;

		xm = NavOdom_GetX();
		th = LT_MAP_DETECT_ABS_X_M;
		if (xm > th) {
			s_map_mirror = 1;
			s_map_detect_locked = 1u;
			s_map_odom_latched = 1u;
			return;
		}
		if (xm < -th) {
			s_map_mirror = -1;
			s_map_detect_locked = 1u;
			s_map_odom_latched = 1u;
			return;
		}
	}
#else
	{
		float d;
		float x;
		float dx;

		d = NavOdom_GetTotalDistanceM();
		if (d < NavRoute_PhysicalMToOdomTotalM(LT_MAP_DETECT_ARM_DIST_M))
			return;
		if (!s_map_x_ref_armed) {
			s_map_x_ref = NavOdom_GetX();
			s_map_x_ref_armed = 1;
		}
		x = NavOdom_GetX();
		dx = x - s_map_x_ref;
		if (dx > NavRoute_PhysicalMToOdomTotalM(LT_MAP_DETECT_DX_TH)) {
			s_map_mirror = 1;
			s_map_detect_locked = 1;
			s_map_odom_latched = 1u;
			return;
		}
		if (dx < -NavRoute_PhysicalMToOdomTotalM(LT_MAP_DETECT_DX_TH)) {
			s_map_mirror = -1;
			s_map_detect_locked = 1;
			s_map_odom_latched = 1u;
			return;
		}
		if (s_map_detect_locked)
			return;
		if (d >= NavRoute_PhysicalMToOdomTotalM(LT_MAP_DETECT_FALLBACK_DIST_M)) {
			s_map_mirror = (s8)LT_MAP_DETECT_FALLBACK_MIRROR;
			s_map_detect_locked = 1;
		}
	}
#endif
#endif
}

static u8 LineTrack_InMazeZone(void)
{
	float d;
	float d0, d1;

	if (!Odom_IsValid())
		return 0u;
	d = NavOdom_GetTotalDistanceM();

	d0 = LINE_TRACK_MAZE_DIST0_M;
	d1 = LINE_TRACK_MAZE_DIST1_M;
	if (d1 > d0 && d >= d0 && d <= d1)
		return 1u;

	d0 = LINE_TRACK_MAZE2_DIST0_M;
	d1 = LINE_TRACK_MAZE2_DIST1_M;
	if (d1 > d0 && d >= d0 && d <= d1)
		return 1u;

	return 0u;
}

/*
 * 迷宫光电段：自 660 左拐后下一节点为出迷宫点前起，至 MAZE_OPT_DONE（含尾段盲走 LT_MAZE_TAIL_FWD_AFTER_LAT_M）
 * 右横移结束后累计直行 ≥ LT_MAZE_POST_RLAT_STRAIGHT_M（仅 ODOM，不用十字路口计数）即进入第 4 段左横移前判定。
 * 阶段/分支判定：光电状态机；不触发下一 nav_route 里程点；循迹丢线 vz 不用 ODOM 偏航。
 * 另：进迷宫「右横移前盲走」记 s_maze_mirror_pre_fwd_m/ms，出迷宫「左横移前盲走」在 LT_MAZE_MIRROR_PRE_FWD 下镜像；
 *     右横移/左横移历时见 s_maze_lat_saved_ms（LT_MAZE_TAIL_LAT_TIME_ONLY），与 HoldsOdomExitNode 不冲突。
 */
static u8 LineTrack_MazeOptical_HoldsOdomExitNode(void)
{
	if (!Odom_IsValid())
		return 0u;
	if (NavRoute_GetNextIndex() != LT_NAV_ROUTE_IDX_MAZE_EXIT_RIGHT)
		return 0u;
	if (NavOdom_GetTotalDistanceM()
	    < NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT))
		return 0u;
	if (s_maze_opt_phase == MAZE_OPT_DONE)
		return 0u;
	return 1u;
}

/* 真“横线感”路口：不要全黑(穿心竖线)、不要全白 */
static u8 LtJunc_LikelyLocal(u8 pattern, u8 black_cnt)
{
	if (pattern == LINE_LOST || pattern == 0x00u)
		return 0u;
	if (black_cnt < LT_JUNC_BLACK_MIN || black_cnt > 7u)
		return 0u;
	return 1u;
}

/* T 字路口：8 路全黑（pattern==0） */
static u8 LtRoute_TJuncWideStrict(u8 pattern)
{
	return (pattern == 0x00u) ? 1u : 0u;
}

/* 迷宫入口：L1~L3 见黑，且非全黑 T 字 */
static u8 LtRoute_MazeEntryL123Ok(u8 pattern)
{
	if (pattern == 0x00u)
		return 0u;
	if ((pattern & 0xE0u) != 0u)
		return 0u;
	return 1u;
}

/* 迷宫尾 L1~L5 形态：用于里程虚拟节点引导等（全赛道）；非迷宫第4步专用触发 */
static u8 LtRoute_MazeTailL12345Ok(u8 pattern)
{
	if (pattern == 0x00u)
		return 0u;
	if ((pattern & 0xF8u) != 0u)
		return 0u;
	return 1u;
}

/* 尾段第4步：L1~L4 见黑（bit7~4=0），用于稳定后再左横移 */
static u8 LtRoute_MazeL1234AllBlackOk(u8 pattern)
{
	if (pattern == LINE_LOST)
		return 0u;
	if ((pattern & 0xF0u) != 0u)
		return 0u;
	return 1u;
}

/* 入口 T：严 0x00 或 black_cnt≥7（近全黑） */
static u8 LtMaze_EntryTJuncOk(u8 pattern, u8 black_cnt)
{
	if (LtRoute_TJuncWideStrict(pattern))
		return 1u;
	if (pattern == LINE_LOST)
		return 0u;
	return (black_cnt >= LT_MAZE_T_LOOSE_MIN_BLACK) ? 1u : 0u;
}

/* 尾段 L1234：严四路黑或 L1~L4 至少 LT_MAZE_L1234_LOOSE_MIN_BLACK 路黑 */
static u8 LtMaze_TailL1234Ok(u8 pattern)
{
	u8 n;

#if LT_MAZE_TAIL_EXIT_L12_IMMEDIATE != 0
	/* L1(bit7) 见黑(0)即触发（正图左出迷宫，单侧 L1 也可判过） */
	if (pattern != LINE_LOST && (pattern & 0x80u) == 0u)
		return 1u;
#endif
	if (LtRoute_MazeL1234AllBlackOk(pattern))
		return 1u;
	if (pattern == LINE_LOST)
		return 0u;
	n = 0u;
	if ((pattern & 0x80u) == 0u) n++;
	if ((pattern & 0x40u) == 0u) n++;
	if ((pattern & 0x20u) == 0u) n++;
	if ((pattern & 0x10u) == 0u) n++;
	return (n >= LT_MAZE_L1234_LOOSE_MIN_BLACK) ? 1u : 0u;
}

/* 镜像场尾段：左右对调判 L5~L8（非与正场共用同一套阈值时可再拆宏） */
static u8 LtRoute_MazeR5678AllBlackOk(u8 pattern)
{
	if (pattern == LINE_LOST)
		return 0u;
	if ((pattern & 0x0Fu) != 0u)
		return 0u;
	return 1u;
}

static u8 LtMaze_TailR5678Ok(u8 pattern)
{
	u8 n;

#if LT_MAZE_TAIL_EXIT_L12_IMMEDIATE != 0
	/* 镜像场：L8(bit0) 见黑(0)即触发 */
	if (pattern != LINE_LOST && (pattern & 0x01u) == 0u)
		return 1u;
#endif
	if (LtRoute_MazeR5678AllBlackOk(pattern))
		return 1u;
	if (pattern == LINE_LOST)
		return 0u;
	n = 0u;
	if ((pattern & 0x08u) == 0u) n++;
	if ((pattern & 0x04u) == 0u) n++;
	if ((pattern & 0x02u) == 0u) n++;
	if ((pattern & 0x01u) == 0u) n++;
	return (n >= LT_MAZE_L1234_LOOSE_MIN_BLACK) ? 1u : 0u;
}

/* 第2步右横移结束（宽松）：仅 L6~L8 同时为白(1)；须先经历全黑阶段（见 Process 中 s_maze_t_seen_00_in_lat） */
static u8 LtRoute_MazeL678AllWhiteOk(u8 pattern)
{
	if (pattern == LINE_LOST)
		return 0u;
	return ((pattern & 0x07u) == 0x07u) ? 1u : 0u;
}

/*
 * 迷宫尾出弯（出口几何，供 L78 快判 / 虚拟右转）：右侧见线 + 左侧偏白。
 * L6 不参与。宏 LT_MAZE_TAIL_L78_RIGHT_OR_ONE_BLACK / LT_MAZE_TAIL_L78_LOOSE_L5 控制激进程度。
 */
static u8 LtRoute_MazeTailL78OnlyOk(u8 pattern)
{
	if (pattern == LINE_LOST || pattern == 0x00u)
		return 0u;
#if LT_MAZE_TAIL_L78_RIGHT_OR_ONE_BLACK != 0
	/* L7、L8 至少一路黑（非双白） */
	if ((pattern & 0x03u) == 0x03u)
		return 0u;
#else
	if ((pattern & 0x03u) != 0u)
		return 0u;
#endif
#if LT_MAZE_TAIL_L78_LOOSE_L5 != 0
	if ((pattern & 0xF0u) != 0xF0u)
		return 0u;
#else
	if ((pattern & 0xF8u) != 0xF8u)
		return 0u;
#endif
	return 1u;
}

/* idx3(10.10m) 后窗口内：L6~L8 全黑且非全黑路口 → 判右转直角；镜像场用 L1~L3 全黑 → 左转直角 */
static u8 LtPost950_PatRight678(u8 pattern)
{
	if (pattern == LINE_LOST || pattern == 0x00u)
		return 0u;
	if ((pattern & 0x07u) != 0u)
		return 0u;
	if ((pattern & 0xF8u) == 0u)
		return 0u;
	return 1u;
}

static u8 LtPost950_PatLeft123(u8 pattern)
{
	if (pattern == LINE_LOST || pattern == 0x00u)
		return 0u;
	if ((pattern & 0xE0u) != 0u)
		return 0u;
	if ((pattern & 0x1Fu) == 0u)
		return 0u;
	return 1u;
}

#if LT_POST_950_CORNER_GUIDE_ENABLE != 0u
static u8 LtPost950_InOdomWindow(void)
{
	u8 nx;

	if (!Odom_IsValid())
		return 0u;
	if (NavOdom_GetTotalDistanceM() < NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_MAZE_EXIT_RIGHT))
		return 0u;
	nx = NavRoute_GetNextIndex();
	if (nx < LT_NAV_ROUTE_IDX_CIRCLE_OUTER_ARC)
		return 0u;
	if (nx >= LT_NAV_ROUTE_IDX_FINISH_APPROACH)
		return 0u;
	return 1u;
}
#endif /* LT_POST_950_CORNER_GUIDE_ENABLE */

/* idx4 触发后～idx5 触发前：出迷宫后入 8 字前的分地图弯段 */
static u8 LtIdx45_InCurveWindow(void)
{
	float total;
	float t4;
	float t5;
	float lead;

	if (!Odom_IsValid())
		return 0u;
	if (LineTrack_InMazeZone() || LineTrack_MazeOptical_HoldsOdomExitNode())
		return 0u;
	if (s_maze_opt_phase == MAZE_OPT_BLIND_RUN)
		return 0u;
	lead = NavRoute_PhysicalMToOdomTotalM(ROUTE_TRIGGER_LEAD_M);
	total = NavOdom_GetTotalDistanceM();
	t4 = NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_MAZE_EXIT_RIGHT);
	t5 = NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_CIRCLE_OUTER_ARC);
	if (total < t4 - lead)
		return 0u;
	if (total >= t5 - lead)
		return 0u;
	return 1u;
}

#if LT_IDX45_CURVE_GUIDE_ENABLE != 0
static void LineTrack_ApplyIdx45CurveAssist(float *err_io)
{
	float ge;
	float t;

	if (err_io == NULL || !LtIdx45_InCurveWindow())
		return;
	if (s_route_guide_ticks > 0u)
		return;
	ge = ROUTE_GUIDE_ERR * LT_IDX45_CURVE_GUIDE_SOFT_K;
	t = LT_IDX45_CURVE_GUIDE_BLEND;
	/* 右图(mirror<0)左拐；左图(mirror>=0)右拐 */
	if (s_map_mirror >= 0) {
		ge *= LT_ROUTE_GUIDE_WIDE_R_K;
		if (*err_io > -ge)
			*err_io = *err_io + (-ge - *err_io) * t;
	} else {
		ge *= LT_ROUTE_GUIDE_LEFT650_K;
		if (*err_io < ge)
			*err_io = *err_io + (ge - *err_io) * t;
	}
}
#endif /* LT_IDX45_CURVE_GUIDE_ENABLE */

static u8 LtArcKeep_InSegment(float total_m)
{
	float t0;
	float span_odom;

	if (!Odom_IsValid())
		return 0u;
	t0 = NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_ARC_KEEP_START);
	if (total_m < t0)
		return 0u;
	span_odom = NavRoute_PhysicalMToOdomTotalM(LT_ARC_KEEP_SEGMENT_LEN_M);
	if (total_m >= t0 + span_odom)
		return 0u;
	return 1u;
}

/* 弧外侧（易误入另一侧直道）：非镜像右外 L678 全白；镜像左外 L123 全白 */
static u8 LtArcKeep_OutsideWhite678(u8 pattern)
{
	if (pattern == LINE_LOST || pattern == 0x00u)
		return 0u;
	return ((pattern & 0x07u) == 0x07u) ? 1u : 0u;
}

static u8 LtArcKeep_OutsideWhite123(u8 pattern)
{
	if (pattern == LINE_LOST || pattern == 0x00u)
		return 0u;
	return ((pattern & 0xE0u) == 0xE0u) ? 1u : 0u;
}

#if LT_RADAR_BRANCH_ENABLE
static void LtFsm_ResetCorner(void);

static void LtNavIdx9_GetWindow(float *t_start, float *t_end, float *t9_table)
{
	float lead;

	if (t_end)
		*t_end = NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT);
	if (t9_table)
		*t9_table = NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_RADAR_POST_HINT);
	if (t_start) {
#if LT_NAV_IDX9_WINDOW_USE_LEAD
		lead = NavRoute_PhysicalMToOdomTotalM(ROUTE_TRIGGER_LEAD_M);
		*t_start = NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_RADAR_POST_HINT) - lead;
#else
		*t_start = NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_RADAR_POST_HINT);
#endif
	}
}

static u8 LtNavIdx9_InWindow(float total_m)
{
	float ts, te;

	if (!Odom_IsValid())
		return 0u;
	LtNavIdx9_GetWindow(&ts, &te, NULL);
	return (total_m >= ts && total_m < te) ? 1u : 0u;
}

/* idx10 盲弧：idx9 窗内须先完成/进行出口盲转，避免 19.15m 抢跑清掉 idx9 状态 */
static u8 LtNavIdx9_BlindArcAllowed(void)
{
	if (!Odom_IsValid())
		return 1u;
	if (!LtNavIdx9_InWindow(NavOdom_GetTotalDistanceM()))
		return 1u;
	if (s_nav_idx9_exit_rot_done != 0u)
		return 1u;
	if (s_nav_idx9_exit_rot_wait != 0u)
		return 1u;
	if (s_nav_idx9_post_rot_blind_fwd_active != 0u)
		return 1u;
	return 0u;
}

static float LtNavIdx9_PickRzDeg(u8 use_left)
{
	return use_left ? (-LT_NAV_IDX9_EXIT_SENDROT_DEG) : (LT_NAV_IDX9_EXIT_SENDROT_DEG);
}

static u8 LtNavIdx9_TrySendrot(float rz)
{
	if (!sendrot_AsyncBegin(0.0f, 0.0f, rz, LT_NAV_IDX9_EXIT_SENDROT_OMEGA))
		return 0u;
	s_nav_idx9_exit_rot_motion_start = HAL_GetTick();
	s_nav_idx9_exit_rot_wait = 1u;
	s_nav_idx9_exit_rot_edge_streak = 0u;
	return 1u;
}

static u8 LtNavIdx9_ForcedRz(float *rz_out)
{
	if (rz_out == NULL)
		return 0u;
	if (s_lt_radar_pick_valid) {
		*rz_out = LtNavIdx9_PickRzDeg(s_lt_radar_go_left != 0u);
		return 1u;
	}
	/* 无雷达选边：与 idx10 盲弧地图约定一致（左图 rz 负 / 右图 rz 正） */
	*rz_out = (s_map_mirror >= 0)
	    ? (-LT_NAV_IDX9_EXIT_SENDROT_DEG)
	    : (LT_NAV_IDX9_EXIT_SENDROT_DEG);
	return 1u;
}

/**
 * [idx9,idx10)：口字出口盲转。
 * - sendrot_AsyncBegin(0,0,rz, LT_NAV_IDX9_EXIT_SENDROT_OMEGA)，与雷达 LT_RADAR_BRANCH_ROT 相同；
 *   等待期间返回 1 → Step 中 goto line_track_step_led，避免周期发巡线速打断旋转。
 * - DF_RotationAsyncTryConsumeDone 为真后：MarkSegmentStart + 盲直 LT_NAV_IDX9_POST_ROT_BLIND_FWD_M，再正常 PID。
 * - 方向：雷达已选边则跟雷达，光电仅门控；否则纯光电（双缘优先左）。rz 左负右正。
 * - 里程已超过 idx10 仍 s_nav_idx9_exit_rot_wait 时：不清状态，直至 TryConsumeDone（与 Step 里延后 idx10 盲弧一致）。
 */
static u8 LineTrack_NavIdx9ExitRot_Tick(u8 pattern, u8 black_cnt, u8 had_new)
{
	float total;
	float t_start, t_end, t9;
	float forced_after;

	(void)black_cnt;
	if (!Odom_IsValid())
		return 0u;
	total = NavOdom_GetTotalDistanceM();
	LtNavIdx9_GetWindow(&t_start, &t_end, &t9);
	forced_after = NavRoute_PhysicalMToOdomTotalM(LT_NAV_IDX9_EXIT_FORCED_AFTER_M);
	if (total < t_start) {
		s_nav_idx9_exit_rot_done = 0u;
		s_nav_idx9_exit_rot_edge_streak = 0u;
		s_nav_idx9_sendrot_win_int_cleared = 0u;
		s_nav_idx9_exit_rot_wait = 0u;
		s_nav_idx9_exit_rot_motion_start = 0u;
		s_nav_idx9_post_rot_blind_fwd_active = 0u;
		return 0u;
	}
	/* 出 idx9～idx10 窗：仅当未在等 sendrot 时清状态（否则与 idx10 盲弧并发会打断旋转） */
	if (total >= t_end && s_nav_idx9_exit_rot_wait == 0u) {
		s_nav_idx9_exit_rot_done = 0u;
		s_nav_idx9_exit_rot_edge_streak = 0u;
		s_nav_idx9_sendrot_win_int_cleared = 0u;
		s_nav_idx9_exit_rot_motion_start = 0u;
		s_nav_idx9_post_rot_blind_fwd_active = 0u;
		return 0u;
	}
	if (LineTrack_InMazeZone() || LineTrack_MazeOptical_HoldsOdomExitNode())
		return 0u;
	if (s_lt_radar_phase != LT_RADAR_OFF)
		return 0u;
	if (s_nav_idx9_sendrot_win_int_cleared == 0u) {
		s_nav_idx9_sendrot_win_int_cleared = 1u;
		s_err_integral = 0.0f;
		LtFsm_ResetCorner();
	}
	if (s_nav_idx9_exit_rot_wait != 0u) {
		if (DF_RotationAsyncTryConsumeDone(s_nav_idx9_exit_rot_motion_start)) {
			NavOdom_MarkSegmentStart();
			s_nav_idx9_post_rot_blind_fwd_active = 1u;
			s_nav_idx9_exit_rot_done = 1u;
			s_nav_idx9_exit_rot_wait = 0u;
			s_err_integral = 0.0f;
			s_d_filtered = 0.0f;
			s_nav_idx9_exit_rot_edge_streak = 0u;
			return 0u; /* 本周期继续走下方盲直 */
		}
		return 1u;
	}
	if (s_nav_idx9_exit_rot_done != 0u)
		return 0u;

	/* 窗内里程兜底：光电一直未判到缘时也执行出口盲转 */
	if (total >= (t9 + forced_after)) {
		float rz_f;

		if (LtNavIdx9_ForcedRz(&rz_f) != 0u && LtNavIdx9_TrySendrot(rz_f) != 0u)
			return 1u;
	}

	if (had_new == 0u)
		return 0u;
	/* 仅丢线（全白）清 streak；全黑 0x00 允许触发，避免交口宽黑带永远不 sendrot */
	if (pattern == LINE_LOST) {
		s_nav_idx9_exit_rot_edge_streak = 0u;
		return 0u;
	}
	{
		/* 0=黑：左缘=L1(bit7) 或 L2(bit6) 任一路黑；右缘=L8(bit0) 或 L7(bit1) 任一路黑 */
		u8 edge_l = (u8)((((pattern & 0x80u) == 0u) || ((pattern & 0x40u) == 0u)) ? 1u : 0u);
		u8 edge_r = (u8)((((pattern & 0x01u) == 0u) || ((pattern & 0x02u) == 0u)) ? 1u : 0u);
		float rz;

		if (s_lt_radar_pick_valid) {
			if (pattern == 0x00u) {
				rz = LtNavIdx9_PickRzDeg(s_lt_radar_go_left != 0u);
			} else if (s_lt_radar_go_left != 0u) {
				if (edge_l == 0u) {
					s_nav_idx9_exit_rot_edge_streak = 0u;
					return 0u;
				}
				rz = -LT_NAV_IDX9_EXIT_SENDROT_DEG;
			} else {
				if (edge_r == 0u) {
					s_nav_idx9_exit_rot_edge_streak = 0u;
					return 0u;
				}
				rz = LT_NAV_IDX9_EXIT_SENDROT_DEG;
			}
		} else {
			if (pattern == 0x00u) {
				if (LtNavIdx9_ForcedRz(&rz) == 0u)
					return 0u;
			} else if (edge_l == 0u && edge_r == 0u) {
				s_nav_idx9_exit_rot_edge_streak = 0u;
				return 0u;
			} else {
				if (edge_l != 0u && edge_r != 0u)
					edge_r = 0u;
				rz = (edge_l != 0u) ? (-LT_NAV_IDX9_EXIT_SENDROT_DEG) : (LT_NAV_IDX9_EXIT_SENDROT_DEG);
			}
		}
		if (s_nav_idx9_exit_rot_edge_streak < 255u)
			s_nav_idx9_exit_rot_edge_streak++;
		if (s_nav_idx9_exit_rot_edge_streak < LT_NAV_IDX9_EXIT_SENDROT_EDGE_STREAK)
			return 0u;
		if (LtNavIdx9_TrySendrot(rz) == 0u)
			return 0u; /* 下发失败：保留 streak，下周期重试 */
		return 1u;
	}
}
#endif /* LT_RADAR_BRANCH_ENABLE */

static u8 LtRoute_RightWideBoostOk(u8 route_idx, u8 pattern)
{
	if (route_idx == LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX)
		return LtPost950_PatRight678(pattern);
	if (route_idx == LT_MAZE_VIRT_ENTRY_T_RIGHT_IDX)
		return LtRoute_TJuncWideStrict(pattern);
	if (route_idx == LT_MAZE_VIRT_RIGHT_IDX)
		return LtRoute_MazeTailL78OnlyOk(pattern);
	return 0u;
}

/* 迷宫光电虚拟节点（无里程迷宫段） */
static u8 LtRoute_IsMazeSectionIdx(u8 route_idx)
{
	if (route_idx == LT_MAZE_VIRT_LEFT_IDX || route_idx == LT_MAZE_VIRT_RIGHT_IDX)
		return 1u;
	if (route_idx == LT_MAZE_VIRT_ENTRY_T_RIGHT_IDX || route_idx == LT_MAZE_VIRT_ENTRY_L123_LEFT_IDX)
		return 1u;
	return 0u;
}

static void LineTrack_MazeOptical_Reset(void)
{
	s_maze_opt_phase = MAZE_OPT_IDLE;
	s_maze_entry_t_streak = 0;
	s_maze_opt_cd_until = 0u;
	s_maze_opt_arm_until = 0u;
	s_maze_blind_sub = 0u;
	s_maze_blind_next_phase = 0u;
	s_maze_lat_saved_m = 0.0f;
	s_maze_lat_saved_ms = 0u;
	s_maze_entry_lat_start_tick = 0u;
	s_maze_tail_lat_start_tick = 0u;
	s_maze_mirror_pre_fwd_m = 0.0f;
	s_maze_mirror_pre_fwd_ms = 0u;
	s_maze_entry_pre_fwd_start_tick = 0u;
	s_maze_tail_pre_fwd_start_tick = 0u;
	s_maze_t_exit_line_streak = 0u;
	s_maze_t_fullblack_streak = 0u;
	s_maze_t_seen_00_in_lat = 0u;
	s_maze_tail_l1234_streak = 0u;
	s_maze_tail_l1234_enter_tick = 0u;
	s_maze_entry_t_score = 0u;
	s_maze_tail_l1234_score = 0u;
	s_maze_t_pre_score = 0u;
	s_maze_tail_l78_exit_streak = 0u;
}

static void LineTrack_MazeOptical_ArmCooldownMs(u32 now_ms, u32 dur_ms)
{
	s_maze_opt_cd_until = now_ms + dur_ms;
}

static void LineTrack_MazeOptical_ArmCooldown(u32 now_ms)
{
	LineTrack_MazeOptical_ArmCooldownMs(now_ms, (u32)LT_MAZE_OPTICAL_COOLDOWN_MS);
}

static void LineTrack_SendVelFull(float vx, float vy, float vz)
{
	/* 全局硬限速：巡线/引导/迷宫各路径下发前统一约束 |vy|<=10 */
	if (vy > 10.0f)
		vy = 10.0f;
	else if (vy < -10.0f)
		vy = -10.0f;
	s_last_send_vx = vx;
	s_last_send_vy = vy;
	s_last_send_vz = vz;
#if LINE_TRACK_SENDVEL_PERIOD_MS > 0
	{
		static u32 s_last_sendvel_ms;
		u32 now = HAL_GetTick();
		if (s_last_sendvel_ms != 0u) {
			u32 elapsed = now - s_last_sendvel_ms;
			if (elapsed < (u32)LINE_TRACK_SENDVEL_PERIOD_MS)
				return;
		}
		s_last_sendvel_ms = now;
	}
#endif
	sendVel_NoWait(vx, vy, vz);
}

static void LineTrack_MazeEntryLateralR_Start(void)
{
	s_maze_opt_phase = MAZE_OPT_BLIND_RUN;
	s_maze_blind_sub = MAZE_BLIND_SUB_T_PRE_STABILIZE;
	s_maze_blind_next_phase = MAZE_OPT_TAIL_COUNT_CROSS;
	s_maze_t_exit_line_streak = 0u;
	s_maze_t_fullblack_streak = 0u;
	s_maze_t_seen_00_in_lat = 0u;
	s_maze_t_pre_score = 0u;
}

static void LineTrack_MazeTailMove_Start(u32 now_ms)
{
	if (s_maze_lat_saved_m < NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_LAT_MIN_M))
		s_maze_lat_saved_m = NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_LAT_MIN_M);
	s_maze_opt_phase = MAZE_OPT_BLIND_RUN;
	s_maze_blind_sub = MAZE_BLIND_SUB_TAIL_PRE_FWD;
	s_maze_blind_next_phase = MAZE_OPT_DONE;
	s_maze_opt_cd_until = now_ms;
	if (Odom_IsValid())
		NavOdom_MarkSegmentStart();
	s_maze_tail_pre_fwd_start_tick = now_ms;
}

static void LineTrack_MazeBlind_OnComplete(void)
{
	s_maze_blind_sub = 0u;
	s_maze_opt_phase = s_maze_blind_next_phase;
	if (s_maze_opt_phase == MAZE_OPT_TAIL_COUNT_CROSS && Odom_IsValid()) {
		NavOdom_MarkSegmentStart(); /* 自此累计直行，满 LT_MAZE_POST_RLAT_STRAIGHT_M 后进第 4 段 */
		s_maze_tail_l78_exit_streak = 0u;
	}
	if (s_maze_opt_phase == MAZE_OPT_DONE)
		LineTrack_MazeOptical_ArmCooldown(HAL_GetTick());
}

/*
 * 迷宫横移/尾段：本周期已发 sendVel，返回 1 表示主循环不再跑 PID。
 * 右横移(ENTRY_LAT_R)用 ODOM 累计弧长写入 s_maze_lat_saved_m，并记历时 s_maze_lat_saved_ms；
 * 左横移(TAIL_LAT_L)：LT_MAZE_TAIL_LAT_TIME_ONLY 时仅历时≥右横移历时；否则 ODOM 或历时先到。
 * 右横移前盲走(ENTRY_PRE_FWD)结束时写入 s_maze_mirror_pre_fwd_m/ms；左横移前盲走(TAIL_PRE_FWD)在 LT_MAZE_MIRROR_PRE_FWD 下镜像。
 */
static u8 LineTrack_MazeBlind_Process(u8 pattern, u8 black_cnt)
{
	float delta;
	float vy;

	if (s_maze_opt_phase != MAZE_OPT_BLIND_RUN)
		return 0u;

	if (s_maze_blind_sub == MAZE_BLIND_SUB_T_PRE_STABILIZE) {
		if (LtMaze_EntryTJuncOk(pattern, black_cnt)) {
			if (s_maze_t_fullblack_streak < 255u)
				s_maze_t_fullblack_streak++;
			if (s_maze_t_pre_score <= 255u - LT_MAZE_T_ENTRY_SCORE_ADD)
				s_maze_t_pre_score += LT_MAZE_T_ENTRY_SCORE_ADD;
			else
				s_maze_t_pre_score = 255u;
		} else {
			s_maze_t_fullblack_streak = 0u;
			if (LT_MAZE_T_ENTRY_SCORE_SUB != 0u) {
				if (s_maze_t_pre_score > LT_MAZE_T_ENTRY_SCORE_SUB)
					s_maze_t_pre_score -= LT_MAZE_T_ENTRY_SCORE_SUB;
				else
					s_maze_t_pre_score = 0u;
			}
		}
		if (s_maze_t_fullblack_streak >= LT_MAZE_T_PRE_FULLBLACK_STREAK
		    || s_maze_t_pre_score >= LT_MAZE_T_ENTRY_SCORE_TRIGGER) {
			s_maze_t_fullblack_streak = 0u;
			s_maze_t_pre_score = 0u;
			s_maze_t_exit_line_streak = 0u;
			s_maze_t_seen_00_in_lat = 1u;
			if (Odom_IsValid()) {
				s_maze_blind_sub = MAZE_BLIND_SUB_ENTRY_PRE_FWD;
				NavOdom_MarkSegmentStart();
				s_maze_entry_pre_fwd_start_tick = HAL_GetTick();
			} else {
				s_maze_blind_sub = MAZE_BLIND_SUB_ENTRY_LAT_R;
				s_maze_entry_lat_start_tick = HAL_GetTick();
			}
			LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
			return 1u;
		}
		if (LtMaze_EntryTJuncOk(pattern, black_cnt))
			LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
		else
			LineTrack_SendVelFull(0.0f,
				-LINE_SPEED * LT_MAZE_T_OVERSHOOT_BACK_VY_K, 0.0f);
		return 1u;
	}

	if (s_maze_blind_sub == MAZE_BLIND_SUB_ENTRY_PRE_FWD) {
		if (s_maze_entry_pre_fwd_start_tick == 0u)
			s_maze_entry_pre_fwd_start_tick = HAL_GetTick();
		vy = LINE_SPEED * LT_MAZE_BLIND_FWD_VY_K;
		LineTrack_SendVelFull(0.0f, vy, 0.0f);
		if (Odom_IsValid()) {
			delta = NavOdom_GetDeltaSinceLastMarkM();
			if (delta >= NavRoute_PhysicalMToOdomTotalM(LT_MAZE_BLIND_PRE_LAT_M)) {
				s_maze_mirror_pre_fwd_m = delta;
				if (s_maze_entry_pre_fwd_start_tick != 0u)
					s_maze_mirror_pre_fwd_ms = HAL_GetTick()
					    - s_maze_entry_pre_fwd_start_tick;
				else
					s_maze_mirror_pre_fwd_ms = 0u;
				s_maze_blind_sub = MAZE_BLIND_SUB_ENTRY_LAT_R;
				NavOdom_MarkSegmentStart(); /* 右横移段长从此计 */
				s_maze_entry_lat_start_tick = HAL_GetTick();
			}
		} else {
			s_maze_blind_sub = MAZE_BLIND_SUB_ENTRY_LAT_R;
			s_maze_entry_lat_start_tick = HAL_GetTick();
		}
		return 1u;
	}

	if (s_maze_blind_sub == MAZE_BLIND_SUB_ENTRY_LAT_R) {
		if (s_maze_entry_lat_start_tick == 0u)
			s_maze_entry_lat_start_tick = HAL_GetTick();
		if (LtMaze_EntryTJuncOk(pattern, black_cnt))
			s_maze_t_seen_00_in_lat = 1u; /* 横移中仍压在 T/近全黑上 */
		/* 右平移距离：纯 ODOM 段长，供后续左平移对称，不可删 */
		LineTrack_SendVelFull(LT_MAZE_ENTRY_VX * LT_MAZE_ENTRY_VX_SLOW_K * (float)s_map_mirror,
				0.0f, 0.0f);
		if (s_maze_t_seen_00_in_lat != 0u && LtRoute_MazeL678AllWhiteOk(pattern)) {
			if (s_maze_t_exit_line_streak < 255u)
				s_maze_t_exit_line_streak++;
		} else {
			s_maze_t_exit_line_streak = 0u;
		}
		if (Odom_IsValid()) {
			delta = NavOdom_GetDeltaSinceLastMarkM();
			if (s_maze_t_exit_line_streak >= LT_MAZE_T_LATERAL_LINE_STREAK) {
				s_maze_lat_saved_m = delta;
				if (s_maze_lat_saved_m
				    < NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_LAT_MIN_M))
					s_maze_lat_saved_m =
					    NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_LAT_MIN_M);
				if (s_maze_entry_lat_start_tick != 0u)
					s_maze_lat_saved_ms = HAL_GetTick() - s_maze_entry_lat_start_tick;
				else
					s_maze_lat_saved_ms = 0u;
				LineTrack_MazeBlind_OnComplete();
				return 1u;
			}
			if (delta >= NavRoute_PhysicalMToOdomTotalM(LT_MAZE_ENTRY_LAT_MAX_M)) {
				s_maze_lat_saved_m = delta;
				if (s_maze_lat_saved_m
				    < NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_LAT_MIN_M))
					s_maze_lat_saved_m =
					    NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_LAT_MIN_M);
				if (s_maze_entry_lat_start_tick != 0u)
					s_maze_lat_saved_ms = HAL_GetTick() - s_maze_entry_lat_start_tick;
				else
					s_maze_lat_saved_ms = 0u;
				LineTrack_MazeBlind_OnComplete();
				return 1u;
			}
		} else if (s_maze_t_exit_line_streak >= LT_MAZE_T_LATERAL_LINE_STREAK) {
			s_maze_lat_saved_m =
			    NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_LAT_MIN_M);
			if (s_maze_entry_lat_start_tick != 0u)
				s_maze_lat_saved_ms = HAL_GetTick() - s_maze_entry_lat_start_tick;
			else
				s_maze_lat_saved_ms = 0u;
			LineTrack_MazeBlind_OnComplete();
			return 1u;
		}
		return 1u;
	}

	if (s_maze_blind_sub == MAZE_BLIND_SUB_TAIL_PRE_FWD) {
		vy = LINE_SPEED * LT_MAZE_BLIND_FWD_VY_K;
		LineTrack_SendVelFull(0.0f, vy, 0.0f);
		if (Odom_IsValid()) {
			u32 elapsed;
			u8 done;

			if (s_maze_tail_pre_fwd_start_tick == 0u)
				s_maze_tail_pre_fwd_start_tick = HAL_GetTick();
			delta = NavOdom_GetDeltaSinceLastMarkM();
			elapsed = HAL_GetTick() - s_maze_tail_pre_fwd_start_tick;
			if (LT_MAZE_MIRROR_PRE_FWD != 0u && s_maze_mirror_pre_fwd_ms > 0u) {
				if (LT_MAZE_TAIL_LAT_TIME_ONLY != 0u)
					done = (elapsed >= s_maze_mirror_pre_fwd_ms) ? 1u : 0u;
				else {
					done = (delta >= s_maze_mirror_pre_fwd_m) ? 1u : 0u;
					if (done == 0u && elapsed >= s_maze_mirror_pre_fwd_ms)
						done = 1u;
				}
			} else {
				done = (delta
					>= NavRoute_PhysicalMToOdomTotalM(LT_MAZE_BLIND_PRE_LAT_M))
				    ? 1u : 0u;
			}
			if (done != 0u) {
				NavOdom_MarkSegmentStart();
				s_maze_blind_sub = MAZE_BLIND_SUB_TAIL_LAT_L;
				s_maze_tail_lat_start_tick = HAL_GetTick();
			}
		} else {
			s_maze_blind_sub = MAZE_BLIND_SUB_TAIL_LAT_L;
			s_maze_tail_lat_start_tick = HAL_GetTick();
		}
		return 1u;
	}

	if (s_maze_blind_sub == MAZE_BLIND_SUB_TAIL_LAT_L) {
		LineTrack_SendVelFull(-LT_MAZE_ENTRY_VX * LT_MAZE_ENTRY_VX_SLOW_K * (float)s_map_mirror,
				0.0f, 0.0f);
		if (Odom_IsValid()) {
			u32 elapsed;
			u8 done;

			delta = NavOdom_GetDeltaSinceLastMarkM();
			if (s_maze_tail_lat_start_tick == 0u)
				s_maze_tail_lat_start_tick = HAL_GetTick();
			elapsed = HAL_GetTick() - s_maze_tail_lat_start_tick;
			if (LT_MAZE_TAIL_LAT_TIME_ONLY != 0u && s_maze_lat_saved_ms > 0u)
				done = (elapsed >= s_maze_lat_saved_ms) ? 1u : 0u;
			else if (s_maze_lat_saved_ms > 0u) {
				done = (delta >= s_maze_lat_saved_m) ? 1u : 0u;
				if (done == 0u && elapsed >= s_maze_lat_saved_ms)
					done = 1u;
			} else {
				done = (delta >= s_maze_lat_saved_m) ? 1u : 0u;
			}
			if (done != 0u) {
				NavOdom_MarkSegmentStart();
				s_maze_blind_sub = MAZE_BLIND_SUB_TAIL_FWD;
			}
		} else {
			s_maze_blind_sub = MAZE_BLIND_SUB_TAIL_FWD;
		}
		return 1u;
	}

	if (s_maze_blind_sub == MAZE_BLIND_SUB_TAIL_FWD) {
		vy = LINE_SPEED * LT_MAZE_BLIND_FWD_VY_K;
		LineTrack_SendVelFull(0.0f, vy, 0.0f);
		if (Odom_IsValid()) {
			delta = NavOdom_GetDeltaSinceLastMarkM();
			if (delta >= NavRoute_PhysicalMToOdomTotalM(LT_MAZE_TAIL_FWD_AFTER_LAT_M)) {
				LineTrack_MazeBlind_OnComplete();
				return 1u;
			}
		} else {
			LineTrack_MazeBlind_OnComplete();
		}
		return 1u;
	}

	return 0u;
}

/* 下一里程点为 idx3(10.10m)：迷宫光电流程；正图 idx3 语义见 nav_route（右转进8字前提示线）；T → 右横移 → … → 左横移+直走 → DONE */
static void LineTrack_MazeOptical_Update(u8 pattern, u8 black_cnt)
{
	float total;
	u32 now;
	u8 next_idx;

	if (s_maze_opt_phase == MAZE_OPT_BLIND_RUN)
		return;

	if (!Odom_IsValid())
		return;

	total = NavOdom_GetTotalDistanceM();
	next_idx = NavRoute_GetNextIndex();
	/* 须与 LineTracking_Step 中顺序一致：本函数先执行，TryFire 后执行。
	 * 下一节点为 idx3：正常迷宫窗口。
	 * 下一节点仍为 idx2 但里程已过 idx2 触发线：本周期尚未 TryFire，勿 reset，否则永远进不了等 T。 */
	if (next_idx != LT_NAV_ROUTE_IDX_MAZE_EXIT_RIGHT) {
		if (next_idx != LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT
		    || total < NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT)) {
			LineTrack_MazeOptical_Reset();
			return;
		}
	}

	if (total < NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT)) {
		LineTrack_MazeOptical_Reset();
		return;
	}

	/* 勿用「距 idx3 剩余≤0」提前 return：接近 10.10m 时仍须判 T、走盲走；idx3 实际触发由 TryFire+HoldsOdom 约束 */

	now = HAL_GetTick();

	/* 尾段等 L1234：丢线由主循环倒车(2560)。ENTRY_WAIT_T/IDLE 须在 LINE_LOST 下仍跑，否则全白先进迷宫会永远不判 T、盲走不启动 */
	if (pattern == LINE_LOST && s_maze_opt_phase != MAZE_OPT_TAIL_WAIT_L1234
	    && s_maze_opt_phase != MAZE_OPT_ENTRY_WAIT_T && s_maze_opt_phase != MAZE_OPT_IDLE
	    && s_maze_opt_phase != MAZE_OPT_TAIL_COUNT_CROSS)
		return;

	/* 与 idx2 后「左拐引导」并行：入口等 T / 右横移盲走须仍能推进，否则引导未结束迷宫永远不开始 */
	if (s_route_guide_ticks > 0u && s_maze_opt_phase != MAZE_OPT_TAIL_WAIT_L1234
	    && s_maze_opt_phase != MAZE_OPT_ENTRY_WAIT_T && s_maze_opt_phase != MAZE_OPT_IDLE
	    && s_maze_opt_phase != MAZE_OPT_BLIND_RUN && s_maze_opt_phase != MAZE_OPT_TAIL_COUNT_CROSS)
		return;

	/* 冷却期内仍须等 T/进盲走：否则 MAZE_OPT_DONE 后 ArmCooldown(300ms) 会把入口整段冻死 */
	if (now < s_maze_opt_cd_until && s_maze_opt_phase != MAZE_OPT_TAIL_WAIT_L1234
	    && s_maze_opt_phase != MAZE_OPT_ENTRY_WAIT_T && s_maze_opt_phase != MAZE_OPT_IDLE)
		return;
	if (s_maze_opt_arm_until != 0u && now < s_maze_opt_arm_until)
		return;
	if (s_maze_opt_arm_until != 0u && now >= s_maze_opt_arm_until)
		s_maze_opt_arm_until = 0u;

	if (s_maze_opt_phase == MAZE_OPT_DONE)
		return;

	if (s_maze_opt_phase == MAZE_OPT_IDLE) {
		s_maze_opt_phase = MAZE_OPT_ENTRY_WAIT_T;
		s_maze_opt_arm_until = now + (u32)LT_MAZE_OPTICAL_PRE_ARM_MS;
		return;
	}

	if (s_maze_opt_phase == MAZE_OPT_ENTRY_WAIT_T) {
		if (LtMaze_EntryTJuncOk(pattern, black_cnt)) {
			if (s_maze_entry_t_streak < 255u)
				s_maze_entry_t_streak++;
			if (s_maze_entry_t_score <= 255u - LT_MAZE_T_ENTRY_SCORE_ADD)
				s_maze_entry_t_score += LT_MAZE_T_ENTRY_SCORE_ADD;
			else
				s_maze_entry_t_score = 255u;
		} else {
			s_maze_entry_t_streak = 0u;
			if (LT_MAZE_T_ENTRY_SCORE_SUB != 0u) {
				if (s_maze_entry_t_score > LT_MAZE_T_ENTRY_SCORE_SUB)
					s_maze_entry_t_score -= LT_MAZE_T_ENTRY_SCORE_SUB;
				else
					s_maze_entry_t_score = 0u;
			}
		}
		if (s_maze_entry_t_streak >= LT_MAZE_T_FULL_NEED_FRAMES
		    || s_maze_entry_t_score >= LT_MAZE_T_ENTRY_SCORE_TRIGGER) {
			s_maze_entry_t_streak = 0u;
			s_maze_entry_t_score = 0u;
			LineTrack_MazeEntryLateralR_Start();
			LineTrack_MazeOptical_ArmCooldown(now);
			s_maze_opt_cd_until = now;
		}
		return;
	}

	if (s_maze_opt_phase == MAZE_OPT_TAIL_COUNT_CROSS) {
#if LT_MAZE_TAIL_L78_FAST_BLIND_ENABLE
		/* 与下方 ODOM 门限一致：须先满 LT_MAZE_POST_RLAT_STRAIGHT_M 再允许 L78 快启，否则未直行够即尾盲→DONE→idx3/Post950 提前 */
		if (LtRoute_MazeTailL78OnlyOk(pattern)
		    && (!Odom_IsValid()
			|| NavOdom_GetDeltaSinceLastMarkM()
			    >= NavRoute_PhysicalMToOdomTotalM(LT_MAZE_POST_RLAT_STRAIGHT_M))) {
			if (s_maze_tail_l78_exit_streak < 255u)
				s_maze_tail_l78_exit_streak++;
			if (s_maze_tail_l78_exit_streak >= LT_MAZE_TAIL_L78_FAST_BLIND_FRAMES) {
				s_maze_tail_l78_exit_streak = 0u;
				LineTrack_MazeTailMove_Start(now);
				return;
			}
		} else {
			s_maze_tail_l78_exit_streak = 0u;
		}
#endif
		if (Odom_IsValid()
		    && NavOdom_GetDeltaSinceLastMarkM()
			>= NavRoute_PhysicalMToOdomTotalM(LT_MAZE_POST_RLAT_STRAIGHT_M)) {
			s_maze_tail_l78_exit_streak = 0u;
			s_maze_opt_phase = MAZE_OPT_TAIL_WAIT_L1234;
			s_maze_opt_cd_until = 0u;
			s_maze_tail_l1234_enter_tick = now;
			s_maze_tail_l1234_streak = 0u;
			s_maze_tail_l1234_score = 0u;
		}
		return;
	}

	if (s_maze_opt_phase == MAZE_OPT_TAIL_WAIT_L1234) {
#if LT_MAZE_TAIL_EXIT_L12_IMMEDIATE != 0
		/* 正图 L1=黑立即进入尾横移；镜像 L8=黑，不等待 streak/score */
		if (s_map_mirror >= 0) {
			if (pattern != LINE_LOST && (pattern & 0x80u) == 0u) {
				s_maze_tail_l1234_streak = 0u;
				s_maze_tail_l1234_score = 0u;
				LineTrack_MazeTailMove_Start(now);
				return;
			}
		} else {
			if (pattern != LINE_LOST && (pattern & 0x01u) == 0u) {
				s_maze_tail_l1234_streak = 0u;
				s_maze_tail_l1234_score = 0u;
				LineTrack_MazeTailMove_Start(now);
				return;
			}
		}
#endif
#if LT_MAZE_TAIL_L78_FAST_BLIND_ENABLE
		if (LtRoute_MazeTailL78OnlyOk(pattern)) {
			if (s_maze_tail_l78_exit_streak < 255u)
				s_maze_tail_l78_exit_streak++;
			if (s_maze_tail_l78_exit_streak >= LT_MAZE_TAIL_L78_FAST_BLIND_FRAMES) {
				s_maze_tail_l78_exit_streak = 0u;
				LineTrack_MazeTailMove_Start(now);
				return;
			}
		} else {
			s_maze_tail_l78_exit_streak = 0u;
		}
#endif
		if (s_maze_tail_l1234_enter_tick == 0u)
			s_maze_tail_l1234_enter_tick = now;
		if ((u32)(now - s_maze_tail_l1234_enter_tick) >= LT_MAZE_TAIL_L1234_FORCE_MS) {
			s_maze_tail_l1234_streak = 0u;
			s_maze_tail_l1234_score = 0u;
			LineTrack_MazeTailMove_Start(now);
			return;
		}
		if ((s_map_mirror >= 0 && LtMaze_TailL1234Ok(pattern))
		    || (s_map_mirror < 0 && LtMaze_TailR5678Ok(pattern))) {
			if (s_maze_tail_l1234_streak < 255u)
				s_maze_tail_l1234_streak++;
			if (s_maze_tail_l1234_score <= 255u - LT_MAZE_TAIL_L1234_SCORE_ADD)
				s_maze_tail_l1234_score += LT_MAZE_TAIL_L1234_SCORE_ADD;
			else
				s_maze_tail_l1234_score = 255u;
		} else {
			s_maze_tail_l1234_streak = 0u;
			if (LT_MAZE_TAIL_L1234_SCORE_SUB != 0u) {
				if (s_maze_tail_l1234_score > LT_MAZE_TAIL_L1234_SCORE_SUB)
					s_maze_tail_l1234_score -= LT_MAZE_TAIL_L1234_SCORE_SUB;
				else
					s_maze_tail_l1234_score = 0u;
			}
		}
		if (s_maze_tail_l1234_streak >= LT_MAZE_TAIL_L1234_STREAK_FRAMES
		    || s_maze_tail_l1234_score >= LT_MAZE_TAIL_L1234_SCORE_TRIGGER) {
			s_maze_tail_l1234_streak = 0u;
			s_maze_tail_l1234_score = 0u;
			LineTrack_MazeTailMove_Start(now);
		}
		return;
	}
}

static void LtFsm_ArmCooldown(void)
{
	if (Odom_IsValid())
		s_fsm_cool_until_dist = NavOdom_GetTotalDistanceM()
			+ NavRoute_PhysicalMToOdomTotalM(LT_FSM_COOLDOWN_M);
	else
		s_fsm_cool_until_dist = -1.0f;
}

static u8 LtFsm_CooldownActive(void)
{
	if (s_fsm_cool_until_dist < 0.0f || !Odom_IsValid())
		return 0u;
	return (NavOdom_GetTotalDistanceM() < s_fsm_cool_until_dist) ? 1u : 0u;
}

static void LtFsm_ResetCorner(void)
{
	s_fsm = LT_FSM_TRACKING;
	s_junc_enter_streak = 0;
	s_pre_exit_streak = 0;
	s_blind_ticks = 0;
	s_pivot_seg_timer = 0;
	s_pivot_rot_phase = 1;
	s_lost_streak = 0;
}

void LineTracking_Init(void)
{
	u8 zi;

	NavRoute_Init();
	s_lt_step_epoch_ms = 0u;
	for (zi = 0u; zi < IR_NUM; zi++)
		IR_Data_number[zi] = 1u; /* 1=白底，pattern≈LINE_LOST，避免未解析时全 0 被当成全黑 T */
	s_sensor_valid = 0;
	s_force_action_ticks = 0;
	s_route_guide_ticks = 0;
	s_route_release_cnt = 0;
	s_current_act = NAV_ROUTE_STRAIGHT;
	s_last_junction_act = NAV_ROUTE_STRAIGHT;
	s_last_route_node_idx = 255u;
	s_line_track_halted = 0u;
	s_final_stop_blind_active = 0u;
	s_last_action_tick = 0;
	s_err = 0;
	s_err_last = 0;
	s_err_integral = 0;
	s_d_filtered = 0;
	LtFsm_ResetCorner();
	s_fsm_cool_until_dist = -1.0f;
	s_err_last_seen = 0.0f;
	s_flow_hnov_streak = 0;
	s_flow_right_angle_latched = 0;
	s_fig8_branch_armed = 0u;
	s_fig8_branch_done = 0u;
	s_fig8_junc_streak = 0u;
	s_fig8_circle_outer_l12 = 0u;
#if LT_POST_950_CORNER_GUIDE_ENABLE != 0u
	s_post_950_corner_streak = 0u;
	s_post_950_forced_once = 0u;
#endif
#if LT_ROUTE_NODE_DEBUG_PAUSE_MS > 0
	s_route_debug_pause_until_ms = 0u;
#endif
	s_vz_slew_prev = 0.0f;
	s_vz_straight_lp = 0.0f;
	s_vy_smoothed = LINE_SPEED;
	s_curve_blend = 0.0f;
	s_straight_blend = 0.0f;
	s_last_send_vx = 0.0f;
	s_last_send_vy = 0.0f;
	s_last_send_vz = 0.0f;
	s_last_ir_tick_ms = 0u;
	s_prev_err_raw_sc = 0.0f;
	s_blind_arc_done = 0u;
#if LT_MAZE_YAW_CHK_ENABLE
	s_maze_chk_active = 0;
#endif
#if LT_LED_ROUTE_DEBUG
	LineTrack_LedRoute_Reset();
#endif
	LineTrack_MazeOptical_Reset();
	LineTrack_MapDetect_Reset();
#if LT_RADAR_BRANCH_ENABLE
	s_lt_radar_phase = LT_RADAR_OFF;
	s_lt_radar_t_streak = 0;
	s_lt_radar_creep_ticks = 0;
	s_lt_radar_go_left = 0u;
	s_lt_radar_pick_valid = 0u;
	s_nav_idx9_exit_rot_done = 0u;
	s_nav_idx9_exit_rot_edge_streak = 0u;
	s_nav_idx9_sendrot_win_int_cleared = 0u;
	s_nav_idx9_exit_rot_wait = 0u;
	s_nav_idx9_exit_rot_motion_start = 0u;
	s_nav_idx9_post_rot_blind_fwd_active = 0u;
	s_oled_radar_pick_deadline_ms = 0u;
	s_oled_radar_pick_is_left = 0u;
	Radar_Reset();
	radar_parsing_enabled = 0u;
#endif
}

/* ==================== Track_Err 偏差计算 ====================
 * 查找表：pattern=(L1<<7)|(L2<<6)|...|L8，0=黑线
 * 左偏(正)：线在左需左转；右偏(负)：线在右需右转
 */
/* L1..L8 与 pattern 位一致：1=该路见黑 */
static u8 LtPat_IsBlack(u8 pattern, u8 lk)
{
	return (u8)((((pattern >> (8u - lk)) & 1u) == 0u) ? 1u : 0u);
}

static float Track_ErrFromPattern(u8 pattern)
{
	switch (pattern)
	{
		/* 左偏模式：线在左，需左转，err>0（单侧越早给足偏差，圆弧上少拖到最外圈才猛打） */
		case 0xEF: return  4.2f;   /* 仅 L4：明显左偏 */
		case 0xE7: return  0.0f;   /* L4,L5 居中 */
		case 0xE3: return  0.0f;
		case 0xC7: return  0.0f;
		case 0xDF: return  3.8f;   /* 11011111 仅 L3 */
		case 0xBF: return  6.0f;   /* 10111111 仅 L2 */
		case 0x7F: return  9.2f;   /* 01111111 仅 L1：出弯拉直时常见，略增便于拉回中路 */
		case 0x00: return  0.0f;
		case 0x80: return  0.0f;
		case 0x01: return  0.0f;
		case 0xC0: return  0.0f;
		case 0x03: return  0.0f;
		case 0xCF: return  4.2f;
		case 0x9F: return  5.8f;
		case 0x3F: return  7.5f;
		case 0x1F: return  9.5f;
		case 0x0F: return 12.0f;
		case 0x07: return 15.0f;
		/* 右偏模式：线在右，err<0 */
		case 0xF7: return -4.2f;   /* 仅 L5：顺时圆弧上尽早右转 */
		case 0xF3: return -5.2f;
		case 0xF9: return -6.2f;
		case 0xFB: return -4.0f;   /* 11111011 仅 L6 */
		case 0xFD: return -6.0f;   /* 11111101 仅 L7 */
		case 0xFE: return -8.0f;   /* 11111110 仅 L8 */
		case 0xFC: return -7.5f;
		case 0xF8: return -9.5f;
		case 0xF0: return -12.0f;
		case 0xE0: return -15.0f;
		case 0xFF: return 0.0f;    /* 丢线：不继承旧 err，避免 T 字口单点误判锁死 */
		default:
			/* 单点见黑（如仅 L8）：按探头位置给明确偏置，避免 default 锁死 s_err */
			{
				u8 bc = NavSense_BlackCount(pattern);
				if (bc == 1u) {
					signed char idx = -1;
					u8 i;
					for (i = 0; i < 8u; i++) {
						if (((pattern >> (7u - i)) & 1u) == 0u) {
							idx = (signed char)i;
							break;
						}
					}
					if (idx >= 0)
						return (3.5f - (float)idx) * 5.0f; /* 左探头 err>0 左转 */
				}
			}
			return 0.0f; /* 其余稀疏/乱码：直行优先，利于穿出长方形干扰 */
	}
}

/* 8 路黑线质心：索引 0=L1..7=L8，与单点 default (3.5-idx)*5 同尺度 */
static float Track_ErrCentroid(u8 pattern)
{
	u8 i;
	float sum = 0.0f;
	u8 n = 0u;

	if (pattern == LINE_LOST)
		return 0.0f;
	for (i = 0u; i < 8u; i++) {
		if (((pattern >> (7u - i)) & 1u) == 0u) {
			sum += (float)i;
			n++;
		}
	}
	if (n == 0u)
		return 0.0f;
	return (SENSOR_CENTER - sum / (float)n) * 5.0f;
}

/* 以 L4/L5 为主：尽量两路都压黑；双黑时用 L3/L6 微调，使线对准中路（须与 pattern_pid 同源，勿混用 IR_Data_number） */
static float Track_Err_MidPairOnly_FromPattern(u8 pattern)
{
	u8 l3 = LtPat_IsBlack(pattern, 3);
	u8 l4 = LtPat_IsBlack(pattern, 4);
	u8 l5 = LtPat_IsBlack(pattern, 5);
	u8 l6 = LtPat_IsBlack(pattern, 6);

	if (l4 && l5) {
		if (l3 && !l6)
			return LINE_TRACK_MID_FINE_GAIN;
		if (l6 && !l3)
			return -LINE_TRACK_MID_FINE_GAIN;
		return 0.0f;
	}
	if (l4 && !l5)
		return LINE_TRACK_MID_PAIR_GAIN;
	if (!l4 && l5)
		return -LINE_TRACK_MID_PAIR_GAIN;
	return 0.0f;
}

/* 查表 err、中路偏置、质心 融合；质心软化 pattern 跳变 */
static float Track_ErrBlendWithMidPair(float err_table, u8 pattern, u8 black_cnt)
{
	float em = Track_Err_MidPairOnly_FromPattern(pattern);
	float w;
	float ebase;
	float ecent = Track_ErrCentroid(pattern);
	float wc;

	if (LtPat_IsBlack(pattern, 4) && LtPat_IsBlack(pattern, 5))
		w = LINE_TRACK_MID_HOLD_BLEND_BOTH;
	else
		w = LINE_TRACK_MID_HOLD_BLEND;
	/* 单点见黑（常见仅 L1 压线）：不做中路融合，否则查表 err 被压到 ~1/3 易出弯丢线 */
	if (black_cnt == 1u)
		w = 0.0f;

	if (LineTrack_InMazeZone())
		w += LINE_TRACK_MID_HOLD_MAZE_PLUS;
	if (w > LINE_TRACK_MID_HOLD_WMAX)
		w = LINE_TRACK_MID_HOLD_WMAX;
	ebase = err_table * (1.0f - w) + em * w;

	wc = LT_ERR_CENTROID_BLEND;
	if (black_cnt == 0u || black_cnt >= 7u || pattern == 0x00u)
		wc = 0.0f;
	else if (black_cnt >= 6u)
		wc *= 0.55f;

	return ebase * (1.0f - wc) + ecent * wc;
}

/*
 * 左直角：要求 **L1～L5 五路全黑**（bit7～bit3 为 0）。线在车头**左侧**铺开成「要左拐」的宽条时常满足。
 * 与「仅 L1、L2 黑」无关；仅两路黑既不是左直角也不是右直角判据。
 */
static u8 LtGeo_IsLeftRightAnglePat(u8 pattern, u8 black_cnt)
{
	if (pattern == 0x00u || pattern == LINE_LOST)
		return 0u;
	return ((pattern & 0xF8u) == 0u && black_cnt >= 5u) ? 1u : 0u;
}

/*
 * 右直角：要求 **L4～L8 五路全黑**（bit4～bit0 为 0）。线在**右侧**成直角带宽条时常满足。
 * 大左弯外弧上可能 **L4～L8 也同时压黑**，但 L1、L2 仍黑说明线还在左侧——此时不能当真右直角，
 * 故 ApplyGeoLaneShape 里在 L1+L2 仍黑时不再钳右直角 err（见下）。
 */
static u8 LtGeo_IsRightRightAnglePat(u8 pattern, u8 black_cnt)
{
	if (pattern == 0x00u || pattern == LINE_LOST)
		return 0u;
	return ((pattern & 0x1Fu) == 0u && black_cnt >= 5u) ? 1u : 0u;
}

/* 8 字顶弧过 10.80m 后：T/横线感或左直角，表示可左拐进侧向直道 */
static u8 LtRoute_Fig8BranchJunctionOk(u8 pattern, u8 black_cnt)
{
	if (pattern == LINE_LOST || pattern == 0x00u)
		return 0u;
	if (LtJunc_LikelyLocal(pattern, black_cnt))
		return 1u;
	if (LtGeo_IsLeftRightAnglePat(pattern, black_cnt))
		return 1u;
	return 0u;
}

/*
 * 以 PID 为主：在查表+中路融合后再按「线型」微调 err。
 * - 直道：L3~L5 或 L4~L6 三连黑（且非 12345/45678 整段），向 Track_Err_MidPairOnly_FromPattern 融合，牢控 L4/L5。
 * - 直角：L1~L5 全黑 → 左直角(err≥地板)；L4~L8 全黑 → 右直角(err≤-地板)。
 * - 全黑 T 字：不覆盖，交里程/宽路口逻辑。
 * - 圆弧：只要落在 345/456 三连黑段，同样走直道融合，靠中路贴弧。
 */
static void LineTrack_ApplyGeoLaneShape(float *err_io, u8 pattern, u8 black_cnt)
{
	float em;

	if (err_io == NULL || pattern == LINE_LOST)
		return;
	if (pattern == 0x00u)
		return;

#if LT_IDX45_CURVE_SUPPRESS_GEO_CORNER != 0
	if (LtIdx45_InCurveWindow())
		goto geo_straight_mid_only;
#endif

	if (LtGeo_IsLeftRightAnglePat(pattern, black_cnt)) {
		*err_io = fmaxf(*err_io, LT_GEO_CORNER_ERR_FLOOR);
		return;
	}
	if (LtGeo_IsRightRightAnglePat(pattern, black_cnt)) {
		u8 apply_right_floor = 1u;
		/* 右直角判据是 L4～L8 全黑；左弧上 L1+L2 仍黑时排除误钳。须与 pattern 一致，勿读 IR_Data_number（否则 PID 屏蔽无效） */
		if (LtPat_IsBlack(pattern, 1) && LtPat_IsBlack(pattern, 2))
			apply_right_floor = 0u;
#if LT_GEO_RIGHT_CORNER_SUPPRESS_ON_LEFT_YAW_ENABLE
		if (apply_right_floor != 0u && Odom_IsValid()) {
			float dy = NavOdom_GetLastYawDeltaDeg() * NAV_ODOM_YAW_LEFT_SIGN;
			if (dy > LT_GEO_RIGHT_CORNER_SUPPRESS_LEFT_YAW_DEG)
				apply_right_floor = 0u;
		}
#endif
		if (apply_right_floor != 0u) {
			*err_io = fminf(*err_io, -LT_GEO_CORNER_ERR_FLOOR);
			return;
		}
	}
geo_straight_mid_only:
	/* 直道段：345 或 456 三连黑，强化 L4/L5（与传入 pattern 一致） */
	if (black_cnt < 7u) {
		u8 l3 = LtPat_IsBlack(pattern, 3);
		u8 l4 = LtPat_IsBlack(pattern, 4);
		u8 l5 = LtPat_IsBlack(pattern, 5);
		u8 l6 = LtPat_IsBlack(pattern, 6);

		if ((l3 && l4 && l5)
		    || (l4 && l5 && l6)) {
			em = Track_Err_MidPairOnly_FromPattern(pattern);
			*err_io = *err_io * (1.0f - LT_GEO_STRAIGHT_MID_BLEND)
				+ em * LT_GEO_STRAIGHT_MID_BLEND;
		}
	}
}

#if LT_YAW_ALIGN_ASSIST_ENABLE
/* 用 Δyaw 估计车头是否在「扫线」：与横向 err 同量纲叠加，小权重，限幅 */
static void LineTrack_ApplyYawAlignAssist(float *err_io, u8 pattern, u8 black_cnt)
{
	float dyaw;
	float assist;
	float ae;
	float wpath;

	if (err_io == NULL)
		return;
	if (!Odom_IsValid())
		return;
	if (s_route_guide_ticks > 0)
		return;
	if (LineTrack_InMazeZone())
		return;
	if (s_fsm == LT_FSM_PRE_CORNER)
		return;

	dyaw = NavOdom_GetLastYawDeltaDeg() * NAV_ODOM_YAW_LEFT_SIGN;
	{
		float yaw_g;
		float ae0;

		yaw_g = LT_YAW_ALIGN_GAIN;
		ae0 = (*err_io >= 0.0f) ? *err_io : -*err_io;
		/* 直道 |err| 很小时削弱 yaw，避免与光电来回打架发晃 */
		if (ae0 < 0.52f)
			yaw_g *= 0.32f;
		else if (ae0 > 1.1f && !LineTrack_InMazeZone())
			yaw_g *= 1.45f; /* 弯/S 弯：跟航向，车头对切线 */
		assist = -yaw_g * dyaw;
	}
	if (assist > LT_YAW_ALIGN_ABSMAX)
		assist = LT_YAW_ALIGN_ABSMAX;
	else if (assist < -LT_YAW_ALIGN_ABSMAX)
		assist = -LT_YAW_ALIGN_ABSMAX;

	ae = (*err_io >= 0.0f) ? *err_io : -*err_io;
	if (ae > 10.0f)
		wpath = 0.30f;
	else if (ae > 6.0f)
		wpath = 0.55f;
	else if (ae > 3.5f)
		wpath = 0.78f;
	else
		wpath = 1.0f;

	if (pattern == 0x00u || black_cnt >= 7u)
		wpath *= 0.25f;

	*err_io += assist * wpath;
}
#endif /* LT_YAW_ALIGN_ASSIST_ENABLE：关闭时不生成空桩，避免 Keil「声明未引用」警告 */

/*
 * 丢线时 Track_ErrFromPattern(0xFF)=0，须靠记忆/节点/可选 ODOM。
 * 次序：节点强左/右 → |err| 大 → |err| 超过 LT_LOST_REF_SIGN_GATE 即按符号给软转向
 * →（可选）Δyaw → 最后 ref 与 0 比。切勿在 ref 仍有左右符号时优先信 Δyaw，否则弯弧丢线常反向找线。
 */
static float LineTrack_LostChooseVz(void)
{
	float cap;
	float ref;

	/* 迷宫光电流程内：仅用丢线前 err 记忆，不用里程节点/偏航 */
	if (LineTrack_MazeOptical_HoldsOdomExitNode()) {
		cap = LT_LOST_TRACK_VZ_CAP;
		ref = s_err_last_seen;
		if (ref > 0.06f)
			return -cap;
		if (ref < -0.06f)
			return cap;
		if (ref > LT_LOST_REF_SIGN_GATE)
			return -cap * LT_LOST_VZ_SOFT;
		if (ref < -LT_LOST_REF_SIGN_GATE)
			return cap * LT_LOST_VZ_SOFT;
		return 0.0f;
	}

	if (s_last_junction_act == NAV_ROUTE_LEFT)
		return -VZ_ANGLE_MAX;
	if (s_last_junction_act == NAV_ROUTE_RIGHT)
		return VZ_ANGLE_MAX;

	cap = LT_LOST_TRACK_VZ_CAP;

	ref = s_err_last_seen;
	if (ref > 0.06f)
		return -cap;
	if (ref < -0.06f)
		return cap;
	if (ref > LT_LOST_REF_SIGN_GATE)
		return -cap * LT_LOST_VZ_SOFT;
	if (ref < -LT_LOST_REF_SIGN_GATE)
		return cap * LT_LOST_VZ_SOFT;

#if LT_LOST_USE_ODOM_YAW_HINT
	if (Odom_IsValid()) {
		d = NavOdom_GetLastYawDeltaDeg() * NAV_ODOM_YAW_LEFT_SIGN;
		if (d > NAV_ODOM_YAW_DELTA_DEADZONE_DEG)
			return -cap;
		if (d < -NAV_ODOM_YAW_DELTA_DEADZONE_DEG)
			return cap;
		switch (NavOdom_GetTurnHint()) {
		case NAV_TURN_LEFT:
			return -cap * LT_LOST_VZ_SOFT;
		case NAV_TURN_RIGHT:
			return cap * LT_LOST_VZ_SOFT;
		default:
			break;
		}
	}
#endif

	if (ref > 0.0f)
		return -cap * LT_LOST_VZ_SOFT;
	if (ref < 0.0f)
		return cap * LT_LOST_VZ_SOFT;
	return 0.0f;
}

/*
 * 丢线帧：上一帧前进则倒车找线；vz 以 LostChooseVz（err/节点/Δyaw）为主。
 * 根因修复：若在丢线前 PID 已在「朝线转」的方向上（线在左 err>0 且 vz<0 等），再混入 vz_opp=-last_vz
 * 会把舵甩向背离线的一侧，表现为「线在左却右转、线在右却左转」。
 */
static void LineTrack_LostApplyOpposite(float *out_vy, float *out_vz, u8 maze_mode)
{
	float vz_base = LineTrack_LostChooseVz();
	float vy;
	float vz;
	float fwd_k = maze_mode ? LT_MAZE_LOST_FORWARD_K : LT_LOST_FORWARD_K;
	float back_k = LT_LOST_BACK_K * (maze_mode ? LT_MAZE_LOST_BACK_K_SCALE : 1.0f);
	float vz_cap = maze_mode ? LT_MAZE_VZ_ABS_MAX : LT_LOST_TRACK_VZ_CAP;
	u8 steer_toward_line;
	float err_abs_seen;

	if (s_last_send_vy > LT_LOST_VY_OPP_THRESH)
		vy = -LINE_SPEED * back_k;
	else if (s_last_send_vy < -LT_LOST_VY_OPP_THRESH)
		vy = LINE_SPEED * fwd_k * 0.48f;
	else
		vy = LINE_SPEED * fwd_k;

	/*
	 * 已在朝线转：err 与下发 vz 须异号（线在左 err>0→须 vz<0；线在右 err<0→须 vz>0）。
	 * 用乘积<0 判定，与 LINE_TRACK_INVERT_STEER_CMD 无关（s_last_send_vz 已是底盘系）。
	 */
	err_abs_seen = (s_err_last_seen >= 0.0f) ? s_err_last_seen : -s_err_last_seen;
	steer_toward_line = 0u;
	if (err_abs_seen > 0.05f && (s_err_last_seen * s_last_send_vz) < -0.02f)
		steer_toward_line = 1u;

	if (s_last_send_vz > LT_LOST_VZ_OPP_THRESH || s_last_send_vz < -LT_LOST_VZ_OPP_THRESH) {
		float vz_opp = -s_last_send_vz * LT_LOST_VZ_OPP_GAIN;

		if (steer_toward_line != 0u)
			vz = vz_base;
		else
			vz = vz_opp * LT_LOST_VZ_OPP_BLEND + vz_base * (1.0f - LT_LOST_VZ_OPP_BLEND);
	} else
		vz = vz_base;

	if (maze_mode) {
		vz *= LT_MAZE_LOST_VZ_SCALE;
		if (vz > vz_cap) vz = vz_cap;
		if (vz < -vz_cap) vz = -vz_cap;
	} else {
		if (vz > vz_cap) vz = vz_cap;
		if (vz < -vz_cap) vz = -vz_cap;
	}

	*out_vy = vy;
	*out_vz = LineTrack_SteerSignApply(vz);
}

/* ==================== PID计算（吸附效果） ====================
 * 偏离时自动拉回：err>0→左转，err<0→右转，err≈0→直行
 * 微分低通滤波：抑制噪声引起的剧烈修正，减少左右晃
 * err过零时积分清零：避免反向积分加剧 overshoot
 */
/* kp_scale/kd_scale：弯区>1 加强 P/D；Ki 不变，减轻弯中积分顶满（常见分段思路） */
static float PID_Calc(float err, float dt_s, float kp_scale, float kd_scale, float d_filt_alpha)
{
	/* 积分：err 过零时清零；dt_s 为相邻两包光电间隔，与 sendVel 更新同步 */
	if (s_err_last * err < 0) s_err_integral = 0;
	s_err_integral += err * dt_s;
	if (s_err_integral > PID_INTEGRAL_MAX)  s_err_integral = PID_INTEGRAL_MAX;
	if (s_err_integral < -PID_INTEGRAL_MAX) s_err_integral = -PID_INTEGRAL_MAX;

	/* 微分低通滤波：d_raw 突变时不会导致剧烈修正；d_filt_alpha 越小 D 越钝 */
	float d_raw = err - s_err_last; /* 移除 /PID_DT 防止短周期导致D项过大引起振荡 */
	s_d_filtered = d_filt_alpha * s_d_filtered + (1.0f - d_filt_alpha) * d_raw;
	s_err_last = err;

	return PID_KP * kp_scale * err + PID_KI * s_err_integral + PID_KD * kd_scale * s_d_filtered;
}

/* 「竖线还在」：中路 L4/L5 或 L3~L6 中带状；须与 pattern 一致（勿读 IR_Data_number，否则 PID 屏蔽无效） */
static u8 LtFlow_HasVerticalFeel(u8 pattern, u8 black_cnt)
{
	u8 n;

	if (pattern == LINE_LOST || black_cnt == 0u)
		return 0u;
	if (LtPat_IsBlack(pattern, 4) || LtPat_IsBlack(pattern, 5))
		return 1u;
	n = 0;
	if (LtPat_IsBlack(pattern, 3)) n++;
	if (LtPat_IsBlack(pattern, 4)) n++;
	if (LtPat_IsBlack(pattern, 5)) n++;
	if (LtPat_IsBlack(pattern, 6)) n++;
	return (n >= 2u) ? 1u : 0u;
}

/* 拐角状态迁移 + 流程图：横+竖→当正常巡线；横在竖无→直角记方向；丢线→近预设多等/远预设快自转 */
static void LtFsm_Update(u8 pattern, u8 black_cnt, float err_raw, u8 horiz, u8 vert)
{
	u8 junc_like = LtJunc_LikelyLocal(pattern, black_cnt);
	u8 cool = LtFsm_CooldownActive();
	u16 need_blind;

	if (s_route_guide_ticks > 0 || s_force_action_ticks > 0) {
		LtFsm_ResetCorner();
		s_flow_hnov_streak = 0;
		s_flow_right_angle_latched = 0;
		return;
	}

	if (LineTrack_InMazeZone()) {
		LtFsm_ResetCorner();
		s_flow_hnov_streak = 0;
		s_flow_right_angle_latched = 0;
		return;
	}

	if (cool)
		junc_like = 0u;

	/* 横线感 + 竖带仍在 ≈ T/十字：不锁直角，不堆 PRE */
	if (horiz && vert) {
		s_flow_hnov_streak = 0;
		s_flow_right_angle_latched = 0;
	} else if (horiz && !vert) {
		if (s_flow_hnov_streak < 255u)
			s_flow_hnov_streak++;
		if (s_flow_hnov_streak >= 2u)
			s_flow_right_angle_latched = 1;
	} else {
		s_flow_hnov_streak = 0;
	}

	if (pattern != LINE_LOST) {
		s_lost_streak = 0;
		if (!horiz && vert && black_cnt >= 1u && black_cnt <= 5u)
			s_flow_right_angle_latched = 0;
	}

	/* 丢线：竖线消失；下一节点近则多等里程预设，否则直角时快盲冲→自转 */
	if (pattern == LINE_LOST) {
#if LT_RADAR_BRANCH_ENABLE && LT_NAV_IDX9_SUPPRESS_LOST_PIVOT
		if (Odom_IsValid() && LtNavIdx9_InWindow(NavOdom_GetTotalDistanceM())
		    && s_nav_idx9_exit_rot_done == 0u && s_nav_idx9_exit_rot_wait == 0u
		    && s_lt_radar_phase == LT_RADAR_OFF) {
			s_lost_streak = 0u;
			return;
		}
#endif
		if (!cool && (s_fsm == LT_FSM_TRACKING || s_fsm == LT_FSM_PRE_CORNER)) {
			if (s_lost_streak < 255u)
				s_lost_streak++;
			need_blind = LT_LOST_TO_BLIND_FRAMES;
			if (Odom_IsValid()) {
				float rem = NavRoute_GetRemainingToNextM(NavOdom_GetTotalDistanceM(),
						ROUTE_TRIGGER_LEAD_M);
				if (rem < NavRoute_PhysicalMToOdomTotalM(LT_FLOW_PRESET_NEAR_M)
				    && rem > -0.08f)
					need_blind = LT_LOST_PRESET_PATIENCE;
				else if (s_flow_right_angle_latched
					 && rem > NavRoute_PhysicalMToOdomTotalM(LT_FLOW_FREEPIVOT_FAR_M)
					 && NavRoute_GetNextIndex() < NavRoute_GetCount())
					need_blind = LT_LOST_CORNER_FAST;
			}
			if (s_lost_streak >= need_blind) {
				s_fsm = LT_FSM_BLIND_SPRINT;
				s_blind_ticks = LT_BLIND_FRAMES;
				s_pivot_dir = (s_err_last_seen >= 0.0f) ? -1 : 1;
				s_last_action_tick = HAL_GetTick();
				s_lost_streak = 0;
				s_flow_right_angle_latched = 0;
			}
		}
		return;
	}

	switch (s_fsm) {
	case LT_FSM_TRACKING:
		if (horiz && vert) {
			s_junc_enter_streak = 0;
			break;
		}
		if (junc_like) {
			if (s_junc_enter_streak < 255u)
				s_junc_enter_streak++;
			if (s_junc_enter_streak >= LT_PRE_ENTER_FRAMES)
				s_fsm = LT_FSM_PRE_CORNER;
		} else {
			s_junc_enter_streak = 0;
		}
		break;
	case LT_FSM_PRE_CORNER:
		if (horiz && vert) {
			s_fsm = LT_FSM_TRACKING;
			s_junc_enter_streak = 0;
			s_pre_exit_streak = 0;
			break;
		}
		if (!junc_like) {
			float err_abs = (err_raw >= 0.0f) ? err_raw : -err_raw;
			if (err_abs < LT_PRE_ERR_MAX) {
				if (s_pre_exit_streak < 255u)
					s_pre_exit_streak++;
				if (s_pre_exit_streak >= LT_PRE_EXIT_FRAMES)
					LtFsm_ResetCorner();
			} else {
				s_pre_exit_streak = 0;
			}
		} else {
			s_pre_exit_streak = 0;
		}
		break;
	default:
		break;
	}
}

/* 盲冲结束进入 PIVOTING */
static void LtFsm_BlindToPivot(void)
{
	s_fsm = LT_FSM_PIVOTING;
	s_pivot_seg_timer = 0;
	s_pivot_rot_phase = 1;
	s_last_action_tick = HAL_GetTick();
}

static void LineTrack_SendVelSave(float vy, float vz)
{
	LineTrack_SendVelFull(0.0f, vy, vz);
}

#if LT_RADAR_BRANCH_ENABLE
/*
 * 选「走哪一条支路」，不是报「障碍在哪一侧」。
 * 返回 1：左支路中位距离 ≥ 右支路 → 左更空 / 右更挡 → 应进左支（串口打 PICK_LEFT）。
 * 返回 0：右支更空 → 进右支（PICK_RIGHT）。
 * 与 OXY 里 obs 一致：obs=R 表示右支回波更近(更挡)，常与 PICK_LEFT 同时出现。
 * 全 0 帧计数：一侧满 LT_RADAR_PICK_ZERO_BLOCKED_MIN 次 0 表示该侧等效「极近」(见 RADAR_OBSTACLE_ZEROS_IMPLY_BLOCKED_M)，勿再按旧逻辑「左全 0=左空」。
 */
static u8 LineTrack_RadarPickGoLeft(void)
{
	u8 lz = Radar_GetLeftZeroCount();
	u8 rz = Radar_GetRightZeroCount();

	if (lz >= LT_RADAR_PICK_ZERO_BLOCKED_MIN && rz < LT_RADAR_PICK_ZERO_BLOCKED_MIN)
		return 0u; /* 左支全 0→判挡，走右 */
	if (rz >= LT_RADAR_PICK_ZERO_BLOCKED_MIN && lz < LT_RADAR_PICK_ZERO_BLOCKED_MIN)
		return 1u; /* 右支全 0→判挡，走左 */
	if (lz >= LT_RADAR_PICK_ZERO_BLOCKED_MIN && rz >= LT_RADAR_PICK_ZERO_BLOCKED_MIN) {
		if (lz < rz)
			return 0u;
		if (rz < lz)
			return 1u;
		return (Radar_GetLeftDist() > Radar_GetRightDist()) ? 1u : 0u;
	}
	return (Radar_GetLeftDist() > Radar_GetRightDist()) ? 1u : 0u;
}
#endif

/** idx11 STOP 后盲直：发 vy、满里程则停机置 halted；返回 1 表示本步已处理须 goto line_track_step_led */
static u8 LineTrack_FinalStopBlind_Apply(void)
{
	if (s_final_stop_blind_active == 0u || !Odom_IsValid())
		return 0u;
	if (NavOdom_GetDeltaSinceLastMarkM()
	    >= NavRoute_PhysicalMToOdomTotalM(LT_FINAL_STOP_BLIND_FWD_M)) {
		s_final_stop_blind_active = 0u;
		s_line_track_halted = 1u;
		LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
	} else {
		LineTrack_SendVelSave(LINE_SPEED * LT_FINAL_STOP_BLIND_FWD_VY_K, 0.0f);
	}
	return 1u;
}

/* ==================== 主控制 ====================
 * 光电：有新包才刷新 err 并算 PID+sendVel，积分 dt 用相邻两包间隔；无新包可重发上一帧速度。
 * IR_Send_Control_Data 仍每周期调用，与主循环节拍一致。
 */
u8 LineTracking_Step(void)
{
	Odom_MainLoop();  /* 周期请求ODOM，维持Dcar回传 */
	IR_Send_Control_Data(0, 0, 1);  /* 触发传感器输出，频率由主循环决定 */

	/* ==================== ODOM 轨迹积分（nav_odom 单点维护） ==================== */
	NavOdom_UpdateStep();
#if LT_MAZE_YAW_CHK_ENABLE
	LineTrack_MazeYawChk_Process();
#endif
	LoraRaceReport_Try(NavOdom_GetTotalDistanceM());
	/* DebugOdomUart_Tick：移到 line_track_step_led 处，避免 HAL_UART_Transmit 阻塞拖慢 sendVel */

#if ODOM_TELEMETRY_ENABLE
	OdomTelemetry_Tick(NavOdom_GetTotalDistanceM());
#endif

	u8 had_new = 0;
	/* 集中声明，避免 goto line_track_step_led 跨过初始化触发 Keil #546-D */
	u8 pattern = 0;
	u8 pattern_pid = 0;
	u8 black_cnt = 0;       /* 原始光电图样计数：FSM/几何/路口与 pattern 一致 */
	u8 black_cnt_pid = 0;   /* 与 pattern_pid 一致，供 PID err 链 */
	float err_raw = 0.0f;
	float pid_out = 0.0f;
	float vz = 0.0f;
	float err_abs = 0.0f;
	float speed_factor = 1.0f;
	float vy = 0.0f;
	float derr_jump_abs = 0.0f;

	if (s_lt_step_epoch_ms == 0u)
		s_lt_step_epoch_ms = HAL_GetTick();

	if (g_ir_new_package_flag)
	{
		g_ir_new_package_flag = 0;
		if (Deal_IR_Usart_Data()) {
			had_new = 1;
#if IR_DEBUG_USART3
			{
				u8 pdbg = NavSense_BuildPattern();
				u8 bdbg = NavSense_BlackCount(pdbg);
				float e = Track_ErrBlendWithMidPair(Track_ErrFromPattern(pdbg), pdbg, bdbg);
				char buf[96];
				u8 len = (u8)sprintf(buf, "L1:%d L2:%d L3:%d L4:%d L5:%d L6:%d L7:%d L8:%d err:%.1f\r\n",
					IR_Data_number[0], IR_Data_number[1], IR_Data_number[2], IR_Data_number[3],
					IR_Data_number[4], IR_Data_number[5], IR_Data_number[6], IR_Data_number[7], (double)e);
				HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 10);
			}
#endif
		}
	}
	if (had_new)
		s_sensor_valid = 1;
	else if (!s_sensor_valid && LT_IR_BOOTSTRAP_VALID_MS > 0u
	    && (HAL_GetTick() - s_lt_step_epoch_ms) >= LT_IR_BOOTSTRAP_VALID_MS)
		s_sensor_valid = 1u;

	if (!s_sensor_valid)
	{
		s_vy_smoothed = 0.0f;
		s_curve_blend = 0.0f;
		s_straight_blend = 0.0f;
		LineTrack_SendVelSave(0.0f, 0.0f);
		goto line_track_step_led;
	}

#if LT_ROUTE_NODE_DEBUG_PAUSE_MS > 0
	if (s_route_debug_pause_until_ms != 0u) {
		u32 now_dbg = HAL_GetTick();
		if (now_dbg < s_route_debug_pause_until_ms) {
			LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
			goto line_track_step_led;
		}
		s_route_debug_pause_until_ms = 0u;
	}
#endif

#if LT_RADAR_BRANCH_ENABLE
	/* NAV_ROUTE_RADAR_ARM 武装后：等 T 字全黑 → USART4 雷达扫描 →（路表终点 idx9 STOP，雷达段后接） */
	if (s_lt_radar_phase == LT_RADAR_WAIT_T) {
		u8 pearly = NavSense_BuildPattern();
		u8 bc_radar = NavSense_BlackCount(pearly);

		/* 与 LtMaze_EntryTJuncOk 一致：严 0x00 或近全黑，避免仅严全黑时晚触发 */
		if (LtMaze_EntryTJuncOk(pearly, bc_radar) && !LineTrack_InMazeZone()) {
			if (s_lt_radar_t_streak < 255u)
				s_lt_radar_t_streak++;
			if (s_lt_radar_t_streak >= LT_RADAR_TJUNC_STREAK) {
				s_lt_radar_phase = LT_RADAR_SCANNING;
				s_lt_radar_t_streak = 0;
#if LT_RADAR_PRINT_CHOICE_LPUART1
				{
					static const char k_tj[] = "RADAR: TJUNC\r\n";
					(void)HAL_UART_Transmit(&huart3, (uint8_t *)k_tj, (uint16_t)(sizeof(k_tj) - 1u), 25);
				}
#endif
				/* 先停车；下一周期再只跑 Radar_Update（内会 sendrot）。切勿在同一周期里 sendrot 后再 sendVel(0)，
				 * 底盘多按「后到优先」，零速会冲掉旋转指令。 */
				LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
				Radar_Reset();
				Radar_StartScan();
				goto line_track_step_led;
			}
		} else
			s_lt_radar_t_streak = 0;
	} else if (s_lt_radar_phase == LT_RADAR_SCANNING) {
		/* 扫描全程只发 sendrot，不再周期发 sendVel_NoWait(0,0,0)，否则会取消旋转 */
		Radar_UpdateObstacleScan();
		if (Radar_GetScanState() == RADAR_SCAN_DONE) {
			s_lt_radar_go_left = LineTrack_RadarPickGoLeft();
			s_lt_radar_pick_valid = 1u;
			s_oled_radar_pick_is_left = s_lt_radar_go_left;
			s_oled_radar_pick_deadline_ms = HAL_GetTick() + LT_OLED_RADAR_PICK_SHOW_MS;
#if LT_RADAR_PRINT_CHOICE_LPUART1
			{
				/* USART3/LoRa：首行纯 ASCII 便于任意编码终端；次行 UTF-8 中文（VOFA+ 等选 UTF-8） */
				static const char k_radar_left[] =
					"RADAR_PICK=L\r\n"
					"\xe9\x9b\xb7\xe8\xbe\xbe:\xe9\x80\x89\xe6\x8b\xa9\xe5\xb7\xa6\xe8\xbe\xb9\r\n";
				static const char k_radar_right[] =
					"RADAR_PICK=R\r\n"
					"\xe9\x9b\xb7\xe8\xbe\xbe:\xe9\x80\x89\xe6\x8b\xa9\xe5\x8f\xb3\xe8\xbe\xb9\r\n";
				const char *msg = s_lt_radar_go_left ? k_radar_left : k_radar_right;
				u8 mlen = (u8)(s_lt_radar_go_left ? (sizeof(k_radar_left) - 1u) : (sizeof(k_radar_right) - 1u));
				(void)HAL_UART_Transmit(&huart3, (uint8_t *)msg, mlen, 30);
			}
#endif
			{
				/* 与 sendrot 底盘角约定一致；若与巡线 vz 反号仅改 LINE_TRACK_INVERT_STEER_CMD 不够时需在此改符号 */
				float rz_br = s_lt_radar_go_left ? (-LT_RADAR_BRANCH_ROT_DEG) : (LT_RADAR_BRANCH_ROT_DEG);
				(void)sendrot_AsyncBegin(0.0f, 0.0f, rz_br, 20.0f);
			}
			s_lt_radar_motion_start = HAL_GetTick();
			s_lt_radar_phase = LT_RADAR_BRANCH_ROT;
		}
		goto line_track_step_led;
	} else if (s_lt_radar_phase == LT_RADAR_BRANCH_ROT) {
		/* 等同 SCANNING：等待分支 sendrot 完成期间勿刷零速 */
		if (DF_RotationAsyncTryConsumeDone(s_lt_radar_motion_start)) {
			s_lt_radar_creep_ticks = LT_RADAR_CREEP_TICKS;
			s_lt_radar_phase = LT_RADAR_CREEP;
		}
		goto line_track_step_led;
	} else if (s_lt_radar_phase == LT_RADAR_CREEP) {
		if (s_lt_radar_creep_ticks > 0u) {
			s_lt_radar_creep_ticks--;
			LineTrack_SendVelSave(LINE_SPEED * 0.35f, 0.0f);
		} else {
			Radar_Reset();
			radar_parsing_enabled = 0u;
			s_lt_radar_phase = LT_RADAR_OFF;
		}
		goto line_track_step_led;
	}
#endif

	/* 里程 STOP 节点已触发：不再跑巡线 PID，速度保持 0（雷达等由其他任务接管） */
	if (s_line_track_halted) {
		LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
		goto line_track_step_led;
	}
	/* idx11：TryFire 当帧末尾置位；此后每周期先于 PID 盲直 */
	if (LineTrack_FinalStopBlind_Apply())
		goto line_track_step_led;

	pattern = NavSense_BuildPattern();
	pattern_pid = pattern;
	black_cnt = NavSense_BlackCount(pattern);
	black_cnt_pid = black_cnt;
	/* 须在 LINE_LOST 大分支之前：全白时该分支会 goto，若此处不跑则迷宫光电/盲走永远不推进 */
	if (Odom_IsValid())
		LineTrack_MazeOptical_Update(pattern, black_cnt);

#if LT_RADAR_BRANCH_ENABLE
	if (LineTrack_NavIdx9ExitRot_Tick(pattern, black_cnt, had_new))
		goto line_track_step_led;
	/* 口字盲转后：先按里程盲直 LT_NAV_IDX9_POST_ROT_BLIND_FWD_M，再进入下方 PID */
	if (s_nav_idx9_post_rot_blind_fwd_active && Odom_IsValid()) {
		if (NavOdom_GetDeltaSinceLastMarkM()
		    >= NavRoute_PhysicalMToOdomTotalM(LT_NAV_IDX9_POST_ROT_BLIND_FWD_M)) {
			s_nav_idx9_post_rot_blind_fwd_active = 0u;
		} else {
			LineTrack_SendVelSave(LINE_SPEED * LT_NAV_IDX9_POST_ROT_BLIND_FWD_VY_K, 0.0f);
			goto line_track_step_led;
		}
	}
#endif

	/* idx10：须在 idx9 口字盲转之后；窗内 idx9 未完成时禁止抢跑 */
	if (!s_blind_arc_done && Odom_IsValid() && !LineTrack_InMazeZone()
	    && !LineTrack_MazeOptical_HoldsOdomExitNode()
#if LT_RADAR_BRANCH_ENABLE
	    && s_lt_radar_phase == LT_RADAR_OFF
	    && s_nav_idx9_exit_rot_wait == 0u
	    && s_nav_idx9_post_rot_blind_fwd_active == 0u
	    && LtNavIdx9_BlindArcAllowed()
#endif
	    && NavOdom_GetTotalDistanceM() >= NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT)) {
		float ang_deg = (s_map_mirror >= 0)
		    ? (-LT_BLIND_ARC_ANGLE_MAG_DEG)
		    : (LT_BLIND_ARC_ANGLE_MAG_DEG);

		sendArcDisplacement(LT_BLIND_ARC_RADIUS_M, ang_deg, LT_BLIND_ARC_SPEED);
		s_blind_arc_done = 1u;
		s_err_integral = 0.0f;
		s_d_filtered = 0.0f;
#if LT_RADAR_BRANCH_ENABLE
		s_nav_idx9_post_rot_blind_fwd_active = 0u;
#endif
	}

	err_raw = Track_ErrBlendWithMidPair(Track_ErrFromPattern(pattern_pid), pattern_pid, black_cnt_pid);
	LineTrack_ApplyGeoLaneShape(&err_raw, pattern_pid, black_cnt_pid);
	LineTrack_MapDetect_Tick();
#if LT_MAP_DETECT_ENABLE && LT_MAP_DETECT_MIRROR_ERR_RAW
	if (s_map_detect_locked)
		err_raw *= (float)s_map_mirror;
#endif

	if (pattern != LINE_LOST)
		s_err_last_seen = err_raw;

	/* 迷宫内：横线感/全黑按“直行”处理，其余减弱横向 err，减少吸进两侧框线 */
	if (LineTrack_InMazeZone()) {
		if (black_cnt >= 6u || pattern == 0x00u)
			err_raw = 0.0f;
		else
			err_raw *= LT_MAZE_ERR_SCALE;
	}

	/* 尾段：弱/关横向 PID；s_map_mirror<0 用 *_MIRROR 系数，可与正场分开标定 */
	if (s_maze_opt_phase == MAZE_OPT_TAIL_COUNT_CROSS) {
		err_raw *= (s_map_mirror >= 0)
		    ? LT_MAZE_EXIT_TAIL3_PID_ERR_SCALE
		    : LT_MAZE_EXIT_TAIL3_PID_ERR_SCALE_MIRROR;
	} else if (s_maze_opt_phase == MAZE_OPT_TAIL_WAIT_L1234) {
		float k_l123 = (s_map_mirror >= 0)
		    ? LT_MAZE_EXIT_L1234_PID_ERR_SCALE
		    : LT_MAZE_EXIT_L1234_PID_ERR_SCALE_MIRROR;
		err_raw *= k_l123;
		if (k_l123 < 1e-3f)
			s_err_integral = 0.0f;
	}

#if LT_IDX45_CURVE_GUIDE_ENABLE != 0
	LineTrack_ApplyIdx45CurveAssist(&err_raw);
#endif

	/* idx8 起～idx9 前：光电偏置贴内弧（与强引导叠加时仍有效） */
	if (s_sensor_valid && !LineTrack_InMazeZone()
	    && !LineTrack_MazeOptical_HoldsOdomExitNode()
#if LT_RADAR_BRANCH_ENABLE
	    && s_lt_radar_phase == LT_RADAR_OFF
#endif
	    && pattern != LINE_LOST
	    && LtArcKeep_InSegment(NavOdom_GetTotalDistanceM())) {
		float b = LT_ARC_KEEP_BASE_BIAS;

		err_raw += (s_map_mirror >= 0) ? b : (-b);
		if (s_map_mirror >= 0) {
			if (LtArcKeep_OutsideWhite678(pattern) != 0u)
				err_raw += LT_ARC_KEEP_OUTSIDE_BOOST;
		} else {
			if (LtArcKeep_OutsideWhite123(pattern) != 0u)
				err_raw -= LT_ARC_KEEP_OUTSIDE_BOOST;
		}
	}
	/* 迷宫横移/尾段：本周期已 sendVel，不再跑 FSM/PID/丢线分支 */
	if (s_maze_opt_phase == MAZE_OPT_BLIND_RUN) {
		if (LineTrack_MazeBlind_Process(pattern, black_cnt))
			goto line_track_step_led;
	}

	/* 第4段等 L1234：冲出线全白时倒车找回，避免卡在「等稳定」 */
	if (s_maze_opt_phase == MAZE_OPT_TAIL_WAIT_L1234 && pattern == LINE_LOST) {
		LineTrack_SendVelFull(0.0f, -LINE_SPEED * LT_MAZE_TAIL_RECOVER_BACK_VY_K, 0.0f);
		goto line_track_step_led;
	}

#if LT_YAW_ALIGN_ASSIST_ENABLE
	if (!Odom_IsValid() || LineTrack_InMazeZone() || pattern == LINE_LOST
	    || s_route_guide_ticks > 0u)
		LineTrack_ApplyYawAlignAssist(&err_raw, pattern_pid, black_cnt_pid);
#endif

	/* FSM/弯权/竖带感：与 PID 同源用 pattern_pid */
	{
		u8 horiz = LtJunc_LikelyLocal(pattern_pid, black_cnt_pid);
		u8 vert = LtFlow_HasVerticalFeel(pattern_pid, black_cnt_pid);

		if (!LineTrack_MazeOptical_HoldsOdomExitNode())
			LtFsm_Update(pattern_pid, black_cnt_pid, err_raw, horiz, vert);
		else
			LtFsm_ResetCorner();
		/* 无里程引导时见光电直角：勿停在 PRE 减速，直接满弯转，减拖距离 */
		if (s_route_guide_ticks == 0u && !LineTrack_InMazeZone()
		    && (LtGeo_IsLeftRightAnglePat(pattern_pid, black_cnt_pid)
		        || LtGeo_IsRightRightAnglePat(pattern_pid, black_cnt_pid))
		    && s_fsm == LT_FSM_PRE_CORNER)
			LtFsm_ResetCorner();
	}

	/* ==================== BLIND_SPRINT：短冲越过交叉口 ==================== */
	if (s_fsm == LT_FSM_BLIND_SPRINT) {
		LineTrack_SendVelSave(LINE_SPEED * 0.30f,
				LineTrack_SteerSignApply(LineTrack_LostChooseVz()));
		if (s_blind_ticks > 0u)
			s_blind_ticks--;
		if (s_blind_ticks == 0u)
			LtFsm_BlindToPivot();
		goto line_track_step_led;
	}

	/* ==================== PIVOTING：间歇旋转 + 停，直到重新见线 ==================== */
	if (s_fsm == LT_FSM_PIVOTING) {
		u32 now = HAL_GetTick();
		if ((now - s_last_action_tick) > LT_PIVOT_TIMEOUT_MS) {
			LtFsm_ResetCorner();
			LtFsm_ArmCooldown();
			s_err_integral = 0;
		} else if (pattern != LINE_LOST && black_cnt_pid >= 2u && black_cnt_pid <= 4u
			   && !LtJunc_LikelyLocal(pattern_pid, black_cnt_pid)) {
			/* 必须像“细线”且不像横路口，避免吸到长方形边角就退出 */
			LtFsm_ResetCorner();
			LtFsm_ArmCooldown();
			s_err_integral = 0;
		} else {
			u16 seg_ticks = (u16)(LT_PIVOT_SEG_MS / LINE_TRACK_LOOP_MS);
			if (seg_ticks < 1u)
				seg_ticks = 1u;
			s_pivot_seg_timer++;
			if (s_pivot_seg_timer >= seg_ticks) {
				s_pivot_seg_timer = 0;
				s_pivot_rot_phase = (u8)(1u - s_pivot_rot_phase);
			}
			if (s_pivot_rot_phase)
				LineTrack_SendVelSave(0.0f,
					LineTrack_SteerSignApply((float)s_pivot_dir * LT_PIVOT_WZ));
			else
				LineTrack_SendVelSave(0.0f, 0.0f);
		}
		goto line_track_step_led;
	}

	/* ==================== 极短越线防抖 ==================== */
	if (s_force_action_ticks > 0)
	{
		s_force_action_ticks--;
		LineTrack_SendVelSave(LINE_SPEED * 0.7f, 0.0f);
		goto line_track_step_led;
	}

	if (pattern == LINE_LOST)
	{
		s_err_integral = 0; /* 丢线时清空积分，防止重新找回时过冲 */
		s_vz_slew_prev = 0.0f;
		s_vz_straight_lp = 0.0f;

		/* 迷宫内无节点引导：慢速 + 弱转向（光电流程内不用 ODOM 偏航，见 LostChooseVz） */
		if ((LineTrack_InMazeZone() || LineTrack_MazeOptical_HoldsOdomExitNode())
		    && s_route_guide_ticks == 0)
		{
			float vy_l, vz_l;
			LineTrack_LostApplyOpposite(&vy_l, &vz_l, 1u);
#if LT_DEBUG_LINE_LOST_USART3
			{
				char lb[120];
				u8 ln = (u8)sprintf(lb,
					"LOST m=1 es=%.3f lcz=%.2f osvz=%.2f out=%.2f fsm=%u\r\n",
					(double)s_err_last_seen, (double)LineTrack_LostChooseVz(),
					(double)s_last_send_vz, (double)vz_l, (unsigned)s_fsm);
				(void)HAL_UART_Transmit(&huart3, (uint8_t *)lb, ln, 8);
			}
#endif
			LineTrack_SendVelSave(vy_l, vz_l);
			goto line_track_step_led;
		}

		/* 里程引导 / 无引导：均用丢线前 err + yaw，勿用 err_raw（丢线帧恒为 0） */
		{
			float vy_l, vz_l;
			LineTrack_LostApplyOpposite(&vy_l, &vz_l, 0u);
#if LT_DEBUG_LINE_LOST_USART3
			{
				char lb[120];
				u8 ln = (u8)sprintf(lb,
					"LOST m=0 es=%.3f lcz=%.2f osvz=%.2f out=%.2f fsm=%u\r\n",
					(double)s_err_last_seen, (double)LineTrack_LostChooseVz(),
					(double)s_last_send_vz, (double)vz_l, (unsigned)s_fsm);
				(void)HAL_UART_Transmit(&huart3, (uint8_t *)lb, ln, 8);
			}
#endif
			LineTrack_SendVelSave(vy_l, vz_l);
		}
		goto line_track_step_led;
	}

	/* ==================== 里程触发：到点记节点（IGNORE 不强制弯）；迷宫未结束前不触发后续节点 ==================== */
	if (Odom_IsValid())
	{
		NavRouteAction_t ract;
		u16 gms;
		u8 fired_idx;
		float total = NavOdom_GetTotalDistanceM();

		if (!LineTrack_MazeOptical_HoldsOdomExitNode()
		    && NavRoute_TryFire(total, ROUTE_TRIGGER_LEAD_M, &ract, &gms, &fired_idx))
		{
			s_current_act = ract;
			s_last_junction_act = s_current_act;
			s_last_route_node_idx = fired_idx;
			if (ract == NAV_ROUTE_STOP) {
				s_route_guide_ticks = 0u;
				s_force_action_ticks = 0;
				s_err_integral = 0;
				s_d_filtered = 0.0f;
				if (fired_idx == LT_NAV_ROUTE_IDX_FINAL_STOP && Odom_IsValid()) {
					s_final_stop_blind_active = 1u;
					NavOdom_MarkSegmentStart();
					(void)LineTrack_FinalStopBlind_Apply();
					goto line_track_step_led;
				}
				s_line_track_halted = 1u;
				LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
			} else if (ract == NAV_ROUTE_RADAR_ARM) {
				s_route_guide_ticks = 0u;
				s_route_release_cnt = 0;
				s_force_action_ticks = 0;
				s_err_integral = 0;
#if LT_RADAR_BRANCH_ENABLE
				s_lt_radar_phase = LT_RADAR_WAIT_T;
				s_lt_radar_t_streak = 0;
#endif
				/* 勿保留枚举 6 作「弯动作」，避免与 ROUTE_GUIDE 条件组合歧义 */
				s_last_junction_act = NAV_ROUTE_STRAIGHT;
			} else if (ract == NAV_ROUTE_IGNORE) {
				if (fired_idx == LT_NAV_ROUTE_IDX_RADAR_POST_HINT) {
					/* idx9(18.40m)：仅过点；交口由循线 PID + 光电识别，不注入 ROUTE_GUIDE */
					(void)gms;
					s_route_guide_ticks = 0u;
					s_route_release_cnt = 0u;
					s_force_action_ticks = 0u;
					s_err_integral = 0;
					s_last_junction_act = NAV_ROUTE_STRAIGHT;
					s_current_act = NAV_ROUTE_STRAIGHT;
					s_last_route_node_idx = 255u;
				} else if (fired_idx == LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT) {
					/* idx10：路表 IGNORE；盲走圆弧由累计里程与 NavRoute_GetTriggerDistM(idx10) 比较触发 */
					(void)gms;
					s_route_guide_ticks = 0u;
					s_route_release_cnt = 0u;
					s_force_action_ticks = 0u;
					s_err_integral = 0;
					s_last_junction_act = NAV_ROUTE_STRAIGHT;
					s_current_act = NAV_ROUTE_STRAIGHT;
					s_last_route_node_idx = 255u;
				} else {
					/* 表为提示：不注入弯引导、不越线防抖 */
					s_route_guide_ticks = 0u;
					s_route_release_cnt = 0;
					s_force_action_ticks = 0;
					s_err_integral = 0;
					if (fired_idx == LT_NAV_ROUTE_IDX_CIRCLE_OUTER_ARC) {
						s_fig8_branch_armed = 1u;
						s_fig8_branch_done = 0u;
						s_fig8_junc_streak = 0u;
						s_fig8_circle_outer_l12 = 1u;
						if (Odom_IsValid())
							NavOdom_MarkSegmentStart();
					} else if (fired_idx == LT_NAV_ROUTE_IDX_FIG8_POST_1041) {
						s_fig8_branch_armed = 1u;
						s_fig8_branch_done = 0u;
						s_fig8_junc_streak = 0u;
						s_fig8_circle_outer_l12 = 0u;
						if (Odom_IsValid())
							NavOdom_MarkSegmentStart();
					}
				}
			} else {
				s_route_guide_ticks = gms / LINE_TRACK_LOOP_MS;
				if (s_route_guide_ticks == 0) s_route_guide_ticks = 1;
				s_route_release_cnt = 0;
				/* 只给一个极短的直行越线时间，避免在横线上抖动 */
				s_force_action_ticks = 2;
				s_last_action_tick = HAL_GetTick();
				s_err_integral = 0;
#if LT_MAZE_YAW_CHK_ENABLE
				LineTrack_MazeYawChk_Arm(fired_idx, ract);
#endif
			}

#if LT_LED_ROUTE_DEBUG
			LineTrack_LedRoute_OnNodeFired(fired_idx);
#endif
			#if IR_DEBUG_USART3
			{
				char dbg[64];
				u8 dl = (u8)sprintf(dbg, ">>> NODE #%d act=%d total=%.2f trig=%.2f\r\n",
					(int)(fired_idx + 1), (int)s_current_act,
					(double)total, (double)NavRoute_GetTriggerDistM(fired_idx));
				HAL_UART_Transmit(&huart3, (uint8_t*)dbg, dl, 10);
			}
			#endif
#if LT_ROUTE_NODE_DEBUG_PAUSE_MS > 0
			s_route_debug_pause_until_ms = HAL_GetTick() + (u32)LT_ROUTE_NODE_DEBUG_PAUSE_MS;
#endif
		}
	}

#if LT_ROUTE_NODE_DEBUG_PAUSE_MS > 0
	/* TryFire 在本步较晚执行：触发当帧须立即发零速，避免仍跑一帧 PID */
	if (s_route_debug_pause_until_ms != 0u && HAL_GetTick() < s_route_debug_pause_until_ms) {
		LineTrack_SendVelFull(0.0f, 0.0f, 0.0f);
		goto line_track_step_led;
	}
#endif

	/*
	 * idx4 外弧～idx5：路口完成**统一**用 Fig8BranchJunctionOk。
	 * 旧「仅 L1+L2 黑」在左弯外弧上几乎全程为真，会过早触发虚拟支路 + ROUTE_GUIDE（与 idx4~5 里程段行为一致）。
	 */
	if (Odom_IsValid() && s_fig8_branch_armed != 0u && s_fig8_branch_done == 0u
	    && !LineTrack_MazeOptical_HoldsOdomExitNode()
	    && !LineTrack_InMazeZone()) {
		float dsearch = NavOdom_GetDeltaSinceLastMarkM();
		u8 junction_ok;

		if (dsearch > NavRoute_PhysicalMToOdomTotalM(LT_FIG8_BRANCH_MAX_SEARCH_M)) {
			s_fig8_branch_armed = 0u;
			s_fig8_branch_done = 1u;
			s_fig8_junc_streak = 0u;
		} else if (had_new != 0u && pattern != LINE_LOST) {
			junction_ok = LtRoute_Fig8BranchJunctionOk(pattern, black_cnt);
			if (junction_ok) {
				if (s_fig8_junc_streak < 255u)
					s_fig8_junc_streak++;
				if (s_fig8_junc_streak >= LT_FIG8_BRANCH_JUNC_STREAK) {
					u16 gt;
					u8 fig8_use_mirror;

					s_fig8_branch_done = 1u;
					s_fig8_branch_armed = 0u;
					s_fig8_junc_streak = 0u;
#if LT_MAP_DETECT_ENABLE
					fig8_use_mirror = (LT_FIG8_VIRT_USE_MAP_MIRROR != 0) || (s_map_detect_locked != 0u);
#else
					fig8_use_mirror = (LT_FIG8_VIRT_USE_MAP_MIRROR != 0);
#endif
					if (fig8_use_mirror) {
						if (s_map_mirror >= 0) {
							s_current_act = NAV_ROUTE_LEFT;
							s_last_junction_act = NAV_ROUTE_LEFT;
							s_last_route_node_idx = LT_MAZE_VIRT_FIG8_LEFT_IDX;
						} else {
							s_current_act = NAV_ROUTE_RIGHT;
							s_last_junction_act = NAV_ROUTE_RIGHT;
							s_last_route_node_idx = LT_MAZE_VIRT_FIG8_RIGHT_IDX;
						}
					} else {
						s_current_act = NAV_ROUTE_LEFT;
						s_last_junction_act = NAV_ROUTE_LEFT;
						s_last_route_node_idx = LT_MAZE_VIRT_FIG8_LEFT_IDX;
					}
					if (s_fig8_circle_outer_l12 != 0u)
						gt = (u16)(LT_FIG8_CIRCLE_OUTER_GUIDE_MS / LINE_TRACK_LOOP_MS);
					else
						gt = (u16)(LT_FIG8_BRANCH_GUIDE_MS / LINE_TRACK_LOOP_MS);
					if (gt < 1u)
						gt = 1u;
					s_route_guide_ticks = gt;
					s_route_release_cnt = 0u;
					s_force_action_ticks = 2u;
					s_err_integral = 0.0f;
					LtFsm_ResetCorner();
				}
			} else {
				s_fig8_junc_streak = 0u;
			}
		} else if (had_new != 0u) {
			s_fig8_junc_streak = 0u;
		}
	}

#if LT_POST_950_CORNER_GUIDE_ENABLE != 0u
	/* idx3(10.10m) 后～idx6(15.30m) 前：非镜像 L678 黑→强制 ROUTE_GUIDE 右转；镜像(ms<0) L123 黑→强制左转 */
	if (Odom_IsValid() && had_new != 0u && !LineTrack_InMazeZone()
	    && !LineTrack_MazeOptical_HoldsOdomExitNode()
#if LT_RADAR_BRANCH_ENABLE
	    && s_lt_radar_phase == LT_RADAR_OFF
#endif
	    ) {
		if (!s_post_950_forced_once && s_route_guide_ticks == 0u
		    && LtPost950_InOdomWindow()) {
			u8 hit = 0u;

			if (s_map_mirror >= 0)
				hit = LtPost950_PatRight678(pattern);
			else
				hit = LtPost950_PatLeft123(pattern);
			if (hit != 0u) {
				if (s_post_950_corner_streak < 255u)
					s_post_950_corner_streak++;
			} else {
				s_post_950_corner_streak = 0u;
			}
			if (s_post_950_corner_streak >= LT_POST_950_CORNER_STREAK) {
				u16 gt_ps = (u16)(LT_POST_950_CORNER_GUIDE_MS / LINE_TRACK_LOOP_MS);

				s_current_act = (s_map_mirror >= 0) ? NAV_ROUTE_RIGHT : NAV_ROUTE_LEFT;
				s_last_junction_act = s_current_act;
				s_last_route_node_idx = LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX;
				if (gt_ps < 1u)
					gt_ps = 1u;
				s_route_guide_ticks = gt_ps;
				s_route_release_cnt = 0u;
				s_force_action_ticks = 2u;
				s_last_action_tick = HAL_GetTick();
				s_err_integral = 0.0f;
				s_post_950_forced_once = 1u;
				s_post_950_corner_streak = 0u;
				LtFsm_ResetCorner();
			}
		}
	}
#endif /* LT_POST_950_CORNER_GUIDE_ENABLE */

	if (s_line_track_halted)
		goto line_track_step_led;

	if (had_new) {
		float de_sc;

		s_err = err_raw;
		de_sc = err_raw - s_prev_err_raw_sc;
		derr_jump_abs = (de_sc >= 0.0f) ? de_sc : -de_sc;
	}

	/* ==================== ODOM节点指导 PID ==================== */
	if (s_route_guide_ticks > 0)
	{
		float err_abs_now = (s_err >= 0.0f) ? s_err : -s_err;
		float guide_err = ROUTE_GUIDE_ERR;
		u8 wide_r_hold = 0;

		if (s_last_junction_act == NAV_ROUTE_LEFT) {
			float lg = 1.0f;
			if (s_last_route_node_idx == LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT)
				lg = LT_ROUTE_GUIDE_LEFT650_K * LT_POSMAP_STRONG_LEFT_K;
			else if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_LEFT_IDX)
				lg = LT_ROUTE_GUIDE_LEFT650_K;
			else if (s_last_route_node_idx == LT_MAZE_VIRT_ENTRY_L123_LEFT_IDX
			    && LtRoute_MazeEntryL123Ok(pattern))
				lg = LT_ROUTE_GUIDE_LEFT650_K;
			else if (s_last_route_node_idx == LT_MAZE_VIRT_LEFT_IDX
				 && LtRoute_MazeTailL12345Ok(pattern))
				lg = LT_ROUTE_GUIDE_LEFT650_K;
			else if (s_last_route_node_idx == LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX
				 && LtPost950_PatLeft123(pattern))
				lg = LT_ROUTE_GUIDE_LEFT650_K;
			guide_err *= lg;
		} else if (s_last_junction_act == NAV_ROUTE_RIGHT) {
			if (s_last_route_node_idx == LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT) {
				guide_err *= LT_ROUTE_GUIDE_WIDE_R_K * LT_POSMAP_STRONG_RIGHT_K;
				wide_r_hold = 1u;
			} else if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_RIGHT_IDX) {
				guide_err *= LT_ROUTE_GUIDE_WIDE_R_K;
				wide_r_hold = 1u;
			} else if ((s_last_route_node_idx == LT_MAZE_VIRT_ENTRY_T_RIGHT_IDX
			     || s_last_route_node_idx == LT_MAZE_VIRT_RIGHT_IDX
			     || s_last_route_node_idx == LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX)
				&& LtRoute_RightWideBoostOk(s_last_route_node_idx, pattern)) {
				guide_err *= LT_ROUTE_GUIDE_WIDE_R_K;
				if (s_last_route_node_idx == LT_MAZE_VIRT_ENTRY_T_RIGHT_IDX)
					wide_r_hold = (black_cnt > ROUTE_WIDE_RELEASE_MAX_BLACK) ? 1u : 0u;
				else
					wide_r_hold = 1u; /* 尾段 L7/L8 出弯 */
			}
		}
		if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_LEFT_IDX
		    || s_last_route_node_idx == LT_MAZE_VIRT_FIG8_RIGHT_IDX) {
			if (s_fig8_circle_outer_l12 != 0u)
				guide_err *= LT_FIG8_CIRCLE_OUTER_GUIDE_SOFT_K;
			else {
				guide_err *= LT_FIG8_BRANCH_GUIDE_SOFT_K;
				guide_err *= LT_FIG8_BRANCH_GUIDE_ERR_K;
				if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_LEFT_IDX)
					guide_err *= LT_FIG8_BRANCH_LEFT_GUIDE_EXTRA_K;
			}
		}

		/* 已经重新咬住并基本居中，提前释放上一个节点引导，避免残留转向影响下一个节点 */
		if (!wide_r_hold && err_abs_now < ROUTE_RELEASE_ERR)
		{
			if (s_route_release_cnt < 255) s_route_release_cnt++;
			if (s_route_release_cnt >= ROUTE_RELEASE_CNT)
			{
				s_route_guide_ticks = 0;
			}
		}
		else
		{
			s_route_release_cnt = 0;
		}

		if (s_route_guide_ticks > 0) s_route_guide_ticks--;
		if (s_last_junction_act == NAV_ROUTE_LEFT)
		{
			if (s_err < guide_err) {
				if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_LEFT_IDX) {
					float t;

					if (s_fig8_circle_outer_l12 != 0u)
						t = LT_FIG8_CIRCLE_OUTER_GUIDE_BLEND;
					else
						t = LT_FIG8_BRANCH_LEFT_GUIDE_BLEND;
					s_err = s_err + (guide_err - s_err) * t;
				} else
					s_err = guide_err;
			}
		}
		else if (s_last_junction_act == NAV_ROUTE_RIGHT)
		{
			if (s_err > -guide_err) {
				if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_RIGHT_IDX) {
					float t = (s_fig8_circle_outer_l12 != 0u)
					    ? LT_FIG8_CIRCLE_OUTER_GUIDE_BLEND
					    : LT_FIG8_BRANCH_GUIDE_BLEND;
					s_err = s_err + (-guide_err - s_err) * t;
				} else
					s_err = -guide_err;
			}
		}
		else if (s_last_junction_act == NAV_ROUTE_STRAIGHT)
		{
			s_err = 0.0f;
		}
		/* NAV_ROUTE_IGNORE：不强制 err，交给 Track_Err */
	}

	/* 光电强引导结束：当帧立即回到 TRACKING（下一拍直接纯 PID） */
	if (s_route_guide_ticks == 0u && s_last_route_node_idx == LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX) {
		s_last_junction_act = NAV_ROUTE_STRAIGHT;
		LtFsm_ResetCorner();
		LtFsm_ArmCooldown();
		s_last_route_node_idx = 255u;
	}

	/* 无新光电包：不刷新 err/不跑 PID，仅重发上一帧速度（与光电包率对齐） */
	if (!had_new) {
#if LT_LINE_RESEND_LAST_VEL
		LineTrack_SendVelFull(s_last_send_vx, s_last_send_vy, s_last_send_vz);
#endif
		goto line_track_step_led;
	}

	/* 非节点引导：小误差柔化，减少贴直道时高频抖；弯/S 弯放大等效 err */
	if (s_route_guide_ticks == 0) {
		float ae = (s_err >= 0.0f) ? s_err : -s_err;

		if (ae < LT_ERR_SOFT_DEADZONE)
			s_err *= LT_ERR_SOFT_SCALE;
		else if (ae >= LT_ARC_ERR_MIN && ae <= LT_ARC_ERR_MAX)
			s_err *= LT_ARC_ERR_BOOST;
		else if (ae > LT_ARC_ERR_MAX)
			s_err *= LT_ARC_ERR_BOOST * LT_LARGE_ERR_BOOST;
	}

	/* 弯/直角权重：|err|、PRE、节点转向、Δyaw 综合；低通 s_curve_blend 防输出阶跃 */
	{
		float ae_p;
		float want;
		float dy;
		float blend_a = LT_CURVE_BLEND_ALPHA;

		ae_p = (s_err >= 0.0f) ? s_err : -s_err;
		want = 0.0f;
		if (ae_p >= LT_CURVE_ERR_ENTER)
			want = 1.0f;
		if (s_fsm == LT_FSM_PRE_CORNER && want < 0.88f)
			want = 0.88f;
		if (s_route_guide_ticks > 0 && s_last_junction_act != NAV_ROUTE_STRAIGHT
		    && s_last_junction_act != NAV_ROUTE_IGNORE) {
			float wfloor = 0.82f;
			if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_LEFT_IDX
			    || s_last_route_node_idx == LT_MAZE_VIRT_FIG8_RIGHT_IDX)
				wfloor = LT_FIG8_CIRCLE_OUTER_CURVE_WANT_FLOOR;
			if (want < wfloor)
				want = wfloor;
		}
		if (Odom_IsValid() && !LineTrack_InMazeZone()
		    && !LineTrack_MazeOptical_HoldsOdomExitNode()) {
			dy = NavOdom_GetLastYawDeltaDeg() * NAV_ODOM_YAW_LEFT_SIGN;
			if (dy > 0.32f || dy < -0.32f) {
				if (want < 0.52f)
					want = 0.52f;
			}
		}
		if (LineTrack_InMazeZone() && want > 0.48f)
			want = 0.48f;
		/* 仅最左/最右单点见黑：勿过早当直道，保留弯权利于出弯拉直（与 PID 同源 pattern_pid） */
		if ((pattern_pid == 0x7Fu || pattern_pid == 0xFEu) && want < 0.70f)
			want = 0.70f;
		/* 急 S：偏差跳变大时保持弯模式，减轻中段丢线 */
		if (derr_jump_abs > LT_S_CURVE_ERR_JUMP_TH && want < LT_S_CURVE_JUMP_WANT_MIN)
			want = LT_S_CURVE_JUMP_WANT_MIN;
		/* 正常巡线 + 光电直角：满弯权 */
		if (s_route_guide_ticks == 0u && pattern_pid != 0x00u && pattern_pid != LINE_LOST
		    && !LineTrack_InMazeZone()) {
			u8 geo_hi = 0u;

			if (LtGeo_IsRightRightAnglePat(pattern_pid, black_cnt_pid))
				geo_hi = 1u;
			else if (LtGeo_IsLeftRightAnglePat(pattern_pid, black_cnt_pid))
				geo_hi = 1u;
			if (geo_hi != 0u) {
				want = 1.0f;
				blend_a = LT_GEO_RA_CURVE_BLEND_ALPHA;
			}
		}

		s_curve_blend += blend_a * (want - s_curve_blend);
		if (s_curve_blend > 1.0f)
			s_curve_blend = 1.0f;
		if (s_curve_blend < 0.0f)
			s_curve_blend = 0.0f;
	}

	/* 直道略提速：与 s_curve_blend 互斥；迷宫/宽黑/丢线/节点转弯时不加速 */
	{
		float ae_st;
		float want_s;

		ae_st = (s_err >= 0.0f) ? s_err : -s_err;
		want_s = 0.0f;
		if (!LineTrack_InMazeZone() && !LineTrack_MazeOptical_HoldsOdomExitNode()
		    && s_curve_blend < 0.22f && s_fsm == LT_FSM_TRACKING
		    && pattern_pid != 0x00u && pattern_pid != LINE_LOST
		    && !(s_route_guide_ticks > 0 && s_last_junction_act != NAV_ROUTE_STRAIGHT
		         && s_last_junction_act != NAV_ROUTE_IGNORE)
		    && ae_st <= LT_STRAIGHT_ERR_MAX
		    && derr_jump_abs <= LT_STRAIGHT_DERR_MAX)
			want_s = 1.0f;

		s_straight_blend += LT_STRAIGHT_BLEND_ALPHA * (want_s - s_straight_blend);
		if (s_straight_blend > 1.0f)
			s_straight_blend = 1.0f;
		if (s_straight_blend < 0.0f)
			s_straight_blend = 0.0f;
	}

	{
		float dt_s = PID_DT;
		u32 nowtick = HAL_GetTick();

		if (s_last_ir_tick_ms != 0u) {
			dt_s = (float)(nowtick - s_last_ir_tick_ms) / 1000.0f;
			if (dt_s < 0.002f)
				dt_s = 0.002f;
			else if (dt_s > 0.040f)
				dt_s = 0.040f;
		}
		s_last_ir_tick_ms = nowtick;
		{
			float kps = 1.0f + LT_PID_KP_CURVE_SCALE * s_curve_blend;
			float kds = 1.0f + LT_PID_KD_CURVE_SCALE * s_curve_blend;
			float d_alpha = D_FILTER_ALPHA;

			if (s_curve_blend < 0.25f && s_straight_blend > 0.38f) {
				float sb = s_straight_blend;
				kps *= (1.0f - LT_STRAIGHT_PID_KP_REDUCE * sb);
				kds *= (1.0f + LT_STRAIGHT_PID_KD_EXTRA * sb);
				d_alpha = LT_D_FILTER_STRAIGHT_ALPHA;
			}
			if (LineTrack_InMazeZone()) {
				kps *= 0.92f;
				kds *= 0.94f;
				d_alpha = D_FILTER_ALPHA;
			}
			pid_out = PID_Calc(s_err, dt_s, kps, kds, d_alpha);
		}
	}
	/* 与 XUN 参考一致：err>0→pid>0→vz_xun<0（文档系左转=负 Vz） */
	{
		float vz_xun = -pid_out * (1.0f + LT_CURVE_STEER_PRIOR * s_curve_blend);

		vz = LineTrack_SteerSignApply(vz_xun);
	}
	if (vz > VZ_ANGLE_MAX)  vz = VZ_ANGLE_MAX;
	if (vz < -VZ_ANGLE_MAX) vz = -VZ_ANGLE_MAX;
	if (LineTrack_InMazeZone()) {
		if (vz > LT_MAZE_VZ_ABS_MAX)  vz = LT_MAZE_VZ_ABS_MAX;
		if (vz < -LT_MAZE_VZ_ABS_MAX) vz = -LT_MAZE_VZ_ABS_MAX;
	}
	/* 弯里略放宽 vz 变化率；直道减阶跃、略加大死区，减轻蛇形 */
	{
		float vz_lim = LT_VZ_SLEW_MAX + s_curve_blend * LT_VZ_SLEW_CURVE_ADD;
		if (s_curve_blend < 0.20f) {
			vz_lim -= LT_VZ_SLEW_STRAIGHT_SUB;
			if (vz_lim < 1.9f)
				vz_lim = 1.9f;
		}
		float dv = vz - s_vz_slew_prev;
		if (dv > vz_lim)
			vz = s_vz_slew_prev + vz_lim;
		else if (dv < -vz_lim)
			vz = s_vz_slew_prev - vz_lim;
	}
	{
		float vz_dead = VZ_ANGLE_MIN * (1.0f - s_curve_blend * (1.0f - LT_CURVE_VZ_DEADZONE_SCALE));
		if (s_curve_blend < 0.20f)
			vz_dead *= 1.28f;
		if (vz > -vz_dead && vz < vz_dead)
			vz = 0;
	}
	/* 直道且非节点引导：对 vz 再低通，滤掉光电跳变引起的高频摆头 */
	if (!LineTrack_InMazeZone() && !LineTrack_MazeOptical_HoldsOdomExitNode()
	    && s_curve_blend < LT_STRAIGHT_VZ_LP_CURVE_BLEND_MAX
	    && s_straight_blend > 0.38f
	    && s_fsm == LT_FSM_TRACKING && s_route_guide_ticks == 0) {
		s_vz_straight_lp += LT_STRAIGHT_VZ_LP_ALPHA * (vz - s_vz_straight_lp);
		vz = s_vz_straight_lp;
	} else {
		s_vz_straight_lp = vz;
	}
	s_vz_slew_prev = vz;

	/* 大偏差减速 + 弯里额外让纵向给横向 */
	err_abs = (s_err > 0) ? s_err : -s_err;
	speed_factor = 1.0f - 0.38f * (err_abs / 15.0f);
	if (speed_factor < 0.40f) speed_factor = 0.40f;
	speed_factor *= (1.0f - LT_CURVE_VY_EXTRA_PENALTY * s_curve_blend);
	if (speed_factor < 0.36f) speed_factor = 0.36f;
	vy = LINE_SPEED * speed_factor;
	if (derr_jump_abs > LT_S_CURVE_ERR_JUMP_TH && s_curve_blend > 0.28f)
		vy *= LT_TIGHT_S_VY_SCALE;

	/* 下一节点前：先减速（迷宫光电段内不用剩余里程判，避免 ODOM 参与） */
	if (Odom_IsValid() && s_route_guide_ticks == 0
	    && !LineTrack_MazeOptical_HoldsOdomExitNode()
	    && NavRoute_GetNextIndex() < NavRoute_GetCount()) {
		float rem_ap = NavRoute_GetRemainingToNextM(NavOdom_GetTotalDistanceM(),
				ROUTE_TRIGGER_LEAD_M);
		if (rem_ap < NavRoute_PhysicalMToOdomTotalM(ROUTE_APPROACH_SLOW_M)
		    && rem_ap > -0.05f)
			vy *= ROUTE_APPROACH_VY_K;
	}

	/* idx4～idx5 弯段：在接近减速之上再略降纵向 */
	if (Odom_IsValid() && LtIdx45_InCurveWindow())
		vy *= LT_IDX45_CURVE_VY_K;

	/* 节点引导期额外降速，方便吸入目标分支 */
	if (s_route_guide_ticks > 0 && s_last_junction_act != NAV_ROUTE_STRAIGHT
	    && s_last_junction_act != NAV_ROUTE_IGNORE)
	{
		float gk = ROUTE_GUIDE_SPEED_K;

		if (LtRoute_IsMazeSectionIdx(s_last_route_node_idx))
			gk = ROUTE_GUIDE_KEY_SPEED_K;
		if (s_last_route_node_idx == LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT)
			gk = LT_POSMAP_STRONG_LEFT_GUIDE_VY_K;
		{
			float guide_vy = LINE_SPEED * gk;
			if (vy > guide_vy) vy = guide_vy;
		}
		if ((s_last_route_node_idx == LT_MAZE_VIRT_ENTRY_L123_LEFT_IDX
		     && LtRoute_MazeEntryL123Ok(pattern))
		    || (s_last_route_node_idx == LT_MAZE_VIRT_LEFT_IDX
			&& LtRoute_MazeTailL12345Ok(pattern))
		    || (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_RIGHT_IDX)
		    || ((s_last_route_node_idx == LT_MAZE_VIRT_ENTRY_T_RIGHT_IDX
			 || s_last_route_node_idx == LT_MAZE_VIRT_RIGHT_IDX
			 || s_last_route_node_idx == LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX)
			&& LtRoute_RightWideBoostOk(s_last_route_node_idx, pattern))
		    || (s_last_route_node_idx == LT_NAV_ROUTE_VIRT_POST_950_CORNER_IDX
			&& LtPost950_PatLeft123(pattern))
		    || (s_last_route_node_idx == LT_NAV_ROUTE_IDX_POSMAP_STRONG_LEFT
			&& ((s_last_junction_act == NAV_ROUTE_LEFT
			     && LtPost950_PatLeft123(pattern))
			    || (s_last_junction_act == NAV_ROUTE_RIGHT
				&& LtPost950_PatRight678(pattern)))))
			vy *= ROUTE_GUIDE_PATTERN_SLOW_K;
		if (s_last_route_node_idx == LT_MAZE_VIRT_FIG8_LEFT_IDX
		    || s_last_route_node_idx == LT_MAZE_VIRT_FIG8_RIGHT_IDX)
			vy *= LT_FIG8_CIRCLE_OUTER_GUIDE_VY_K;
	}

	/* PRE_MAZE 门限后～出迷宫光电流程结束：降速；MAZE_OPT_DONE 后本段不生效，纵向恢复 */
	if (Odom_IsValid()
	    && NavOdom_GetTotalDistanceM() >= NavRoute_GetTriggerDistM(LT_NAV_ROUTE_IDX_PRE_MAZE_LEFT)
	    && s_maze_opt_phase != MAZE_OPT_DONE) {
		vy *= LT_MAZE_CORRIDOR_VY_K;
	}

	/* 尾段第 3 段：右横移后至满直行门限前（仅 ODOM 判定距离） */
	if (s_maze_opt_phase == MAZE_OPT_TAIL_COUNT_CROSS)
		vy *= LT_MAZE_TAIL_STRAIGHT_RUN_VY_K;

	/* 第 4 段：等 L1234 稳定黑左横移 */
	if (s_maze_opt_phase == MAZE_OPT_TAIL_WAIT_L1234)
		vy *= LT_MAZE_TAIL_SECOND_CROSS_VY_K;

	/* PRE_CORNER：接近路口时略减速（对应文档 PRE） */
	if (s_fsm == LT_FSM_PRE_CORNER)
		vy *= 0.72f;

	/* 全黑穿心（长方形竖边）：略降速直行，减少甩尾冲进侧线 */
	if (pattern == 0x00u)
		vy *= 0.82f;

	vy *= (1.0f + LT_STRAIGHT_SPEED_BOOST * s_straight_blend);
	if (vy > LINE_SPEED * LT_STRAIGHT_VY_CAP_K)
		vy = LINE_SPEED * LT_STRAIGHT_VY_CAP_K;

	if (LineTrack_InMazeZone())
		vy *= LT_MAZE_COMPLEX_VY_K;

	/* 纵向一阶限斜率：加速柔、入弯可较快降速，减少卡顿感 */
	{
		float dvy = vy - s_vy_smoothed;
		if (dvy > LT_VY_SLEW_UP_PT)
			dvy = LT_VY_SLEW_UP_PT;
		else if (dvy < -LT_VY_SLEW_DN_PT)
			dvy = -LT_VY_SLEW_DN_PT;
		s_vy_smoothed += dvy;
		vy = s_vy_smoothed;
	}
	if (had_new)
		s_prev_err_raw_sc = err_raw;

	LineTrack_SendVelSave(vy, vz);

	#if IR_DEBUG_USART3
	s_debug_cnt++;
	if (s_debug_cnt >= IR_DEBUG_CNT) { s_debug_cnt = 0; }
	#endif

line_track_step_led:
	/* 必须在本周期末尾更新 PC13：OnNodeFired 在中后段才把 s_route_led_st 置位，
	 * 若 Process/Odom 灯在周期开头跑，会错一整相；下一周期 Process 先拉灭灯时
	 * Odom 尚未再次置位 → 「常亮一会又灭」。 */
#if DEBUG_ODOM_UART_ENABLE
	/* 放在 sendVel 与各 goto 汇合之后：避免 USART3/LoRa 阻塞整段巡线控制 */
	DebugOdomUart_Tick();
#endif
#if LT_LED_ROUTE_DEBUG
	LineTrack_LedRoute_Process();
#endif
	LineTrack_OdomLinkLed_Tick();
	return had_new;
}

void LineTracking_Run(void)
{
	while (1)
	{
		LineTracking_Step();
		HAL_Delay(LINE_TRACK_LOOP_MS);
	}
}

u8 LineTracking_IsHalted(void)
{
	return s_line_track_halted;
}

line_track_oled_disp_t LineTrack_GetOledDisplayStatus(void)
{
#if LT_RADAR_BRANCH_ENABLE
	{
		u32 t;

		t = HAL_GetTick();
		if (s_oled_radar_pick_deadline_ms != 0u) {
			if ((s32)(t - s_oled_radar_pick_deadline_ms) < 0)
				return s_oled_radar_pick_is_left ? LINE_TRACK_OLED_RADAR_PICK_L
								   : LINE_TRACK_OLED_RADAR_PICK_R;
			s_oled_radar_pick_deadline_ms = 0u;
		}
	}
	if (s_lt_radar_phase == LT_RADAR_WAIT_T
	    || s_lt_radar_phase == LT_RADAR_SCANNING
	    || s_lt_radar_phase == LT_RADAR_BRANCH_ROT
	    || s_lt_radar_phase == LT_RADAR_CREEP) {
		return LINE_TRACK_OLED_RADAR;
	}
#endif
	{
		u8 pat = NavSense_BuildPattern();

		if (pat == 0xFFu) {
			return LINE_TRACK_OLED_LOST;
		}
		{
			u8 bc = NavSense_BlackCount(pat);
			float e = Track_ErrBlendWithMidPair(Track_ErrFromPattern(pat), pat, bc);
			const float dead = 2.5f;

			if (e > dead) {
				return LINE_TRACK_OLED_STEER_L;
			}
			if (e < -dead) {
				return LINE_TRACK_OLED_STEER_R;
			}
		}
	}
	return LINE_TRACK_OLED_TRACK;
}

s8 LineTrack_GetMapMirror(void)
{
	return s_map_mirror;
}

u8 LineTrack_IsMapDetectLocked(void)
{
#if LT_MAP_DETECT_ENABLE
	return s_map_detect_locked;
#else
	return 0u; /* 判别关闭：mlk=0，避免与「已过 xcm 门槛锁定」混淆 */
#endif
}

long LineTrack_GetMapDetectXcm(void)
{
	float xm;

	if (!Odom_IsValid())
		return 0L;
	xm = NavOdom_GetX();
	return (long)(xm * 100.0f + (xm >= 0.0f ? 0.5f : -0.5f));
}

u8 LineTrack_IsMapOdomLatched(void)
{
#if LT_MAP_DETECT_ENABLE
	return s_map_odom_latched;
#else
	return 0u;
#endif
}
