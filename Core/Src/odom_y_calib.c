/**
 * @file odom_y_calib.c
 * @brief N_PosY 绝对阈值停车：py ≥ ODOM_Y_CALIB_PY_STOP_M 时发零速，与 OXY 行中 py 定义一致。
 */
#include "odom_y_calib.h"

#if ODOM_Y_1M_CALIB_TEST

#include "DF_Communication.h"
#include "nav_odom.h"
#include <stdio.h>
#if DEBUG_ODOM_UART_ENABLE
#include "debug_odom_uart.h"
#endif

typedef enum {
	YCAL_WAIT_ODOM = 0,
	YCAL_RUN,
	YCAL_DONE
} OdomYCalibState_t;

static OdomYCalibState_t s_st;
static uint32_t s_t_arm_ms;

void OdomYCalib_Init(void)
{
	s_st = YCAL_WAIT_ODOM;
	s_t_arm_ms = 0U;
}

void OdomYCalib_Tick(void)
{
	const float py_stop = ODOM_Y_CALIB_PY_STOP_M;

	Odom_MainLoop();
	NavOdom_UpdateStep();

	switch (s_st) {
	case YCAL_WAIT_ODOM:
		if (!Odom_IsValid()) {
			sendVel_NoWait(0.0f, 0.0f, 0.0f);
			break;
		}
		s_t_arm_ms = HAL_GetTick();
		s_st = YCAL_RUN;
		printf("[YCAL] armed py=%.4f stop_at>=%.4f m Vy=%.1f (absolute py, same as OXY)\r\n",
		       (double)NavOdom_GetY(), (double)py_stop, (double)ODOM_Y_CALIB_VY);
		break;

	case YCAL_RUN: {
		float py = NavOdom_GetY();

		if ((uint32_t)(HAL_GetTick() - s_t_arm_ms) >= ODOM_Y_CALIB_TIMEOUT_MS) {
			sendVel_NoWait(0.0f, 0.0f, 0.0f);
			s_st = YCAL_DONE;
			printf("[YCAL] TIMEOUT py=%.4f (target >= %.4f)\r\n", (double)py, (double)py_stop);
			break;
		}

		if (py >= py_stop) {
			sendVel_NoWait(0.0f, 0.0f, 0.0f);
			s_st = YCAL_DONE;
			printf("[YCAL] STOP py=%.4f (>= %.4f m)\r\n", (double)py, (double)py_stop);
			break;
		}

		sendVel_NoWait(0.0f, ODOM_Y_CALIB_VY, 0.0f);
		break;
	}

	case YCAL_DONE:
	default:
		sendVel_NoWait(0.0f, 0.0f, 0.0f);
		break;
	}

#if DEBUG_ODOM_UART_ENABLE
	DebugOdomUart_Tick();
#endif
}

#else /* !ODOM_Y_1M_CALIB_TEST */

void OdomYCalib_Init(void) { }
void OdomYCalib_Tick(void) { }

#endif
