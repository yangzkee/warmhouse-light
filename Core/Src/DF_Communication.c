#include "DF_Communication.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

flag cpt_flag;
odom_data_t g_odom = {0};

static volatile u32 s_usart1_last_rx_tick = 0U;
static volatile u8 s_usart1_rx_ever = 0U;

u32 Uart1_GetLastRxTick(void)
{
	return s_usart1_last_rx_tick;
}

u8 Uart1_RxEver(void)
{
	return s_usart1_rx_ever;
}

static volatile u8 cmd_sequence_counter = 0;
static volatile u8 current_cmd_type_A = 0;
static volatile u8 current_cmd_type_B = 0;

void LED_Set(u8 status)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, status ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void delay_us(uint32_t us)
{
    uint32_t delay = us * (SystemCoreClock / 1000000) / 5;
    while (delay--)
    {
        __NOP();
    }
}

static u8 wait_flag_with_timeout(volatile u8 *flag_ptr, u8 use_led)
{
    u32 timeout_ms = COMM_ACK_TIMEOUT_MS;
    u32 start_tick = HAL_GetTick();
    u32 led_tick = start_tick;
    u8 led_status = 0;

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if (*flag_ptr)
        {
            if (use_led)
                LED_Set(1);
            *flag_ptr = 0;
            return 1;
        }

        if (use_led && (HAL_GetTick() - led_tick) >= 100)
        {
            led_status = !led_status;
            LED_Set(led_status);
            led_tick = HAL_GetTick();
        }
        HAL_Delay(1);
    }

    if (use_led)
        LED_Set(1);
    return 0;
}

static u8 add_checksum(u8 *DataTOSend, u8 data_len, u8 cnt)
{
    u16 sc = 0;
    u8 checksum_len = data_len + 7;
    for (u8 i = 0; i < checksum_len; i++)
    {
        sc += DataTOSend[i];
    }
    DataTOSend[cnt++] = BYTE0(sc);
    DataTOSend[cnt++] = BYTE1(sc);
    return cnt;
}

static u8 build_frame_header(u8 *DataTOSend, u8 cnt, u8 cmd_A, u8 cmd_B)
{
    DataTOSend[cnt++] = FRAME_HEADER;
    DataTOSend[cnt++] = ROBOT_ID;
    DataTOSend[cnt++] = PC_ID;
    DataTOSend[cnt++] = cmd_A;
    DataTOSend[cnt++] = cmd_B;
    return cnt;
}

static u8 validate_speed(float max_spd)
{
    return (max_spd >= -100.0f && max_spd <= 100.0f) ? 1 : 0;
}

static u8 send_command_and_wait(u8 *DataTOSend, u8 cnt, volatile u8 *flag_ptr, u8 use_led, u8 cmd_A, u8 cmd_B)
{
    cmd_sequence_counter++;
    if (cmd_sequence_counter == 0)
        cmd_sequence_counter = 1;

    current_cmd_type_A = cmd_A;
    current_cmd_type_B = cmd_B;
    *flag_ptr = 0;

    if (HAL_UART_Transmit(&huart1, DataTOSend, cnt, 100) != HAL_OK)
    {
        current_cmd_type_A = 0;
        current_cmd_type_B = 0;
        return 0;
    }

    delay_us(500);

    /* 若应答在发送后极短时间内已到，flag 已置位；不可再清零，否则 wait 会丢掉本次 ACK 直至超时 */
    if (*flag_ptr)
    {
        *flag_ptr = 0;
        current_cmd_type_A = 0;
        current_cmd_type_B = 0;
        delay_us(200);
        return 1;
    }

    u8 result = wait_flag_with_timeout(flag_ptr, use_led);

    current_cmd_type_A = 0;
    current_cmd_type_B = 0;

    if (result)
    {
        delay_us(200);
    }

    return result;
}

/* 解析 ODOM（0x6C 0x80，LEN=34）：与 LIGHT/test_odom/main.c ParseOdomFrame 同一套字节与公式。
 * payload=p[0..33] 对应帧内 frame[6..39]；ODOM_DATA_LEN=34。
 * yaw/vel/pos 与参考工程一致；可选解析 [0..11] IMU（参考工程不读）。 */
static void Parse_OdomData1(u8 *p, u8 len)
{
	if (len < 34)
		return;

#if ODOM_IMU_U16_AS_S16
	{
		s16 ax = (s16)(p[0] | ((u16)p[1] << 8));
		s16 ay = (s16)(p[2] | ((u16)p[3] << 8));
		s16 az = (s16)(p[4] | ((u16)p[5] << 8));
		s16 gx = (s16)(p[6] | ((u16)p[7] << 8));
		s16 gy = (s16)(p[8] | ((u16)p[9] << 8));
		s16 gz = (s16)(p[10] | ((u16)p[11] << 8));
		g_odom.acc_x = (float)ax / ODOM_ACC_SCALE_DIV;
		g_odom.acc_y = (float)ay / ODOM_ACC_SCALE_DIV;
		g_odom.acc_z = (float)az / ODOM_ACC_SCALE_DIV;
		g_odom.gyro_x = (float)gx / ODOM_GYRO_SCALE_DIV;
		g_odom.gyro_y = (float)gy / ODOM_GYRO_SCALE_DIV;
		g_odom.gyro_z = (float)gz / ODOM_GYRO_SCALE_DIV;
	}
#endif

	{
		s16 yaw_raw = (s16)(p[12] | ((u16)p[13] << 8));
		s16 vx_raw = (s16)(p[14] | ((u16)p[15] << 8));
		s16 vy_raw = (s16)(p[16] | ((u16)p[17] << 8));
		/* test_odom: heading = yaw_raw/100; vx,vy = (raw/SCALE_VELOCITY)*SPEED_CORRECTION */
		g_odom.yaw = ((float)yaw_raw) / 100.0f;
		g_odom.vel_x = (((float)vx_raw) / ODOM_SCALE_VELOCITY) * ODOM_SPEED_CORRECTION;
		g_odom.vel_y = (((float)vy_raw) / ODOM_SCALE_VELOCITY) * ODOM_SPEED_CORRECTION;
	}

	{
		s32 x_raw = (s32)(p[18] | ((u32)p[19] << 8) | ((u32)p[20] << 16) | ((u32)p[21] << 24));
		s32 y_raw = (s32)(p[22] | ((u32)p[23] << 8) | ((u32)p[24] << 16) | ((u32)p[25] << 24));
		g_odom.pos_x_raw = x_raw;
		g_odom.pos_y_raw = y_raw;
#if ODOM_POS_PAYLOAD_INT32
		/* test_odom: x,y = raw / SCALE_POSITION */
		g_odom.pos_x = ((float)x_raw) / ODOM_POS_SCALE_FACTOR;
		g_odom.pos_y = ((float)y_raw) / ODOM_POS_SCALE_FACTOR;
#else
		memcpy(&g_odom.pos_x, &p[18], 4);
		memcpy(&g_odom.pos_y, &p[22], 4);
#endif
	}

	g_odom.time_int = (u32)p[26] | ((u32)p[27] << 8) | ((u32)p[28] << 16) | ((u32)p[29] << 24);
	g_odom.time_dec = (u32)p[30] | ((u32)p[31] << 8) | ((u32)p[32] << 16) | ((u32)p[33] << 24);

	g_odom.valid = 1;
	g_odom.update_tick = HAL_GetTick();
#if DF_ODOM_LINK_STATS
	g_df_rx_odom_frame_ok++;
#endif
}

/* 官方例程 V0401：运动类回传多次进度；仅当 LEN 后前两字节为 FF 00 或 FF FF 时视为本条任务结束 */
static u8 motion_payload_task_done(const u8 *payload, u8 payload_len)
{
	if (payload_len < 2u || payload[0] != 0xFFu)
		return 0u;
	return (payload[1] == 0x00u || payload[1] == 0xFFu) ? 1u : 0u;
}

static void DF_HandleFrameWithProgress(u8 A, u8 B, u8 *payload, u8 len)
{
	/* ODOM（0x6C 0x80）：仅走 Parse_OdomData1，不参与运动完成标志（勿改） */
	if (A == CMD_TYPE_DATA_RETURN && B == SUB_CMD_ODOM_DATA1)
	{
		Parse_OdomData1(payload, len);
		return;
	}

	Flag_Anaylsis(A, B, payload, len);
}

static u8 s_df_rx_buff[128];
static u8 s_df_rx_step = 0;
static u8 s_df_payload_len = 0;
static u32 s_df_last_byte_ms = 0;

/* 电机 PWM/大电流时 UART 字节可能断续，80ms 易把一帧拆成两段丢弃 → 只收字节不解析 */
#define DF_RX_INTERBYTE_TIMEOUT_MS  250u

#if DF_ODOM_LINK_STATS
volatile u32 g_df_rx_complete_frames;
volatile u32 g_df_rx_chksum_fail;
volatile u32 g_df_rx_odom_frame_ok;
volatile u32 g_df_rx_id_skip;
volatile u32 g_df_rx_asm_parse_fail; /* 组帧长度已够(≥9)但整缓冲区未通过 ID/尾/校验的次数 */
#endif

void DF_Uart1_ResetParser(void)
{
	s_df_rx_step = 0;
	s_df_payload_len = 0;
}

void Deal_DF_Usart(u8 data)
{
	u32 t;

	/* 半帧超时仅在组帧中检查，少一次 HAL_GetTick 分支（热路径每字节一次） */
	if (s_df_rx_step != 0) {
		t = HAL_GetTick();
		if ((t - s_df_last_byte_ms) > DF_RX_INTERBYTE_TIMEOUT_MS)
			DF_Uart1_ResetParser();
		s_df_last_byte_ms = t;
	} else {
		t = HAL_GetTick();
	}

	s_usart1_last_rx_tick = t;
	s_usart1_rx_ever = 1U;

	if (s_df_rx_step == 0)
	{
		if (data == FRAME_HEADER)
		{
			s_df_rx_buff[s_df_rx_step++] = data;
			s_df_last_byte_ms = t; /* 帧头也参与字节间超时，避免误判丢弃整帧 */
		}
		return;
	}

	s_df_rx_buff[s_df_rx_step++] = data;

	if (s_df_rx_step == 6) /* Length byte */
	{
		s_df_payload_len = data;
		if (s_df_payload_len > 100)
			DF_Uart1_ResetParser();
	}
	else if (s_df_rx_step == s_df_payload_len + 9)
	{
		Data_Anaylsis(s_df_rx_buff, s_df_rx_step);
		DF_Uart1_ResetParser();
	}

	if (s_df_rx_step >= 128)
		DF_Uart1_ResetParser();
}

/*
 * 同一条 USART1 RX 上会连续出现多类回传，前 3 字节均为 DF + 两字节 ID（常见 DF 97 01），
 * 第 4、5 字节才是类型：ODOM 为 6C 80、sendVel 状态为 6F 67 等。帧长度由第 6 字节 LEN 决定，
 * 各帧独立做尾字节 FD 与 16 位校验；混发不会互相「解析失败」，除非丢字节/错步导致整帧校验不过。
 */
void Data_Anaylsis(u8 *data, u8 size)
{
    u8 pos = 0;
    u8 max_iterations = 20;
    u8 iteration_count = 0;
    u8 handled_any = 0;

    while (pos < size && iteration_count < max_iterations)
    {
        iteration_count++;
        u8 first = pos;
        u8 search_end = (size > pos + 50) ? (pos + 50) : size;

        while (first < search_end)
        {
            if (data[first] == FRAME_HEADER)
                break;
            first++;
        }

        if (first >= size)
            break;

        if (first + 5 >= size)
            break;

        /* 回传：规范为 DF 97 01…（PC_ID 在前）；部分固件仍为 DF 01 97…，须两种都认 */
        {
            u8 b1 = data[first + 1];
            u8 b2 = data[first + 2];
            u8 id_ok = (b1 == PC_ID && b2 == ROBOT_ID) || (b1 == ROBOT_ID && b2 == PC_ID);
            if (!id_ok)
            {
#if DF_ODOM_LINK_STATS
                g_df_rx_id_skip++;
#endif
                pos = first + 1;
                continue;
            }
        }

        u8 cmd_type_A = data[first + 3];
        u8 cmd_type_B = data[first + 4];
        u8 len = data[first + 5];

        if (len > 100)
        {
            pos = first + 1;
            continue;
        }

        u16 tail_pos = first + 6 + len;
        if ((tail_pos + 2) >= (u16)size)
            break;

        if (data[tail_pos] != FRAME_TAIL)
        {
            pos = first + 1;
            continue;
        }

        /* 与 add_checksum / DFLink 一致：对「帧头…数据…含 0xFD」共 len+7 字节求和，再与 16 位校验比对 */
        u16 checksum_calc = 0;
        u8 checksum_len = (u8)(len + 7u);
        u8 i;
        for (i = 0; i < checksum_len; i++)
            checksum_calc = (u16)(checksum_calc + data[first + i]);

        u16 checksum_recv;
#if DF_RX_CHECKSUM_BIG_ENDIAN
        checksum_recv = ((u16)data[tail_pos + 1] << 8) | data[tail_pos + 2];
#else
        checksum_recv = data[tail_pos + 1] | ((u16)data[tail_pos + 2] << 8);
#endif

        if (checksum_calc != checksum_recv) {
            /* 少数固件对「不含帧尾 FD」共 len+6 字节求和；首算失败时再试一次 */
            checksum_calc = 0;
            checksum_len = (u8)(len + 6u);
            for (i = 0; i < checksum_len; i++)
                checksum_calc = (u16)(checksum_calc + data[first + i]);
        }

        if (checksum_calc != checksum_recv)
        {
#if DF_ODOM_LINK_STATS
			g_df_rx_chksum_fail++;
#endif
            pos = first + 1;
            continue;
        }

#if DF_ODOM_LINK_STATS
		g_df_rx_complete_frames++;
#endif

#if ODOM_USART3_FORWARD_RAW && !USART3_MIRROR_USART1_RX
		/* 仅在不使用「字节镜像」时整帧转发，避免 USART3 上同一帧发两遍 */
		{
			u16 flen = (u16)(tail_pos + 3U - (u16)first);
			if (flen > 0U && flen <= 200U)
				(void)HAL_UART_Transmit(&huart3, &data[first], flen, 80);
		}
#endif

        DF_HandleFrameWithProgress(cmd_type_A, cmd_type_B, &data[first + 6], len);
        handled_any = 1;
        pos = tail_pos + 3;
    }
#if DF_ODOM_LINK_STATS
    if (size >= 9u && !handled_any)
        g_df_rx_asm_parse_fail++;
#endif
}

void Flag_Anaylsis(u8 A, u8 B, const u8 *payload, u8 payload_len)
{
	u8 is_expected_cmd = 0;

	if (current_cmd_type_A != 0 && current_cmd_type_B != 0)
	{
		if (current_cmd_type_A == CMD_TYPE_MOTION_BASE && A == CMD_TYPE_MOTION)
		{
			if (current_cmd_type_B == B)
				is_expected_cmd = 1;
		}
		else if (current_cmd_type_A == CMD_TYPE_NO_HEAD_LINE && A == 0x62)
			is_expected_cmd = 1;
		else if (current_cmd_type_A == 0x01 && A == CMD_TYPE_IMU_CALIB && current_cmd_type_B == B)
			is_expected_cmd = 1;
		else if (current_cmd_type_A == A && current_cmd_type_B == B)
			is_expected_cmd = 1;
	}
	else
		is_expected_cmd = 1;

	if (!is_expected_cmd)
		return;

	switch (A)
	{
	case CMD_TYPE_IMU_CALIB:
		if (B == SUB_CMD_IMU_CALIB)
			cpt_flag.sendimucali = 1;
		break;
	case CMD_TYPE_MOTION:
		/* A=6F：进度帧多次；仅 FF 00 / FF FF 结束本条（与官方例程 V0401 一致） */
		if (!motion_payload_task_done(payload, payload_len))
			break;
		switch (B)
		{
		case SUB_CMD_VEL_DISP:
			cpt_flag.sendVelDisplacement = 1;
			break;
		case SUB_CMD_ADAPTIVE_DISP:
			cpt_flag.sendpos = 1;
			break;
		case SUB_CMD_ROTATION:
			cpt_flag.sendrot = 1;
			break;
		case SUB_CMD_VEL_MOTION:
			cpt_flag.sendVel = 1;
			break;
		case SUB_CMD_ARC_DISP:
			cpt_flag.sendArcDisplacement = 1;
			break;
		default:
			break;
		}
		break;
	case CMD_TYPE_PARAM_SET:
		switch (B)
		{
		case SUB_CMD_ROBOT_PARAM:
			cpt_flag.SendRobotParSet = 1;
			break;
		case SUB_CMD_YAW_OFFSET:
			cpt_flag.sendyawoffset = 1;
			break;
		default:
			break;
		}
		break;
	case 0x62:
		cpt_flag.sendNoHeadLine = 1;
		break;
	default:
		break;
	}
}

void sendimucali(uint8_t cali_flag)
{
    u8 _cnt = 0;
    u8 DataTOSend[20];
    _cnt = build_frame_header(DataTOSend, _cnt, 0x01, SUB_CMD_IMU_CALIB);
    DataTOSend[_cnt++] = 0x01;
    DataTOSend[_cnt++] = cali_flag;
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendimucali, 1, 0x01, SUB_CMD_IMU_CALIB);
}

void sendpos(float p_x, float p_y, float p_z, float max_spd)
{
    if (!validate_speed(max_spd))
        return;
    u8 _cnt = 0;
    s32 posx = (s32)(p_x * SCALE_FACTOR_POSITION);
    s32 posy = (s32)(p_y * SCALE_FACTOR_POSITION);
    s32 posz = (s32)(p_z * SCALE_FACTOR_POSITION);
    s16 spd = (s16)(max_spd * SCALE_FACTOR_VELOCITY);
    u8 DataTOSend[30];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_MOTION_BASE, SUB_CMD_ADAPTIVE_DISP);
    DataTOSend[_cnt++] = 14;
    DataTOSend[_cnt++] = BYTE0(posx);
    DataTOSend[_cnt++] = BYTE1(posx);
    DataTOSend[_cnt++] = BYTE2(posx);
    DataTOSend[_cnt++] = BYTE3(posx);
    DataTOSend[_cnt++] = BYTE0(posy);
    DataTOSend[_cnt++] = BYTE1(posy);
    DataTOSend[_cnt++] = BYTE2(posy);
    DataTOSend[_cnt++] = BYTE3(posy);
    DataTOSend[_cnt++] = BYTE0(posz);
    DataTOSend[_cnt++] = BYTE1(posz);
    DataTOSend[_cnt++] = BYTE2(posz);
    DataTOSend[_cnt++] = BYTE3(posz);
    DataTOSend[_cnt++] = BYTE0(spd);
    DataTOSend[_cnt++] = BYTE1(spd);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendpos, 1, CMD_TYPE_MOTION_BASE, SUB_CMD_ADAPTIVE_DISP);
}

void sendrot(float r_x, float r_y, float r_z, float r_max)
{
    if (!validate_speed(r_max))
        return;
    u8 _cnt = 0;
    s16 rotx = (s16)(r_x * SCALE_FACTOR_ANGLE);
    s16 roty = (s16)(r_y * SCALE_FACTOR_ANGLE);
    s32 rotz = (s32)(r_z * SCALE_FACTOR_POSITION);
    s16 rotspd = (s16)(r_max * SCALE_FACTOR_VELOCITY);
    u8 DataTOSend[20];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_MOTION_BASE, SUB_CMD_ROTATION);
    DataTOSend[_cnt++] = 0x0A;
    DataTOSend[_cnt++] = BYTE0(rotx);
    DataTOSend[_cnt++] = BYTE1(rotx);
    DataTOSend[_cnt++] = BYTE0(roty);
    DataTOSend[_cnt++] = BYTE1(roty);
    DataTOSend[_cnt++] = BYTE0(rotz);
    DataTOSend[_cnt++] = BYTE1(rotz);
    DataTOSend[_cnt++] = BYTE2(rotz);
    DataTOSend[_cnt++] = BYTE3(rotz);
    DataTOSend[_cnt++] = BYTE0(rotspd);
    DataTOSend[_cnt++] = BYTE1(rotspd);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendrot, 1, CMD_TYPE_MOTION_BASE, SUB_CMD_ROTATION);
}

u8 sendrot_AsyncBegin(float r_x, float r_y, float r_z, float r_max)
{
	u8 _cnt = 0;
	s16 rotx;
	s16 roty;
	s32 rotz;
	s16 rotspd;
	u8 DataTOSend[20];

	if (!validate_speed(r_max))
		return 0;

	cmd_sequence_counter++;
	if (cmd_sequence_counter == 0)
		cmd_sequence_counter = 1;

	current_cmd_type_A = CMD_TYPE_MOTION_BASE;
	current_cmd_type_B = SUB_CMD_ROTATION;
	cpt_flag.sendrot = 0;

	rotx = (s16)(r_x * SCALE_FACTOR_ANGLE);
	roty = (s16)(r_y * SCALE_FACTOR_ANGLE);
	rotz = (s32)(r_z * SCALE_FACTOR_POSITION);
	rotspd = (s16)(r_max * SCALE_FACTOR_VELOCITY);

	_cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_MOTION_BASE, SUB_CMD_ROTATION);
	DataTOSend[_cnt++] = 0x0A;
	DataTOSend[_cnt++] = BYTE0(rotx);
	DataTOSend[_cnt++] = BYTE1(rotx);
	DataTOSend[_cnt++] = BYTE0(roty);
	DataTOSend[_cnt++] = BYTE1(roty);
	DataTOSend[_cnt++] = BYTE0(rotz);
	DataTOSend[_cnt++] = BYTE1(rotz);
	DataTOSend[_cnt++] = BYTE2(rotz);
	DataTOSend[_cnt++] = BYTE3(rotz);
	DataTOSend[_cnt++] = BYTE0(rotspd);
	DataTOSend[_cnt++] = BYTE1(rotspd);
	DataTOSend[_cnt++] = FRAME_TAIL;
	_cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);

	if (HAL_UART_Transmit(&huart1, DataTOSend, _cnt, 100) != HAL_OK) {
		current_cmd_type_A = 0;
		current_cmd_type_B = 0;
		return 0;
	}

	delay_us(500);
	return 1;
}

u8 DF_RotationAsyncTryConsumeDone(u32 motion_start_tick)
{
    if (!cpt_flag.sendrot)
        return 0;
    if ((HAL_GetTick() - motion_start_tick) < 300u)
        return 0;

    cpt_flag.sendrot = 0;
    current_cmd_type_A = 0;
    current_cmd_type_B = 0;
    delay_us(200);
    return 1;
}

void DF_RotationAsyncForceClear(void)
{
	cpt_flag.sendrot = 0;
	current_cmd_type_A = 0;
	current_cmd_type_B = 0;
}

void sendVel(float V_x, float V_y, float V_z)
{
    u8 _cnt = 0;
    s16 rotx = V_x * SCALE_FACTOR_VELOCITY;
    s16 roty = V_y * SCALE_FACTOR_VELOCITY;
    s16 rotz = V_z * SCALE_FACTOR_VELOCITY;
    u8 DataTOSend[20];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_MOTION_BASE, SUB_CMD_VEL_MOTION);
    DataTOSend[_cnt++] = 0x06;
    DataTOSend[_cnt++] = BYTE0(rotx);
    DataTOSend[_cnt++] = BYTE1(rotx);
    DataTOSend[_cnt++] = BYTE0(roty);
    DataTOSend[_cnt++] = BYTE1(roty);
    DataTOSend[_cnt++] = BYTE0(rotz);
    DataTOSend[_cnt++] = BYTE1(rotz);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendVel, 1, CMD_TYPE_MOTION_BASE, SUB_CMD_VEL_MOTION);
}

void sendVel_NoWait(float V_x, float V_y, float V_z)
{
    u8 _cnt = 0;
    s16 rotx = V_x * SCALE_FACTOR_VELOCITY;
    s16 roty = V_y * SCALE_FACTOR_VELOCITY;
    s16 rotz = V_z * SCALE_FACTOR_VELOCITY;
    u8 DataTOSend[20];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_MOTION_BASE, SUB_CMD_VEL_MOTION);
    DataTOSend[_cnt++] = 0x06;
    DataTOSend[_cnt++] = BYTE0(rotx);
    DataTOSend[_cnt++] = BYTE1(rotx);
    DataTOSend[_cnt++] = BYTE0(roty);
    DataTOSend[_cnt++] = BYTE1(roty);
    DataTOSend[_cnt++] = BYTE0(rotz);
    DataTOSend[_cnt++] = BYTE1(rotz);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    /* 高频控制专用，直接发，不等待接收板回ACK，也不调用任何延迟函数 */
    HAL_UART_Transmit(&huart1, DataTOSend, _cnt, 5);
}

void sendVelDisplacement(float V_x, float V_y, float V_z, float r_max)
{
    if (!validate_speed(r_max))
        return;
    u8 _cnt = 0;
    s32 rotx = (s32)(V_x * SCALE_FACTOR_POSITION);
    s32 roty = (s32)(V_y * SCALE_FACTOR_POSITION);
    s32 rotz = (s32)(V_z * SCALE_FACTOR_POSITION);
    s16 rotspd = (s16)(r_max * SCALE_FACTOR_VELOCITY);
    u8 DataTOSend[20];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_MOTION_BASE, SUB_CMD_VEL_DISP);
    DataTOSend[_cnt++] = 0x0E;
    DataTOSend[_cnt++] = BYTE0(rotx);
    DataTOSend[_cnt++] = BYTE1(rotx);
    DataTOSend[_cnt++] = BYTE2(rotx);
    DataTOSend[_cnt++] = BYTE3(rotx);
    DataTOSend[_cnt++] = BYTE0(roty);
    DataTOSend[_cnt++] = BYTE1(roty);
    DataTOSend[_cnt++] = BYTE2(roty);
    DataTOSend[_cnt++] = BYTE3(roty);
    DataTOSend[_cnt++] = BYTE0(rotz);
    DataTOSend[_cnt++] = BYTE1(rotz);
    DataTOSend[_cnt++] = BYTE2(rotz);
    DataTOSend[_cnt++] = BYTE3(rotz);
    DataTOSend[_cnt++] = BYTE0(rotspd);
    DataTOSend[_cnt++] = BYTE1(rotspd);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendVelDisplacement, 1, CMD_TYPE_MOTION_BASE, SUB_CMD_VEL_DISP);
}

void SendRobotParSet(float Par1, float Par2, float Par3, float Par4)
{
    u8 _cnt = 0;
    s32 Par11 = Par1 * SCALE_FACTOR_POSITION;
    s32 Par22 = Par2 * SCALE_FACTOR_POSITION;
    s32 Par33 = Par3 * SCALE_FACTOR_POSITION;
    s32 Par44 = Par4 * SCALE_FACTOR_POSITION;
    u8 DataTOSend[40];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_PARAM_SET, SUB_CMD_ROBOT_PARAM);
    DataTOSend[_cnt++] = 0x10;
    DataTOSend[_cnt++] = BYTE0(Par11);
    DataTOSend[_cnt++] = BYTE1(Par11);
    DataTOSend[_cnt++] = BYTE2(Par11);
    DataTOSend[_cnt++] = BYTE3(Par11);
    DataTOSend[_cnt++] = BYTE0(Par22);
    DataTOSend[_cnt++] = BYTE1(Par22);
    DataTOSend[_cnt++] = BYTE2(Par22);
    DataTOSend[_cnt++] = BYTE3(Par22);
    DataTOSend[_cnt++] = BYTE0(Par33);
    DataTOSend[_cnt++] = BYTE1(Par33);
    DataTOSend[_cnt++] = BYTE2(Par33);
    DataTOSend[_cnt++] = BYTE3(Par33);
    DataTOSend[_cnt++] = BYTE0(Par44);
    DataTOSend[_cnt++] = BYTE1(Par44);
    DataTOSend[_cnt++] = BYTE2(Par44);
    DataTOSend[_cnt++] = BYTE3(Par44);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.SendRobotParSet, 1, CMD_TYPE_PARAM_SET, SUB_CMD_ROBOT_PARAM);
}

void sendyawoffset(float offset, uint8_t revise_flag, uint8_t resolve_flag)
{
    u8 _cnt = 0;
    s32 _offset = offset * SCALE_FACTOR_POSITION;
    u8 DataTOSend[40];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_PARAM_SET, SUB_CMD_YAW_OFFSET);
    DataTOSend[_cnt++] = 0x06;
    DataTOSend[_cnt++] = BYTE0(_offset);
    DataTOSend[_cnt++] = BYTE1(_offset);
    DataTOSend[_cnt++] = BYTE2(_offset);
    DataTOSend[_cnt++] = BYTE3(_offset);
    DataTOSend[_cnt++] = revise_flag;
    DataTOSend[_cnt++] = resolve_flag;
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendyawoffset, 1, CMD_TYPE_PARAM_SET, SUB_CMD_YAW_OFFSET);
}

void sendArcDisplacement(float radius, float angle, float speed)
{
    float speed_abs = (speed < 0) ? -speed : speed;
    if (speed_abs == 0 || speed_abs > 100.0f)
        return;
    u8 _cnt = 0;
    s16 arc_radius = (s16)(radius * SCALE_FACTOR_VELOCITY);
    s16 arc_angle = (s16)(angle * SCALE_FACTOR_ANGLE);
    s16 arc_speed = (s16)(speed * SCALE_FACTOR_VELOCITY);
    u8 DataTOSend[30];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_MOTION_BASE, SUB_CMD_ARC_DISP);
    DataTOSend[_cnt++] = 0x06;
    DataTOSend[_cnt++] = BYTE0(arc_radius);
    DataTOSend[_cnt++] = BYTE1(arc_radius);
    DataTOSend[_cnt++] = BYTE0(arc_angle);
    DataTOSend[_cnt++] = BYTE1(arc_angle);
    DataTOSend[_cnt++] = BYTE0(arc_speed);
    DataTOSend[_cnt++] = BYTE1(arc_speed);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendArcDisplacement, 1, CMD_TYPE_MOTION_BASE, SUB_CMD_ARC_DISP);
}

void sendNoHeadLine(float x, float y, float z, float max_spd)
{
    if (!validate_speed(max_spd))
        return;
    u8 _cnt = 0;
    s32 line_x = (s32)(x * SCALE_FACTOR_POSITION);
    s32 line_y = (s32)(y * SCALE_FACTOR_POSITION);
    s32 line_z = (s32)(z * SCALE_FACTOR_POSITION);
    s16 line_max_spd = (s16)(max_spd * SCALE_FACTOR_VELOCITY);
      u8 DataTOSend[30];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_NO_HEAD_LINE, SUB_CMD_NO_HEAD_LINE);
    DataTOSend[_cnt++] = 0x0E;
    DataTOSend[_cnt++] = BYTE0(line_x);
    DataTOSend[_cnt++] = BYTE1(line_x);
    DataTOSend[_cnt++] = BYTE2(line_x);
    DataTOSend[_cnt++] = BYTE3(line_x);
    DataTOSend[_cnt++] = BYTE0(line_y);
    DataTOSend[_cnt++] = BYTE1(line_y);
    DataTOSend[_cnt++] = BYTE2(line_y);
    DataTOSend[_cnt++] = BYTE3(line_y);
    DataTOSend[_cnt++] = BYTE0(line_z);
    DataTOSend[_cnt++] = BYTE1(line_z);
    DataTOSend[_cnt++] = BYTE2(line_z); 
    DataTOSend[_cnt++] = BYTE3(line_z);
    DataTOSend[_cnt++] = BYTE0(line_max_spd);
    DataTOSend[_cnt++] = BYTE1(line_max_spd);
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    send_command_and_wait(DataTOSend, _cnt, &cpt_flag.sendNoHeadLine, 1, CMD_TYPE_NO_HEAD_LINE, SUB_CMD_NO_HEAD_LINE);
}

/*
 * 与协议「连续访问 10Hz」一致：DF 01 97 04 80 02 01 0A FD [chk]
 * LEN=2，负载 0x01 + 0x0A（10Hz，须用 0x0A 不能写 0x01）
 */
void requestOdomData(void)
{
    u8 _cnt = 0;
    u8 DataTOSend[20];
    _cnt = build_frame_header(DataTOSend, _cnt, CMD_TYPE_DATA_ACCESS, SUB_CMD_ODOM_DATA1);
    DataTOSend[_cnt++] = 0x02;   /* 负载长度 2 */
    DataTOSend[_cnt++] = 0x01;
    DataTOSend[_cnt++] = 0x0A;   /* 10Hz */
    DataTOSend[_cnt++] = FRAME_TAIL;
    _cnt = add_checksum(DataTOSend, DataTOSend[5], _cnt);
    HAL_UART_Transmit(&huart1, DataTOSend, _cnt, 100);
}

void System_Init(void)
{
    LED_Set(1);
    // UARTs are already initialized by CubeMX
}

float Odom_GetYaw(void)
{
	return g_odom.valid ? g_odom.yaw : 0.0f;
}

u8 Odom_IsValid(void)
{
	return g_odom.valid;
}

void Odom_Init(void)
{
#if ODOM_REQUEST_PERIODIC
	u32 t0 = HAL_GetTick();
	while ((HAL_GetTick() - t0) < 2000) {
		requestOdomData();
		HAL_Delay(100);
	}
	HAL_Delay(300);
#else
	/* 协议为「连续访问 10Hz」开话题：发一次即可；若底盘仍不吐 ODOM，可改 ODOM_REQUEST_PERIODIC=1 或检查 RX/解析 */
	requestOdomData();
	HAL_Delay(50);
#endif
}

void Odom_MainLoop(void)
{
	static u16 s_vel_keep = 0;
#if ODOM_REQUEST_PERIODIC
	static u16 cnt = 0;
	static u32 s_last_burst_ms = 0;
	u32 now;
	u16 period;

	cnt++;
	period = Odom_IsValid() ? 25U : 10U;
	if (cnt >= period) {
		cnt = 0;
		requestOdomData();
	}
#endif
#if DCAR_ODOM_KEEPALIVE_SENDVEL
	/* 与「必须一直 sendVel 车才吐 ODOM」的底盘习惯兼容：未解析成功时周期性发零速保活 */
	if (!Odom_IsValid()) {
		if (s_vel_keep < 65535u)
			s_vel_keep++;
		if (s_vel_keep >= 25u) {
			s_vel_keep = 0;
			sendVel_NoWait(0.0f, 0.0f, 0.0f);
		}
	} else {
		s_vel_keep = 0;
	}
#endif
#if ODOM_REQUEST_PERIODIC
	now = HAL_GetTick();
	if ((now - s_last_burst_ms) >= 5000U) {
		s_last_burst_ms = now;
		requestOdomData();
		requestOdomData();
		requestOdomData();
	}
#endif
}
