#ifndef _DF_COMMUNICATION_
#define _DF_COMMUNICATION_
#include "main.h"

// 协议宏定义（替换魔法数）
#define FRAME_HEADER        0xDF    // 帧头
#define FRAME_TAIL          0xFD    // 帧尾
#define ROBOT_ID            0x01    // 小车ID
#define PC_ID               0x97    // 电脑ID

/* 官方例程 V0401 默认 10s（长圆弧等）；可预定义覆盖 */
#ifndef COMM_ACK_TIMEOUT_MS
#define COMM_ACK_TIMEOUT_MS 10000UL
#endif

// 指令类型定义（A字段）
#define CMD_TYPE_IMU_CALIB         0x46
#define CMD_TYPE_MOTION            0x6f
#define CMD_TYPE_PARAM_SET         0x82
#define CMD_TYPE_NO_HEAD_LINE      0x02
#define CMD_TYPE_MOTION_BASE       0x02  // 运动类指令基类
#define CMD_TYPE_DATA_ACCESS       0x04  // 数据访问请求
#define CMD_TYPE_DATA_RETURN       0x6C  // 数据回传（ODOM数据）

// 子指令定义（B字段）
#define SUB_CMD_IMU_CALIB          0x3c
#define SUB_CMD_VEL_DISP           0x64
#define SUB_CMD_ADAPTIVE_DISP      0x65
#define SUB_CMD_ROTATION           0x66
#define SUB_CMD_VEL_MOTION         0x67
#define SUB_CMD_ARC_DISP           0x63
#define SUB_CMD_NO_HEAD_LINE       0x62
#define SUB_CMD_ROBOT_PARAM        0x6A
#define SUB_CMD_YAW_OFFSET         0x6F
#define SUB_CMD_ODOM_DATA1         0x80  // ODOM数据1

// 数据缩放因子定义
#define SCALE_FACTOR_POSITION   10000  // 位置精度：0.0001
#define SCALE_FACTOR_VELOCITY  100    // 速度精度：0.01
#define SCALE_FACTOR_ANGLE     100    // 角度精度：0.01度
/* N_PosX/Y：文档常写 F32，多数底盘固件实际为 int32 定点（÷ODOM_POS_SCALE_FACTOR→米）。
 * 与 LIGHT/test_odom（main.c ParseOdomFrame）一致：payload[18..21]/[22..25] 小端 int32，SCALE_POSITION=23810.0f。
 * 若坐标明显不对可改 ODOM_POS_PAYLOAD_INT32=0 试 F32 memcpy，或覆盖 ODOM_POS_SCALE_FACTOR。 */
#ifndef ODOM_POS_PAYLOAD_INT32
#define ODOM_POS_PAYLOAD_INT32  1
#endif
#ifndef ODOM_POS_SCALE_FACTOR
#define ODOM_POS_SCALE_FACTOR  23810.0f
#endif
/* 与 test_odom/main.c ParseOdomFrame：B_Vel 先 ÷SCALE_VELOCITY 再 ×SPEED_CORRECTION */
#ifndef ODOM_SCALE_VELOCITY
#define ODOM_SCALE_VELOCITY  100.0f
#endif
#ifndef ODOM_SPEED_CORRECTION
#define ODOM_SPEED_CORRECTION  0.42f
#endif
/* 负载 34 字节为一条完整 ODOM（文档「数据1/数据2」实为同一条拆行）：ACC/GYRO 各 3×F16，再接 Yaw、Vel、Pos、TIME。
 * test_odom 不解析前 12 字节；默认 ODOM_IMU_U16_AS_S16=0 与之对齐。需 IMU 时预定义为 1。 */
#ifndef ODOM_IMU_U16_AS_S16
#define ODOM_IMU_U16_AS_S16  0
#endif
#ifndef ODOM_ACC_SCALE_DIV
#define ODOM_ACC_SCALE_DIV  3.0f
#endif
#ifndef ODOM_GYRO_SCALE_DIV
#define ODOM_GYRO_SCALE_DIV  7.0f
#endif

// 字节提取宏（用于多字节数据拆分）
#define BYTE0(dwTemp) (*((char *) (&dwTemp)))
#define BYTE1(dwTemp) (*((char *) (&dwTemp) + 1 ))
#define BYTE2(dwTemp) (*((char *) (&dwTemp) + 2 ))
#define BYTE3(dwTemp) (*((char *) (&dwTemp) + 3 ))

// 任务状态标志结构体
typedef struct
{
    volatile u8 sendimucali;          // IMU校准完成
    volatile u8 sendVelDisplacement;  // 匀速位移完成
    volatile u8 sendpos;              // 自适应位移完成
    volatile u8 sendrot;              // 旋转完成
    volatile u8 sendVel;              // 匀速运动完成
    volatile u8 sendyawoffset;        // Yaw角偏移完成
    volatile u8 SendRobotParSet;      // 战车参数修改完成
    volatile u8 sendArcDisplacement;  // 圆弧轨迹任务完成
    volatile u8 sendNoHeadLine;       // 无头直线轨迹任务完成
}flag;

extern flag cpt_flag;

/*
 * N_PosX / N_PosY（g_odom.pos_x / pos_y）：底盘积分得到的**平面位置坐标（米）**，与小车固件约定一致。
 *
 * 约定（每次上电/任务起点视为原点）：
 *   - 上电后从零开始：(0,0) 为起点，车头默认朝向 **+Y**（世界系 y 轴正方向）。
 *   - **前进**：沿 +Y，pos_y **增大**；**后退**：pos_y **减小**。
 *   - **横向**：**左移**（车身向左平移、车头仍朝 +Y）时 pos_x **增大**；**右移**时 pos_x **减小**。
 *     （若口语里「右移」曾写成 y 变化，以底盘实际回传为准；一般横向只主要改变 x。）
 * 本工程解析后单位为米；与巡线 sendVel(Vx,Vy,…) 的轴符号是否同号，以实车为准单独核对。
 */
/* ODOM数据（0x6C 0x80，负载34字节） */
typedef struct {
	float yaw;        /* 航向角，度（s16/100） */
	float pos_x;      /* N_PosX，米，见上文世界系约定 */
	float pos_y;      /* N_PosY，米，见上文世界系约定 */
	float vel_x;      /* B_VelX，m/s 量级（s16/100） */
	float vel_y;      /* B_VelY */
	s32 pos_x_raw;    /* N_PosX：负载字节 18,19,20,21 小端 32bit；调试 npx 为原样 */
	s32 pos_y_raw;    /* N_PosY：负载字节 22,23,24,25 */
	float acc_x;      /* 负载 [0..5]；见 ODOM_ACC_SCALE_DIV */
	float acc_y;
	float acc_z;
	float gyro_x;     /* 负载 [6..11]；见 ODOM_GYRO_SCALE_DIV */
	float gyro_y;
	float gyro_z;
	u32 time_int;     /* 负载 [26..29] */
	u32 time_dec;     /* 负载 [30..33] */
	volatile u8 valid; /* 1=本帧解析成功；ISR(串口回调)写、主循环读，须 volatile */
	volatile u32 update_tick; /* 最近一次成功解析 ODOM 的 systick；用于判断数据是否仍新鲜 */
} odom_data_t;
extern odom_data_t g_odom;

// 错误状态枚举
typedef enum {
    DF_OK = 0,
    DF_ERR_TIMEOUT,
    DF_ERR_INVALID_PARAM,
    DF_ERR_COMM_FAIL
} df_status_t;

// 函数声明
void LED_Set(u8 status);
void Deal_DF_Usart(u8 data);
void DF_Uart1_ResetParser(void); /* 拔插线/帧错误后清组帧状态，并配合 ErrorCallback 重启 RX */
void Data_Anaylsis(u8 *data,u8 size);
void Flag_Anaylsis(u8 A, u8 B, const u8 *payload, u8 payload_len);
void sendimucali(uint8_t cali_flag);
void sendVelDisplacement(float V_x, float  V_y, float  V_z, float r_max);
void sendpos(float p_x, float p_y, float p_z,float max_spd);
void sendrot(float r_x, float r_y, float r_z,float r_max);
/* 旋转指令仅发出、不阻塞；完成时底盘回传由 Flag_Anaylsis 置 cpt_flag.sendrot */
u8 sendrot_AsyncBegin(float r_x, float r_y, float r_z, float r_max);
/* 自运动开始后至少 300ms 且收到完成标志时清标志并返回 1 */
u8 DF_RotationAsyncTryConsumeDone(u32 motion_start_tick);
/* 雷达扫描超时跳过旋转等待时调用，避免 cpt_flag 残留误触发后续 TryConsumeDone */
void DF_RotationAsyncForceClear(void);
void sendVel(float V_x, float  V_y, float  V_z);
void sendVel_NoWait(float V_x, float  V_y, float  V_z);
void sendyawoffset(float offset, uint8_t revise_flag, uint8_t resolve_flag);
void SendRobotParSet(float Par1, float  Par2, float  Par3, float Par4);
void sendArcDisplacement(float radius, float angle, float speed);
void sendNoHeadLine(float x, float y, float z, float max_spd);
void requestOdomData(void);
float Odom_GetYaw(void);   /* 获取当前Yaw，无效返回0 */
u8 Odom_IsValid(void);     /* ODOM数据是否有效 */
u32 Uart1_GetLastRxTick(void); /* 最近一次 USART1 收到字节的时刻（调试用灯） */
u8  Uart1_RxEver(void);         /* 是否曾收到过至少 1 字节（避免 tick==0 误判） */

void System_Init(void);
void Odom_Init(void);
void Odom_MainLoop(void);

/*
 * USART3 输出（默认全关，减轻 USART1 RX 中断与主循环负担，优先保证 ODOM 组帧解析）：
 * - USART3_MIRROR_USART1_RX=1：每从 USART1 收 1 字节镜像到 USART3（调试用，高波特率下曾拖长 ISR）。
 * - ODOM_USART3_FORWARD_RAW=1 且 MIRROR=0：整帧校验通过后再转发到 USART3（与官方二进制工具兼容）。
 */
#ifndef USART3_MIRROR_USART1_RX
#define USART3_MIRROR_USART1_RX  0
#endif
#ifndef ODOM_USART3_FORWARD_RAW
#define ODOM_USART3_FORWARD_RAW  0
#endif

/* 部分 Dcar 需周期收到速度帧才持续回传 ODOM：ODOM 未成功时额外发 sendVel(0,0,0) 保活（与巡线 sendVel 并存一般可接受） */
#ifndef DCAR_ODOM_KEEPALIVE_SENDVEL
#define DCAR_ODOM_KEEPALIVE_SENDVEL  1
#endif

/* 0：仅在 Odom_Init 发一次「连续 ODOM 10Hz」请求（开话题即建立流，不必主循环重复发）；1：恢复旧行为周期性 requestOdomData + 5s 补发 */
#ifndef ODOM_REQUEST_PERIODIC
#define ODOM_REQUEST_PERIODIC  0
#endif

/* 回传帧第 2、3 字节：须为 0x97 与 0x01 两字节（顺序任意，Data_Anaylsis 已同时接受 97 01 与 01 97） */
#ifndef DF_RX_FRAME_ID1
#define DF_RX_FRAME_ID1  PC_ID
#endif
#ifndef DF_RX_FRAME_ID2
#define DF_RX_FRAME_ID2  ROBOT_ID
#endif
/* 校验和低字节在前；若 g_df_rx_chksum_fail 很大而 id_skip 很少，可试预定义为 1 */
#ifndef DF_RX_CHECKSUM_BIG_ENDIAN
#define DF_RX_CHECKSUM_BIG_ENDIAN  0
#endif

/* 调试：在 Watch 里看「有字节但解析不到」是 ID 不对、校验错还是 ODOM 帧少 */
#ifndef DF_ODOM_LINK_STATS
#define DF_ODOM_LINK_STATS  1
#endif
#if DF_ODOM_LINK_STATS
extern volatile u32 g_df_rx_complete_frames; /* 校验通过的完整帧数 */
extern volatile u32 g_df_rx_chksum_fail;     /* 校验失败次数 */
extern volatile u32 g_df_rx_odom_frame_ok;   /* 成功进入 Parse_OdomData1 次数 */
extern volatile u32 g_df_rx_id_skip;         /* ID 与 DF_RX_FRAME_ID1/2 不符而跳过的次数 */
extern volatile u32 g_df_rx_asm_parse_fail;  /* 组帧已凑满一包但 Data_Anaylsis 未解出任何有效帧 */
#endif

#endif
