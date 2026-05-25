/**
 * @file ir_grayscale.c
 * @brief 亚博智能8路灰度传感器串口协议解析
 * 
 * 数据格式：$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
 * 数字型数据，每个xi为0或1
 */
#include "ir_grayscale.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

#define IR_PACKAGE_SIZE 100

static u8 rx_buff[IR_PACKAGE_SIZE];
static u8 new_package[IR_PACKAGE_SIZE];

u8 IR_Data_number[IR_NUM];
u8 g_ir_new_package_flag = 0;

static u8 g_dmode_data = 0;  // 数字型模式标志（内部使用）

/**
 * @brief 逐字节接收并组包
 * 在 HAL 中，通常在 HAL_UART_RxCpltCallback 中调用
 */
void Deal_IR_Usart(u8 rxtemp)
{
	static u8 g_start = 0;
	static u8 step = 0;

	if (rxtemp == '$')
	{
		g_start = 1;
		rx_buff[step] = rxtemp;
		step++;
	}
	else
	{
		if (g_start == 0)
			return;

		rx_buff[step] = rxtemp;
		step++;

		if (rxtemp == '#')  // 帧结束
		{
			g_start = 0;
			step = 0;
			memcpy(new_package, rx_buff, IR_PACKAGE_SIZE);
			g_ir_new_package_flag = 1;
			memset(rx_buff, 0, IR_PACKAGE_SIZE);
		}

		if (step >= IR_PACKAGE_SIZE)  // 异常，防溢出
		{
			g_start = 0;
			step = 0;
			memset(rx_buff, 0, IR_PACKAGE_SIZE);
		}
	}
}

/**
 * @brief 发送配置命令到8路灰度传感器
 * @param adjust 1=校准
 * @param aData  1=输出模拟值
 * @param dData  1=输出数字值（巡线常用）
 */
void IR_Send_Control_Data(u8 adjust, u8 aData, u8 dData)
{
	u8 send_buf[8] = "$0,0,0#";

	if (adjust == 1)
		send_buf[1] = '1';
	else
		send_buf[1] = '0';

	if (aData == 1)
	{
		send_buf[3] = '1';
	}
	else
		send_buf[3] = '0';

	if (dData == 1)
	{
		send_buf[5] = '1';
		g_dmode_data = 1;
	}
	else
	{
		send_buf[5] = '0';
		g_dmode_data = 0;
	}

	HAL_UART_Transmit(&huart2, send_buf, 7, 40);  /* 7 字节 @115200 远小于 40ms；缩短避免拖慢巡线主循环 */
}

/**
 * @brief 解析数字型数据到 IR_Data_number[]
 * 数据格式：$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
 * 每个xi在位置 6+i*5
 */
u8 Deal_IR_Usart_Data(void)
{
	u8 i;
	char c;

	/* 仅在配置为数字型输出时解析；完整帧须为 $D,...#，避免误把模拟帧当「已更新」导致 s_sensor_valid 锁死错误 pattern */
	if (!g_dmode_data)
		return 0u;
	if (new_package[0] != '$' || new_package[1] != 'D')
		return 0u;

	for (i = 0; i < IR_NUM; i++) {
		c = (char)new_package[6 + i * 5];
		if (c != '0' && c != '1')
			return 0u;
		IR_Data_number[i] = (u8)(c - '0');
#if IR_GRAYSCALE_INVERT_DIGITAL
		IR_Data_number[i] = (u8)(1u - IR_Data_number[i]);
#endif
	}

	memset(new_package, 0, IR_PACKAGE_SIZE);
	return 1u;
}
