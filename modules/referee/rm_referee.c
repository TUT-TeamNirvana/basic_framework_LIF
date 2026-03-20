/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				// 2个8位拼成16位int
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (referee_info.CmdID)
{
    case ID_game_state: // 0x0001
        memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
        // 以下代码为调试用,读取裁判系统协议内比赛状态数据,并使用H7tools发送至电脑
        // SEGGER_RTT_SetTerminal(1); // 设置显示的终端
        // sprintf(printf_buf, "Game Type=%d, Progress=%d, Stage Remain Time=%d, Sync Time Stamp=%llu\r\n",
        //         referee_info.GameState.game_type, referee_info.GameState.game_progress,
        //         referee_info.GameState.stage_remain_time, referee_info.GameState.SyncTimeStamp);
        // SEGGER_RTT_WriteString(0, printf_buf);
        break;

    case ID_game_result: // 0x0002
        memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
        // 以下代码为调试用,读取裁判系统协议内比赛结果数据,并使用H7tools发送至电脑
        // SEGGER_RTT_SetTerminal(1); // 设置显示的终端
        // sprintf(printf_buf, "Winner=%d\r\n", referee_info.GameResult.winner);
        // SEGGER_RTT_WriteString(0, printf_buf);
        break;

    case ID_game_robot_survivors: // 0x0003
        memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
        // 以下代码为调试用,读取裁判系统协议内机器人血量数据,并使用H7tools发送至电脑
        // SEGGER_RTT_SetTerminal(1); // 设置显示的终端
        // sprintf(printf_buf, "Red 1 Robot HP=%d, Red 2 Robot HP=%d, Red 3 Robot HP=%d, Red 4 Robot HP=%d, Red 7 Robot HP=%d, Red Outpost HP=%d, Red Base HP=%d\r\n"
        //                     "Blue 1 Robot HP=%d, Blue 2 Robot HP=%d, Blue 3 Robot HP=%d, Blue 4 Robot HP=%d, Blue 7 Robot HP=%d, Blue Outpost HP=%d, Blue Base HP=%d\r\n",
        //         referee_info.GameRobotHP.red_1_robot_HP, referee_info.GameRobotHP.red_2_robot_HP,
        //         referee_info.GameRobotHP.red_3_robot_HP, referee_info.GameRobotHP.red_4_robot_HP,
        //         referee_info.GameRobotHP.red_7_robot_HP, referee_info.GameRobotHP.red_outpost_HP,
        //         referee_info.GameRobotHP.red_base_HP,
        //         referee_info.GameRobotHP.blue_1_robot_HP, referee_info.GameRobotHP.blue_2_robot_HP,
        //         referee_info.GameRobotHP.blue_3_robot_HP, referee_info.GameRobotHP.blue_4_robot_HP,
        //         referee_info.GameRobotHP.blue_7_robot_HP, referee_info.GameRobotHP.blue_outpost_HP,
        //         referee_info.GameRobotHP.blue_base_HP);
        // SEGGER_RTT_WriteString(0, printf_buf);
        break;

	case ID_event_data: // 0x0101
		memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
		// 以下代码为调试用,读取裁判系统协议内场地事件数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Event Type=%d\r\n", referee_info.EventData.event_type);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	// case ID_supply_projectile_action: // 0x0102 2025.3.14删除，2024.12.25 V1.7.0 删除该命令码
	//     memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
	//     // 以下代码为调试用,读取裁判系统协议内场地补给站动作标识数据,并使用H7tools发送至电脑
	//     SEGGER_RTT_SetTerminal(1); // 设置显示的终端
	//     sprintf(printf_buf, "Supply Robot ID=%d, Supply Projectile Step=%d, Supply Projectile Num=%d\r\n",
	//             referee_info.SupplyProjectileAction.supply_robot_id,
	//             referee_info.SupplyProjectileAction.supply_projectile_step,
	//             referee_info.SupplyProjectileAction.supply_projectile_num);
	//     SEGGER_RTT_WriteString(0, printf_buf);
	//     break;

	case ID_referee_warning: // 0x0104 2025.3.14添加
		memcpy(&referee_info.RefereeWarning, (buff + DATA_Offset), LEN_referee_warning);
		// 以下代码为调试用,读取裁判系统协议内裁判警告数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Warning Level=%d, Offending Robot ID=%d, Count=%d\r\n",
		// 		referee_info.RefereeWarning.level, referee_info.RefereeWarning.offending_robot_id,
		// 		referee_info.RefereeWarning.count);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	case ID_dart_info: // 0x0105 2025.3.14添加
		memcpy(&referee_info.DartInfo, (buff + DATA_Offset), LEN_dart_info);
		// 以下代码为调试用,读取裁判系统协议内飞镖发射数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Dart Remaining Time=%d, Dart Info=%d\r\n",
		// 		referee_info.DartInfo.dart_remaining_time, referee_info.DartInfo.dart_info);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	case ID_game_robot_state: // 0x0201
		memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
		// 以下代码为调试用,读取裁判系统协议内机器人状态数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Robot ID=%d, Robot Level=%d, Current HP=%d, Max HP=%d, Shooter Barrel Cooling Value=%d, "
		// 					"Shooter Barrel Heat Limit=%d, Chassis Power Limit=%d, Power Management Gimbal Output=%d, "
		// 					"Power Management Chassis Output=%d, Power Management Shooter Output=%d\r\n",
		// 		referee_info.GameRobotState.robot_id, referee_info.GameRobotState.robot_level,
		// 		referee_info.GameRobotState.current_HP, referee_info.GameRobotState.maximum_HP,
		// 		referee_info.GameRobotState.shooter_barrel_cooling_value,
		// 		referee_info.GameRobotState.shooter_barrel_heat_limit,
		// 		referee_info.GameRobotState.chassis_power_limit,
		// 		referee_info.GameRobotState.power_management_gimbal_output,
		// 		referee_info.GameRobotState.power_management_chassis_output,
		// 		referee_info.GameRobotState.power_management_shooter_output);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	case ID_power_heat_data: // 0x0202
		memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
		// 以下代码为调试用,读取裁判系统协议内全底盘参数,并使用H7tools发送电机功率至电脑
		// float chassispower_temp = referee_info.PowerHeatData.buffer_energy; // 电机功率临时显示，水评测用
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Chassis Power=%f\r\n", chassispower_temp);
		// SEGGER_RTT_WriteString(0, printf_buf);
		// RTT_PrintWave_np(1, chassispower_temp);
		break;

	case ID_game_robot_pos: // 0x0203
		memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
		// 以下代码为调试用,读取裁判系统协议内机器人位置数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Robot X=%f, Robot Y=%f, Robot Angle=%f\r\n",
		// 		referee_info.GameRobotPos.x, referee_info.GameRobotPos.y, referee_info.GameRobotPos.angle);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	case ID_buff_musk: // 0x0204
		memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
		// 以下代码为调试用,读取裁判系统协议内机器人增益数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Recovery Buff=%d, Cooling Buff=%d, Defence Buff=%d, Vulnerability Buff=%d, Attack Buff=%d, Remaining Energy=%d\r\n",
		// 		referee_info.BuffMusk.recovery_buff, referee_info.BuffMusk.cooling_buff,
		// 		referee_info.BuffMusk.defence_buff, referee_info.BuffMusk.vulnerability_buff,
		// 		referee_info.BuffMusk.attack_buff, referee_info.BuffMusk.remaining_energy);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	// case ID_aerial_robot_energy: // 0x0205 2025.3.14删除，2024.12.25 V1.7.0 删除该命令码
	//     memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
	//     // 以下代码为调试用,读取裁判系统协议内空中机器人能量状态数据,并使用H7tools发送至电脑
	//     SEGGER_RTT_SetTerminal(1); // 设置显示的终端
	//     sprintf(printf_buf, "Airforce Status=%d, Time Remain=%d\r\n",
	//             referee_info.AerialRobotEnergy.airforce_status, referee_info.AerialRobotEnergy.time_remain);
	//     SEGGER_RTT_WriteString(0, printf_buf);
	//     break;

	case ID_robot_hurt: // 0x0206
		memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
		// 以下代码为调试用,读取裁判系统协议内伤害状态数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Armor ID=%d, HP Deduction Reason=%d\r\n",
		// 		referee_info.RobotHurt.armor_id, referee_info.RobotHurt.HP_deduction_reason);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	case ID_shoot_data: // 0x0207
		memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
		// 以下代码为调试用,读取裁判系统协议内实时射击数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Bullet Type=%d, Shooter Number=%d, Launching Frequency=%d, Initial Speed=%f\r\n",
		// 		referee_info.ShootData.bullet_type, referee_info.ShootData.shooter_number,
		// 		referee_info.ShootData.launching_frequency, referee_info.ShootData.initial_speed);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;

	case ID_shoot_num: // 0x0208    射击次数
		memcpy(&referee_info.ShootNumAndGoldCoin, (buff + DATA_Offset), LEN_shoot_num);
		// 以下代码为调试用,读取裁判系统协议内发弹量数据,并使用H7tools发送至电脑
		// SEGGER_RTT_SetTerminal(1); // 设置显示的终端
		// sprintf(printf_buf, "Projectile Allowance 17mm=%d, Projectile Allowance 42mm=%d, Remaining Gold Coin=%d\r\n",
		// 		referee_info.ShootNumAndGoldCoin.projectile_allowance_17mm,
		// 		referee_info.ShootNumAndGoldCoin.projectile_allowance_42mm,
		// 		referee_info.ShootNumAndGoldCoin.remaining_gold_coin);
		// SEGGER_RTT_WriteString(0, printf_buf);
		break;
	}
				// case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
				// 	memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
				// 	break;
				// }
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);

		}
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}
