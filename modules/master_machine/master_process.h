#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "robot_vision.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u
#define ACTION_DATA_LENGTH 16
#define SYN_DATA_LENGTH 16
#define CV_SEND_LENGTH 16

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef enum
{
    CRC_RIGHT=0,
    CRC_WRONG=1
} CRC_STATE;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的通信（支持 UART/VCP）
 * @param _handle 串口句柄（VCP 模式下传 NULL）
 * @return 接收缓冲区指针
 */
VisionRecvFrame_t* VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 * @param tx_frame 待发送的数据帧
 */
void VisionSend(VisionSendFrame_t *tx_frame);

/**
 * @brief 获取从视觉接收的数据
 * @return 接收帧指针
 */
VisionRecvFrame_t* VisionGetRecvData(void);

#endif // !MASTER_PROCESS_H