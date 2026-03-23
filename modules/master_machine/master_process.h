#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "robot_vision.h"


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