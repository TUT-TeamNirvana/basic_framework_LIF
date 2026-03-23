//
// Created by Hikawon on 2026/3/19.
//

#ifndef BASIC_FRAMEWORK_ROBOT_VISION_H
#define BASIC_FRAMEWORK_ROBOT_VISION_H

#include <stdint.h>
#include <string.h>
#include "usart.h" // 包含UART_HandleTypeDef的定义
#include "crc_ref.h"

// 帧头和帧尾定义（与视觉端 structure.h 保持一致）
#pragma pack(push, 1)
typedef struct {
    uint8_t sof;
    uint8_t crc8;
} FrameHeader_t;

typedef struct {
    uint16_t crc16;
} FrameTailer_t;
#pragma pack(pop)


// 电控向视觉发送的数据结构,根据需要修改
#pragma pack(push, 1)
typedef struct {
    uint64_t config;     //可用于传递模式、参数等（如无需求可设0）
    float target_pose[3];     //目标三维坐标（如无目标可设0）
    float curr_yaw;     //当前云台偏航角度（从IMU或编码器获取）
    float curr_pitch;     //当前云台俯仰角度（从IMU或编码器获取）
    uint8_t enemy_color;     //敌方颜色（0蓝1红，与你视觉端定义一致）
    uint8_t shoot_config;     //射击配置（如弹速档位、单发/连发等）
} OutputData_t;

// 视觉向电控发送的数据结构,根据需要修改
typedef struct {
    float shoot_yaw;    // 经过自瞄预判的yaw角度
    float shoot_pitch;  // 经过自瞄预判的pitch角度
    uint8_t fire;       // 是否发弹
} InputData_t;

// 电控向视觉发送的帧结构
typedef struct {
    FrameHeader_t frame_header;
    OutputData_t output_data;
    FrameTailer_t frame_tailer;
} VisionSendFrame_t;

// 视觉向电控发送的帧结构
typedef struct {
    FrameHeader_t frame_header;
    InputData_t input_data;
    FrameTailer_t frame_tailer;
} VisionRecvFrame_t;
#pragma pack(pop)

// CRC 计算函数声明（使用 C 链接，方便调用 cpp 文件中的函数）
#ifdef __cplusplus
extern "C" {
#endif
    // CRC8 函数声明（与 modules/referee/crc_ref.c 保持一致）
    uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);
    void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
    uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
    // CRC16 函数声明（与 modules/referee/crc_ref.c 保持一致）
    uint16_t get_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
    void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
    uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif

// 视觉通信初始化（返回接收缓冲区指针）
VisionRecvFrame_t* VisionInit(UART_HandleTypeDef *_handle);

// 发送视觉数据
void VisionSend(VisionSendFrame_t *tx_frame);

// 获取接收到的数据（在回调中调用）
VisionRecvFrame_t* VisionGetRecvData(void);

#endif //BASIC_FRAMEWORK_ROBOT_VISION_H