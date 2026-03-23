#include "daemon.h"
#include "bsp_log.h"
#include <string.h>
#include "master_process.h"
#include "robot_def.h"
//#include "crc.h"
//#include "bsp_dwt.h"

// 根据条件编译选择包含不同的头文件
#ifdef VISION_USE_VCP
#include "bsp_usb.h"
#else
#include "bsp_usart.h"
#endif

// ========== 公共变量定义 ==========
static VisionSendFrame_t send_frame;      // 电控->视觉（对应视觉端的 StateBytes）
static VisionRecvFrame_t recv_frame;      // 视觉->电控（对应视觉端的 OperateBytes）
static DaemonInstance *vision_daemon;

#ifdef VISION_USE_UART
static USARTInstance *vision_usart;
#endif

#ifdef VISION_USE_VCP
static uint8_t *vis_recv_buff;  // VCP 接收缓冲区
#endif


// ========== 公共宏定义 ==========
#define VISION_SOF 0xA5 // 帧头，视觉端 structure.h 中定义为 SOF_BYTE
#define CRC8_INIT 0xff
#define CRC16_INIT 0xffff
#define FRAME_TAIL '\n' // 帧尾，视觉端 receive_thread.cpp 中检查

// ========== 公共函数实现 ==========

/**
 * @brief 打包发送给视觉的帧（电控->视觉）
 * @param frame 待发送的数据帧
 * @param send_buff 发送缓冲区
 * @param tx_len 返回的总长度
 *
 * 帧结构：[SOF][CRC8][OutputData(30bytes)][CRC16][\n]
 * 总长度：2 + 30 + 2 + 1 = 35 bytes
 *
 * 注意：
 * - CRC8 字段存在但不计算（视觉端不校验）
 * - 必须添加\n 帧尾（视觉端 receive_thread 会检查）
 */
static void pack_send_frame(VisionSendFrame_t *frame, uint8_t *send_buff, uint16_t *tx_len)
{
    // 设置 SOF
    frame->frame_header.sof = VISION_SOF;

    // CRC8 字段保留（但不计算，视觉端不校验）
    frame->frame_header.crc8 = 0;

    // 计算并添加 CRC16 (对整个帧：header + output_data + tailer)
    // 注意：append_crc16_check_sum 会直接修改 frame->frame_tailer.crc16
    uint32_t data_len = sizeof(OutputData_t);
    append_crc16_check_sum((uint8_t*)frame, sizeof(FrameHeader_t) + data_len + sizeof(FrameTailer_t));

    // 拷贝数据到发送缓冲区
    memcpy(send_buff, frame, sizeof(VisionSendFrame_t));

    // 添加帧尾 \n（视觉端 receive_thread.cpp 会检查）
    send_buff[sizeof(VisionSendFrame_t)] = FRAME_TAIL;

    // 设置总长度
    *tx_len = sizeof(VisionSendFrame_t) + 1;  // 数据帧 + 帧尾（\n）
}

/**
 * @brief 解包从视觉接收的帧（视觉->电控）
 * @param rx_buff 串口接收缓冲区
 * @param frame 解析后的数据帧
 * @retval 0: 成功，1: CRC16 错误，2: SOF 错误
 *
 * 帧结构：[SOF][CRC8][InputData(9bytes)][CRC16]
 * 总长度：2 + 9 + 2 = 13 bytes（无\n 帧尾）
 *
 * 注意：
 * - 视觉端 control.cpp 发送时没有加\n
 * - CRC8 字段存在但视觉端没有计算
 */
static uint8_t unpack_recv_frame(uint8_t *rx_buff, VisionRecvFrame_t *frame)
{
    // 检查 SOF
    if (rx_buff[0] != VISION_SOF) {
        LOGWARNING("[Vision] Invalid SOF: 0x%02X", rx_buff[0]);
        return 2;
    }

    // 拷贝数据（不拷贝帧尾）
    memcpy(frame, rx_buff, sizeof(VisionRecvFrame_t));

    // 验证 CRC16 (整个帧，不包含帧尾)
    if (!verify_crc16_check_sum(rx_buff, sizeof(VisionRecvFrame_t))) {
        LOGWARNING("[Vision] CRC16 verification failed");
        return 1;
    }

    return 0;
}


/**
 * @brief 离线回调函数
 */
static void VisionOfflineCallback(void *id)
{
    // 清除过期视觉数据，防止拨到S挡时受残留指令影响导致云台回正或不可控
    memset(&recv_frame, 0, sizeof(VisionRecvFrame_t));
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart);
    LOGWARNING("[Vision] offline, restart UART communication");
#else
    // VCP 模式下重启 USB 通信
    LOGWARNING("[Vision] offline, restart VCP communication");
#endif
}


#ifdef VISION_USE_UART

/**
 * @brief 串口接收回调函数( 仅UART模式 )
 */
static void VisionRxCallback(void)
{
    // 注意：recv_buff_size 应该设置为 sizeof(VisionRecvFrame_t)
    // 因为视觉端发送时没有加\n
    uint8_t result = unpack_recv_frame(vision_usart->recv_buff, &recv_frame);

    if (result == 0) {
        DaemonReload(vision_daemon);  // 数据正确，喂狗
    } else {
        switch (result) {
            case 1:
                LOGWARNING("[Vision] CRC16 error");
                break;
            case 2:
                LOGWARNING("[Vision] SOF error: 0x%02X", vision_usart->recv_buff[0]);
                break;
        }
    }
}

/**
 * @brief 初始化视觉通信 - UART 模式
 * @param _handle 串口句柄
 * @return 接收缓冲区指针（视觉->电控的数据）
 */
VisionRecvFrame_t* VisionInit(UART_HandleTypeDef *_handle)
{
    // 配置串口
    USART_Init_Config_s usart_conf = {
        .usart_handle = _handle,
        .recv_buff_size = sizeof(VisionRecvFrame_t),  // 13 bytes（视觉端发送无\n）
        .module_callback = VisionRxCallback
    };
    vision_usart = USARTRegister(&usart_conf);

    // 注册 daemon
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = vision_usart,
        .reload_count = 10
    };
    vision_daemon = DaemonRegister(&daemon_conf);

    // 清空缓冲区
    memset(&send_frame, 0, sizeof(VisionSendFrame_t));
    memset(&recv_frame, 0, sizeof(VisionRecvFrame_t));

    return &recv_frame;
}


/**
 * @brief 发送数据给视觉 - UART 模式
 * @param tx_frame 待发送的数据帧
 */
void VisionSend(VisionSendFrame_t *tx_frame)
{
    static uint8_t send_buff[sizeof(VisionSendFrame_t) + 1];
    static uint16_t tx_len;

    // 打包帧（自动计算并填充 CRC16，添加帧尾）
    pack_send_frame(tx_frame, send_buff, &tx_len);

    // 通过串口发送（使用 DMA）
    USARTSend(vision_usart, send_buff, tx_len, USART_TRANSFER_DMA);
}

#endif // VISION_USE_UART


#ifdef VISION_USE_VCP

/**
 * @brief USB 接收回调函数（仅 VCP 模式）
 * @param len 接收到的数据长度（USB 回调传入的参数）
 */
static void DecodeVision(uint16_t len)
{
    UNUSED(len);  // 不使用该参数，因为数据已经在全局缓冲区 vis_recv_buff 中
    //LOGINFO("RX len: %d", len);
    /*LOGINFO("RX HEX: %02X %02X %02X %02X %02X %02X",
           vis_recv_buff[0], vis_recv_buff[1], vis_recv_buff[2],
           vis_recv_buff[3], vis_recv_buff[4], vis_recv_buff[5]);*/
    // 从 USB 接收缓冲区解包数据
    // 注意：视觉端发送时没有加\n，所以直接解包 sizeof(VisionRecvFrame_t) 字节
     uint8_t result = unpack_recv_frame(vis_recv_buff, &recv_frame);
    if (result == 0) {
        DaemonReload(vision_daemon);  // 数据正确，回传给进程守护
    } else {
        LOGWARNING("[Vision VCP] Frame error: %d", result);
    }
}


/**
 * @brief 初始化视觉通信 - VCP 模式
 * @param _handle 未使用（传入 NULL）
 * @return 接收缓冲区指针（视觉->电控的数据）
 */
VisionRecvFrame_t* VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // VCP 模式不使用串口句柄

    USB_Init_Config_s conf = {
        .rx_cbk = DecodeVision,
        .tx_cbk = NULL
    };
    vis_recv_buff = USBInit(conf);

    // 注册 daemon
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,  // VCP 没有 USARTInstance
        .reload_count = 50,
    };
    vision_daemon = DaemonRegister(&daemon_conf);

    // 清空缓冲区
    memset(&send_frame, 0, sizeof(VisionSendFrame_t));
    memset(&recv_frame, 0, sizeof(VisionRecvFrame_t));

    return &recv_frame;
}


/**
 * @brief 发送数据给视觉 - VCP 模式
 * @param tx_frame 待发送的数据帧
 */
void VisionSend(VisionSendFrame_t *tx_frame)
{
    static uint8_t send_buff[sizeof(VisionSendFrame_t) + 1];  // 必须 +1 用于\n
    static uint16_t tx_len;

    // 打包帧（必须添加\n，因为视觉端会检查）
    pack_send_frame(tx_frame, send_buff, &tx_len);

    // 使用 USB 发送（包含\n）
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP


/**
 * @brief 获取从视觉接收的数据（视觉->电控）
 * @return 接收帧指针
 */
VisionRecvFrame_t* VisionGetRecvData(void)
{
    return &recv_frame;
}