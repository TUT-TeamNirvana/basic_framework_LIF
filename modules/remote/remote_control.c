#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 图传链路数据，用于记录模拟DT7的右侧拨杆状态 (默认处于中档: 底盘跟随模式)
static uint8_t virtual_switch_right = RC_SW_MID; // 记忆当前底盘的开关状态
static uint8_t chassis_behavior = 0;             // 底盘模式: 默认 0 (底盘跟随), 1 (底盘不跟随), 2 (小陀螺 - 临时)
static uint8_t is_emergency_stop = 0;            // 使用 1 代表断电(急停)，0 代表正常工作
static uint8_t last_pause_btn = 0;               // 记忆上一次暂停键状态
static uint8_t last_custom_l = 0;                // 记忆上一次左自定义键状态
static uint8_t pause_cd = 0;                     // 暂停键冷却,防抖处理
static uint8_t custom_cd = 0;                    // 自定义键冷却,防抖处理

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static USARTInstance *vtx_usart_instance; // 新增图传的串口实例
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下
    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4  || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
    rc_ctrl[TEMP].lost_flag = 0; // 收到数据,清除掉线标志
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    rc_ctrl[TEMP].lost_flag = 1;         // 遥控器离线标志位
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}

/**
 * @brief 新图传接收端(VT13+VT3) 21字节协议解析函数
 */
static void vtx_to_rc(const uint8_t *vtx_buf)
{
    if (vtx_buf[0] != 0xA9 || vtx_buf[1] != 0x53) return;

    // 1. 摇杆和拨轮解算
    rc_ctrl[TEMP].rc.rocker_r_ = ((vtx_buf[2] | (vtx_buf[3] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_r1 = (((vtx_buf[3] >> 3) | (vtx_buf[4] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l1 = (((vtx_buf[4] >> 6) | (vtx_buf[5] << 2) | (vtx_buf[6] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l_ = (((vtx_buf[6] >> 1) | (vtx_buf[7] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.dial = (((vtx_buf[8] >> 1) | (vtx_buf[9] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    RectifyRCjoystick();

    // 2. 挡位切换开关 (C/N/S)
    uint8_t gear = (vtx_buf[7] >> 4) & 0x03;
    if (gear == 0)      rc_ctrl[TEMP].rc.switch_left = RC_SW_UP;
    else if (gear == 1) rc_ctrl[TEMP].rc.switch_left = RC_SW_MID;
    else if (gear == 2) rc_ctrl[TEMP].rc.switch_left = RC_SW_DOWN;

    // 3. 提取按键状态
    uint8_t pause_btn = (vtx_buf[7] >> 6) & 0x01;
    uint8_t custom_l  = (vtx_buf[7] >> 7) & 0x01;
    uint8_t trigger   = (vtx_buf[9] >> 4) & 0x01;

    // 冷却倒计时
    if (pause_cd > 0) pause_cd--;
    if (custom_cd > 0) custom_cd--;

    // 4. 防抖 Toggle 逻辑
    // 【暂停键】 -> 控制全车断电急停 (借用 lost_flag)
    if (pause_btn && !last_pause_btn && pause_cd == 0) {
        is_emergency_stop = !is_emergency_stop; // 翻转急停状态
        pause_cd = 10; // 冷却 10 帧 (约 140ms)，防止一次按下触发多次翻转
    }

    // 【左自定义键】 -> 控制底盘是否跟随 (切换右拨杆 MID/DOWN)
    if (custom_l && !last_custom_l && custom_cd == 0) {
        if (virtual_switch_right == RC_SW_DOWN) {
            virtual_switch_right = RC_SW_MID; // 切换回跟随
        } else {
            virtual_switch_right = RC_SW_DOWN; // 切换到不跟随(原版不跟随逻辑)
        }
        custom_cd = 10;
    }

    last_pause_btn = pause_btn;
    last_custom_l = custom_l;

    // 5. 将计算出的状态传给结构体
    rc_ctrl[TEMP].lost_flag = is_emergency_stop; // 触发原车的 EmergencyHandler 断电

    if (trigger) {
        rc_ctrl[TEMP].rc.switch_right = RC_SW_UP; // 扳机小陀螺
    } else {
        // 如果当前是 C 挡(左开关是上)，为了迎合DT7的robot_cmd里的if条件，强制把右开关也切为下
        if (rc_ctrl[TEMP].rc.switch_left == RC_SW_UP) {
            rc_ctrl[TEMP].rc.switch_right = RC_SW_DOWN;
        } else {
            rc_ctrl[TEMP].rc.switch_right = virtual_switch_right;
        }
    }

    // 6. 键鼠透传
    rc_ctrl[TEMP].mouse.x = (int16_t)(vtx_buf[10] | (vtx_buf[11] << 8));
    rc_ctrl[TEMP].mouse.y = (int16_t)(vtx_buf[12] | (vtx_buf[13] << 8));
    rc_ctrl[TEMP].mouse.press_l = vtx_buf[16] & 0x01;
    rc_ctrl[TEMP].mouse.press_r = (vtx_buf[16] >> 2) & 0x01;
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(vtx_buf[17] | (vtx_buf[18] << 8));

    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl)
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));

    if (rc_ctrl[TEMP].key[KEY_PRESS].shift)
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys;
    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4  || i == 5) continue;
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
}

// 图传独立串口接收回调
static void VTXRxCallback()
{
    DaemonReload(rc_daemon_instance);
    vtx_to_rc(vtx_usart_instance->recv_buff); // 使用图传解析逻辑
}

// 图传初始化注册函数 (在 robot_cmd.c 被调用)
void VTXControlInit(UART_HandleTypeDef *vtx_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = VTXRxCallback;
    conf.usart_handle = vtx_usart_handle;
    conf.recv_buff_size = 21; // 图传为21字节
    vtx_usart_instance = USARTRegister(&conf);
}