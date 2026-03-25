#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "robot_vision.h"


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
 * @version 0.1
 * @date 2023-03-11
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "usart.h"
#include "dma.h"
#include "INS_task.h"
#include "arm_math.h"
#include "remote_control.h"
//#include "referee.h"

/*  **人工修正的参数**  */

//云台转轴中心到枪口的竖直距离
#define Z_STATIC -0.01f
//云台转轴中心到发射最大初速度点的距离
#define DISTANCE_STATIC 0.02f
//枪管pitch歪(相较于正确安装的误差)
#define PITCH_STATIC 0.0f
//子弹类型 小弹丸 0，大弹丸 1, 发光大弹丸 2
#define BULLET_TYPE 0

//云台yaw轴模式，置0跟随模式，置1静止瞄准中间模式
#define AUTO_GIMBAL_YAW_MODE 0
// 选择装甲板的方向，置0 顺着选，置1迎着选
#define SELECT_ARMOR_DIR 1

//重力加速度
#define GRAVITY 9.8f

//固有时间偏移--主要是测试发射子弹的延迟
#define TIME_BIAS 15

//允许发弹距离 m
#define ALLOW_ATTACK_DISTANCE 10.0f
//允许发弹的ekf收敛值
#define ALLOE_ATTACK_P 2.0f

/*    **      **     */

//装甲板选择的速度阈值
#define SELECT_ARMOR_V_YAW_THRES 1.5f
//延时等待
#define VISION_SEND_TASK_INIT_TIME 401
//系统延时时间
#define VISION_SEND_CONTROL_TIME_MS 1

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 1
//空气阻力系数来自tup开源
#if (BULLET_TYPE == 0)
// 空气阻力系数简化版
#define AIR_K1 0.019f//0.2f
#elif (BULLET_TYPE == 1)
// 空气阻力系数简化版
#define AIR_K1 0.00556f
#else
// 空气阻力系数简化版
#define AIR_K1 0.00530f
#endif

//弧度制转角度制
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//机器人红蓝id分界值，大于该值则机器人自身为蓝色，小于这个值机器人自身为红色
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100.0f

//最小设定弹速
#define MIN_SET_BULLET_SPEED 20.1f
//最大设定弹速
#define MAX_SET_BULLET_SPEED 25.0f
//初始设定弹速
#define BEGIN_SET_BULLET_SPEED 22.3f

//大装甲板宽度
#define SMALL_ARMOR_WIDTH 0.135f
//小装甲板宽度
#define LARGE_ARM0R_WIDTH 0.230f
//装甲板高度
#define ARMOR_HIGH 0.120f

//初始子弹飞行迭代数值
#define T_0 0.0f
//迭代精度
#define PRECISION 0.0001f
//最小迭代差值
#define MIN_DELTAT 0.001f
//最大迭代次数
#define MAX_ITERATE_COUNT 30
//比例迭代器比例系数
#define ITERATE_SCALE_FACTOR 0.9f
//RK4迭代次数--越大越精准
#define RK_ITER 60

//ms转s
#ifndef TIME_MS_TO_S
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)
#endif // !TIME_MS_TO_S(x)

//全圆弧度
#define ALL_CIRCLE (2 * PI)
//初始飞行时间
#define INIT_FILIGHT_TIME 0.5f
//最大未接受数据的时间 s
#define MAX_NOT_RECEIVE_DATA_TIME 3.0f




////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

typedef float fp32;

typedef uint8_t bool_t;


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////




//子弹类型
typedef enum
{
    BULLET_17 = 1,
    BULLET_42 = 2,
}bullet_type_e;

// 发射枪管id
typedef enum
{
    SHOOTER_17_1 = 1,
    SHOOTER_17_2 = 2,
    SHOOTER_42 = 3,
} shooter_id_e;

//接收数据状态
typedef enum
{
    //未读取
    UNLOADED,
    //已读取
    LOADED,
}receive_state_e;


//数据起始帧类型
typedef enum
{
    //下位机发送到上位机
    LOWER_TO_HIGH_HEAD = 0x5A,
    //上位机发送到下位机
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//装甲板颜色
typedef enum
{
    RED = 0,
    BLUE = 1,
}robot_armor_color_e;


typedef enum
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7,
    //全部机器人
    ARMOR_ALL_ROBOT = 8,
}armor_id_e;

//哨兵发射命令//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
typedef enum
{
    SHOOT_ATTACK,       // 袭击
    SHOOT_READY_ATTACK, // 准备袭击
    SHOOT_STOP_ATTACK,  // 停止袭击
} shoot_command_e;

// 视觉目标状态
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
typedef enum
{
    TARGET_UNAPPEAR, // 未识别到目标
    TARGET_APPEAR,   // 识别到目标
} vision_target_appear_state_e;

//向量结构体
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
    fp32 r;
} vector_t;

// 发送数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t detect_color : 1; // 0-red 1-blue
    bool_t reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;
} send_packet_t;

// 接收数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
typedef struct __attribute__((packed))
{
    uint8_t header;
    bool_t tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // 装甲板数量 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    float p;   //状态协方差矩阵的迹
    float latency_time;
    uint16_t checksum;
} receive_packet_t;

//类型转化
typedef receive_packet_t target_data_t;

// 视觉接收结构体
typedef struct
{
    // 接收状态位
    uint8_t receive_state : 1;
    // 上次接收数据的时间
    fp32 last_receive_time;
    // 当前接受数据时间
    fp32 current_receive_time;

    // 当前时间 -- 用于计算是否长时间未接受
    fp32 current_time;
    //  间隔时间
    fp32 interval_time;
    // 接收数据包
    receive_packet_t receive_packet;
} vision_receive_t;

// 哨兵云台电机运动命令,经滤波处理后的数值
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
typedef struct
{
    // 本次云台yaw轴数值
    fp32 gimbal_yaw;
    // 本次云台pitch轴数值
    fp32 gimbal_pitch;
    //角速度前馈
    fp32 feed_forward_omega;
} gimbal_vision_control_t;

// 哨兵发射电机运动控制命令
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
typedef struct
{
    // 自动发射命令
    shoot_command_e shoot_command;
} shoot_vision_control_t;


// 目标位置结构体
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} target_position_t;


//弹道计算结构体
typedef struct
{
    // 当前弹速
    fp32 current_bullet_speed;
    // 弹道系数
    fp32 k1;
    //子弹飞行时间
    fp32 flight_time;
    // 固有间隔时间
    fp32 time_bias;
    //预测时间
    fp32 predict_time;

    // 目标yaw
    fp32 target_yaw;
    // 装甲板数量
    uint8_t armor_num;
    //IMU到yaw轴电机的竖直距离
    fp32 z_static;
    //枪口前推距离
    fp32 distance_static;
    //由于枪管安装问题产生的pitch的安装误差
    fp32 pitch_static;

    //所有装甲板位置
    target_position_t all_target_position_point[4];

} solve_trajectory_t;

//均值弹速
#define BULLET_SPEED_SIZE 5
typedef struct{
    fp32 bullet_speed[BULLET_SPEED_SIZE];
    fp32 est_bullet_speed;
}bullet_speed_t;

// 视觉任务结构体
typedef struct
{
    // 绝对角指针
    const INS_t* vision_angle_point;
    // 弹速
    bullet_speed_t bullet_speed;
    // 检测装甲板的颜色(敌方装甲板的颜色)
    uint8_t detect_armor_color;


    //目标数据
    target_data_t target_data;
    //上次目标数据
    target_data_t last_target_data;

    //弹道解算
    solve_trajectory_t solve_trajectory;

    // 机器人云台瞄准位置向量
    vector_t robot_gimbal_aim_vector;
    // 以机器人自身为原点在惯性系下敌方机器人的yaw角
    fp32 body_to_enemy_robot_yaw;
    // 云台欲瞄准装甲板的角度
    gimbal_vision_control_t aim_armor_angle;

    //接收的数据包指针
    vision_receive_t* vision_receive_point;
    //发送数据包
    send_packet_t send_packet;

    // 视觉目标状态
    vision_target_appear_state_e vision_target_appear_state;
    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;
    // 发射机构发射命令
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

// 视觉数据处理任务
void vision_task(void const *pvParameters);

/**
 * @brief 接收数据解码
 *
 * @param buf 接收到的数据
 * @param len 接收到的数据长度
 */
void receive_decode(uint8_t* buf, uint32_t len);

/**
 * @brief 将数据包通过usb发送到nuc
 *
 * @param send 发送数据包
 */
void send_packet(vision_control_t* send);

extern vision_control_t vision_control;
extern vision_receive_t vision_receive;

/*       对外接口      */

// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void);

// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void);

/**
 * @brief 判断视觉是否识别到目标
 *
 * @return bool_t 返回1 识别到目标 返回0 未识别到目标
 */
bool_t judge_vision_appear_target(void);

/*    **      **     */

#endif // !VISION_TASK_H







// 视觉任务初始化
extern void vision_task_init(vision_control_t* init);
// 视觉任务数据更新
extern void vision_task_feedback_update(vision_control_t* update);
// 设置目标装甲板颜色
extern void vision_set_target_armor_color(vision_control_t* set_detect_color, uint8_t cur_robot_id);
// 判断是否识别到目标
extern void vision_judge_appear_target(vision_control_t* judge_appear_target);
// 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
extern void vision_data_process(vision_control_t* vision_data);
// 配置发送数据包
extern void set_vision_send_packet(vision_control_t* set_send_packet);



// 初始化弹道解算的参数
extern void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static, fp32 pitch_static);
// 计算弹道落点 -- 单方向空间阻力模型
extern float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// 计算弹道落点 -- 完全空气阻力模型
extern float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// 计算弹道落点 -- RK4
extern float calc_bullet_drop_in_RK4(solve_trajectory_t* solve_trajectory, float x, float z, float bullet_speed, float theta);
// 二维平面弹道模型，计算pitch轴的高度
extern float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, fp32 theta_offset, int mode);
//更新弹速，并估计当前弹速
extern void update_bullet_speed(bullet_speed_t *bullet_speed, fp32 cur_bullet_speed);
//判断发弹
extern void vision_shoot_judge(vision_control_t* shoot_judge);

// 获取接收数据包指针
extern vision_receive_t* get_vision_receive_point(void);




//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////






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