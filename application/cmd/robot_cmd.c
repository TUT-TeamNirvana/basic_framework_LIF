// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "robot_vision.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
#define shoot_frequency 8 //射击频率
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static VisionRecvFrame_t *vision_recv_data; // 视觉接收数据指针,初始化时返回

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态



void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#ifdef VISION_USE_VCP
    vision_recv_data = VisionInit(NULL); // USB虚拟串口模式
#else
    vision_recv_data = VisionInit(&huart6); // 串口模式
#endif
    extern void VTXControlInit(UART_HandleTypeDef *vtx_usart_handle);   //图传链路
    VTXControlInit(&huart1);    //图传链路串口配置

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.lid_mode = LID_CLOSE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.bullet_speed = BULLET_SPEED_NONE;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

//用于转换电机的真实角度
// int16_t map_value(float value, float *ori_scope, float *target_scope) {

//     float from_range = ori_scope[1] - ori_scope[0];
//     float to_range = target_scope[1] - target_scope[0];

//     float scaled_value = (value - ori_scope[0]) / from_range;
//     float result = target_scope[0] + scaled_value * to_range;

//     return result;
// }

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#endif
}

// int16_t CalcNowYawDirection (){
//         // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
//     static float angle;
//     angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
// #if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #endif
//     return (chassis_cmd_send.offset_angle*10000)/0.0174533;
// }

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */

static void RemoteControlSet()
{
    
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中]或[上],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))//右侧开关为上为小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    // 左侧开关状态为[下],遥控器控制下启动视觉调试
    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        // 使用新的视觉接收数据结构
        if (vision_recv_data != NULL)
        {
            InputData_t *vision_input = &vision_recv_data->input_data;

            if (vision_input->shoot_yaw != 0.0f || vision_input->shoot_pitch != 0.0f)
            {
                gimbal_cmd_send.yaw = vision_input->shoot_yaw;
                gimbal_cmd_send.pitch = vision_input->shoot_pitch;
                shoot_cmd_send.shoot_num = vision_input->fire;
                if (shoot_cmd_send.shoot_num == 1)
                {
                    shoot_cmd_send.load_mode = LOAD_VISION;
                }else if (shoot_cmd_send.shoot_num == 0)
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
            } else
            {
                if (rc_data[TEMP].lost_flag == 0 && robot_state != ROBOT_STOP)
                {
                    gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
                    gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
                }
            }
        } else
        {
            if (rc_data[TEMP].lost_flag == 0 && robot_state != ROBOT_STOP)
            {
                gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
                gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
            }
        }
        // 注意：新的协议中没有 reserved_slot 字段
        // 如果需要类似功能，需要在 InputData_t 中添加
        // 这里暂时注释掉相关代码
        // if (vision_recv_data->ACTION_DATA.reserved_slot / 10 == 2)
        // {
        //     shoot_cmd_send.load_mode = LOAD_REVERSE;
        //     shoot_cmd_send.shoot_rate = 4;
        //     shoot_cmd_send.shoot_num = 0;
        // }
        //
        // if (vision_recv_data->ACTION_DATA.reserved_slot % 10 == 2)
        // {
        //     chassis_cmd_send.vy = 10000;
        //      chassis_cmd_send.wz = 0;
        // }else if (vision_recv_data->ACTION_DATA.reserved_slot % 10 == 0)
        // {
        //     chassis_cmd_send.vy = 0;
        //     chassis_cmd_send.wz = 5000;
        // }else if (vision_recv_data->ACTION_DATA.reserved_slot % 10 == 1)
        // {
        //     chassis_cmd_send.vy = -10000;
        //     chassis_cmd_send.wz = 0;
        // }
    } else {
        if (rc_data[TEMP].lost_flag == 0 && robot_state != ROBOT_STOP)
        {
            gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
            gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
        }
        if (gimbal_cmd_send.pitch > 50)
        {
            gimbal_cmd_send.pitch = 50;
        }
        if (gimbal_cmd_send.pitch < -20)
        {
            gimbal_cmd_send.pitch = -20;
        }
        // 按照摇杆的输出大小进行角度增量,增益系数需调整

        // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
        chassis_cmd_send.vx = 100.0f * (float)rc_data[TEMP].rc.rocker_r_; // 右侧摇杆竖直方向控制x方向速度
        chassis_cmd_send.vy = 100.0f * (float)rc_data[TEMP].rc.rocker_r1; // 右侧摇杆水平方向控制y方向速度

    }

    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (shoot_cmd_send.friction_mode == FRICTION_ON)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        if (rc_data[TEMP].rc.dial > 100){
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
            shoot_cmd_send.shoot_num = 1;
            if (shoot_fetch_data.shoot_finish_flag == 1)
            {
                shoot_cmd_send.shoot_num = 0;
            }
            
        } 
        if (rc_data[TEMP].rc.dial < -100)
        {
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            shoot_cmd_send.shoot_rate = shoot_frequency;
            shoot_cmd_send.shoot_num = 0;
        }
        if ((rc_data[TEMP].rc.dial == 0) && (shoot_cmd_send.load_mode != LOAD_VISION) && (shoot_cmd_send.load_mode != LOAD_REVERSE)){
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    } 
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    if (chassis_fetch_data.chassis_power_limit == 70)
    {
        chassis_cmd_send.chassis_speed_buff = 20000;
    }
    else if(chassis_fetch_data.chassis_power_limit == 75)
    {
        chassis_cmd_send.chassis_speed_buff = 22000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 80)
    {
        chassis_cmd_send.chassis_speed_buff = 24000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 85)
    {
        chassis_cmd_send.chassis_speed_buff = 26000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 90)
    {
        chassis_cmd_send.chassis_speed_buff = 28000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 95)
    {
        chassis_cmd_send.chassis_speed_buff = 30000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 100)
    {
        chassis_cmd_send.chassis_speed_buff = 32000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 105)
    {
        chassis_cmd_send.chassis_speed_buff = 34000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 110)
    {
        chassis_cmd_send.chassis_speed_buff = 36000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 120)
    {
        chassis_cmd_send.chassis_speed_buff = 38000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 55)
    {
        chassis_cmd_send.chassis_speed_buff = 14000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 60)
    {
        chassis_cmd_send.chassis_speed_buff = 16000;
    }
    else if (chassis_fetch_data.chassis_power_limit == 65)
    {
        chassis_cmd_send.chassis_speed_buff = 18000;
    }
    else{
        chassis_cmd_send.chassis_speed_buff = 15000;
    }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_X] % 4) //手动选择底盘速度
    // {
    // case 0:
    //     break;
    // case 1:
    //     chassis_cmd_send.chassis_speed_buff = 15000;
    //     break;
    // case 2:
    //     chassis_cmd_send.chassis_speed_buff = 28000;
    //     break;
    // default:
    //     chassis_cmd_send.chassis_speed_buff = 35000;
    //     break;
    // }
    //设置默认控制模式
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.wz = rc_data[TEMP].key[KEY_PRESS].q*5000-rc_data[TEMP].key[KEY_PRESS].e*5000;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //底盘控制量设置
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].d * chassis_cmd_send.chassis_speed_buff - rc_data[TEMP].key[KEY_PRESS].a * chassis_cmd_send.chassis_speed_buff; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * chassis_cmd_send.chassis_speed_buff - rc_data[TEMP].key[KEY_PRESS].s * chassis_cmd_send.chassis_speed_buff;

    if (rc_data[TEMP].mouse.press_r == 1)
    {
        // 获取新接口传入的自瞄数据
        InputData_t *vision_input = &vision_recv_data->input_data;
        
        // 【新增安全判断】：如果视觉的yaw和pitch都是0（说明没扫到目标或者掉线），那就用普通的鼠标操作
        if (vision_input->shoot_yaw == 0.0f && vision_input->shoot_pitch == 0.0f)
        {
            gimbal_cmd_send.yaw -= 0.01f * (float)rc_data[TEMP].mouse.x;
            gimbal_cmd_send.pitch -= 0.01f * (float)rc_data[TEMP].mouse.y;
            shoot_cmd_send.load_mode = LOAD_STOP; // 没目标停火
        }
        else
        {
            // 视觉有输出数据时，接管云台姿态
            gimbal_cmd_send.yaw = vision_input->shoot_yaw;
            gimbal_cmd_send.pitch = vision_input->shoot_pitch;

            // 根据视觉的开火指令控制开火
            if (vision_input->fire == 1) {
                shoot_cmd_send.load_mode = LOAD_1_BULLET; // 或 LOAD_BURSTFIRE 看你需求
            } else {
                shoot_cmd_send.load_mode = LOAD_STOP;
            }
        }

        // 限幅防疯车
        if (gimbal_cmd_send.pitch > 50) gimbal_cmd_send.pitch = 50;
        if (gimbal_cmd_send.pitch < -20) gimbal_cmd_send.pitch = -20;
    } else {
        gimbal_cmd_send.yaw -= 0.01f * (float)rc_data[TEMP].mouse.x;
        gimbal_cmd_send.pitch -= 0.01f * (float)rc_data[TEMP].mouse.y;
        if (gimbal_cmd_send.pitch > 50)
        {
            gimbal_cmd_send.pitch = 50;
        }
        if (gimbal_cmd_send.pitch < -20)
        {
            gimbal_cmd_send.pitch = -20;
        }
    }


    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    
    default:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    }
    if (chassis_cmd_send.vx != 0 && chassis_cmd_send.vy != 0)
    {
        rc_data[TEMP].key_count[KEY_PRESS][Key_R] = 0;
    }
    
    // switch (rc_data[TEMP].key[KEY_PRESS].f) // F键开关摩擦轮
    // {
    // case 0:
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     shoot_cmd_send.bullet_speed = 0;
    //     break;
    
    // default:
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
    //     shoot_cmd_send.bullet_speed = 30;
    //     break;
    // }
    if (rc_data[TEMP].key[KEY_PRESS].f == 1)
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.bullet_speed = 30;
    }
    if (rc_data[TEMP].key[KEY_PRESS].g == 1)
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.bullet_speed = 0;
    }
    
    
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 小陀螺
    {
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;

    default:
        
        break;
    }

    
    switch (rc_data[TEMP].key[KEY_PRESS].ctrl)
    {
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    
    default:

        break;
    }
    if (rc_data[TEMP].mouse.press_l == 0)
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].c) // C键设置播弹盘反转
    {
    case 0:

        break;

    default:
        shoot_cmd_send.load_mode = LOAD_REVERSE;
        shoot_cmd_send.shoot_rate = shoot_frequency;
        shoot_cmd_send.shoot_num = 0;
        break;
    }
    if (rc_data[TEMP].mouse.press_l == 1)
    {
        switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z]%2) // z键设置发射模式
        {
        case 0:
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            shoot_cmd_send.shoot_rate = shoot_frequency;
            shoot_cmd_send.shoot_num = 0;
            break;
        
        case 1:
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
            shoot_cmd_send.shoot_num = 1;
            if (shoot_fetch_data.shoot_finish_flag == 1)
            {
                shoot_cmd_send.shoot_num = 0;
            }
            break;
        }
    }
    
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].lost_flag == 1 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (rc_data[TEMP].lost_flag == 0)
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
                LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)||switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[中]或[下],遥控器控制
    {    
        RemoteControlSet();
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制和相关模式选择
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_right))
        {
            MouseKeySet();
        }
        //添加弹速控制命令
        if(chassis_fetch_data.initial_speed/25 > 0.9 ){
            shoot_cmd_send.bullet_speed = SMALL_AMU_30;

        }
        if (switch_is_up(rc_data[TEMP].rc.switch_right) && rc_data[TEMP].rc.dial < -200)
        {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.bullet_speed = SMALL_AMU_30;
        }else if (switch_is_up(rc_data[TEMP].rc.switch_right) && rc_data[TEMP].rc.dial > 200)
        {
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.bullet_speed = BULLET_SPEED_NONE;
        }
        if (switch_is_mid(rc_data[TEMP].rc.switch_right) && rc_data[TEMP].rc.dial < -200)
        {
            shoot_cmd_send.lid_mode = LID_OPEN;
        } 
        if (switch_is_mid(rc_data[TEMP].rc.switch_right) && rc_data[TEMP].rc.dial > 200)
        {
            shoot_cmd_send.lid_mode = LID_CLOSE;
        }
    }
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
