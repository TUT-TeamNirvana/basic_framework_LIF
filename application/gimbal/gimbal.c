#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_log.h"
#include "bmi088.h"
#include "master_process.h"
#include "robot_vision.h"
#include <math.h>
#include "master_process.h"


static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;
//static float pitch_gravity_feedforward = 0.0f; //pitch前馈重力补偿
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;     // cmd控制消息订阅者

Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息

static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Subscriber_t *chassis_sub;     // 底盘裁判系统数据订阅者
static Chassis_Upload_Data_s chassis_refe_data; // 底盘裁判系统数据
extern VisionSendFrame_t vision_send_frame; // 云台视觉发送数据帧（新协议）


void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 4,
                .Kd = 1,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
            .speed_feedforward_ptr = &gimbal_cmd_recv.yaw_speed_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD,
        },
        .motor_type = GM6020};
    // PITCH，参数x减速比3.4
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 34, // 10
                .Ki = 1.7,
                .Kd = 0,
                .DeadBand = 0.2,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 340,
                .MaxOut = 1700,
            },
            .speed_PID = {
                .Kp = 15,  // 50
                .Ki = 110, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
            //.current_feedforward_ptr = &pitch_gravity_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };

    // 初始化视觉通信（UART 或 VCP模式）
#ifdef VISION_USE_VCP
    VisionInit(NULL);  // USB虚拟串口模式
#else
    VisionInit(&huart6);  // UART模式，使用 uart6
#endif

    // 初始化视觉任务（依赖 IMU 和视觉通信已初始化）
    vision_task_init(&vision_control);




    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(chassis_sub, &chassis_refe_data);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED); // 恢复为电机自身的多圈编码器反馈
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // PITCH 轴使用编码器位置环, 加入3.4的减速比, 且 30.0f 为水平时的编码器绝对角度
        DJIMotorSetRef(pitch_motor, (-gimbal_cmd_recv.pitch * 3.4f) + 30.0f);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED); // 恢复为电机自身的多圈编码器反馈
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // PITCH 轴使用编码器位置环, 加入3.4的减速比, 且 30.0f 为水平时的编码器绝对角度
        DJIMotorSetRef(pitch_motor, (-gimbal_cmd_recv.pitch * 3.4f) + 30.0f);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // 此处的 700.0f 为推测前馈值,仅新熊猫用重力补偿，老熊猫不启用重力补偿
    //pitch_gravity_feedforward = 700.0f * cosf(gimba_IMU_data->Pitch * 3.14159f / 180.0f);

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    
    // 更新发送给视觉的数据（每个周期都更新）
//    vision_send_frame.frame_header.sof = 0xA5;  // 帧头
//    vision_send_frame.output_data.curr_yaw = gimbal_feedback_data.gimbal_imu_data.Yaw * 0.0174532925f;
//    vision_send_frame.output_data.curr_pitch = gimbal_feedback_data.gimbal_imu_data.Pitch * 0.0174532925f;
    // 如果需要，还可以更新其他字段
    // vision_send_frame.output_data.enemy_color = chassis_refe_data.enemy_color;
    // vision_send_frame.output_data.shoot_config = 某个弹速值;
    // vision_send_frame.output_data.target_pose[0] = 0.0f;
    // vision_send_frame.output_data.target_pose[1] = 0.0f;
    // vision_send_frame.output_data.target_pose[2] = 0.0f;

//    VisionSend(&vision_send_frame);



//////////////////////////////////////////////////////--psy_vision

        // 更新数据---这里面有弹速要人工加弹速变量
        vision_task_feedback_update(&vision_control);
        // 设置目标装甲板颜色--人工加自身机器人id
        vision_set_target_armor_color(&vision_control, 04);
        // 判断是否识别到目标
        vision_judge_appear_target(&vision_control);
        // 处理上位机数据,计算弹道的空间落点，并反解空间绝对角,并设置控制命令
        vision_data_process(&vision_control);
        // 配置发送数据包
        set_vision_send_packet(&vision_control);
        // 发送数据包
        send_packet(&vision_control);

////////////////////////////////////////////////




    //LOGINFO("Pitch ECD: %f | IMU: %f", pitch_motor->measure.angle_single_round, gimba_IMU_data->Pitch);
    //LOGINFO("Yaw ECD: %d", yaw_motor->measure.ecd);

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}