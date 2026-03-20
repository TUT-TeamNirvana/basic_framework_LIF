#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H


/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

// int16_t map_value(float value, float *ori_scope, float *target_scope);

#endif // !ROBOT_CMD_H