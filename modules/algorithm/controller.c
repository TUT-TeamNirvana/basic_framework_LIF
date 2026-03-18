/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @author modified for 2006test fusion
 * @brief  融合了软死区与条件抗积分饱和的PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "memory.h"
#include <math.h>

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

// 传统的死限幅积分 (如果启用了PID_Integral_Limit)
static void f_Integral_Limit(PIDInstance *pid)
{
    // 这部分保留给只使用死限幅而不使用条件抗积分饱和的情况
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    
    // 如果没有使用条件抗积分饱和(Clamp)，则使用原来的硬切断
    if (!(pid->Improve & PID_Clamp_Anti_Windup))
    {
        if (fabsf(temp_Output) > pid->MaxOut)
        {
            if (pid->Err * pid->Iout > 0) // 积分却还在累积
            {
                pid->ITerm = 0; // 当前积分项置零
            }
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    memset(pid, 0, sizeof(PIDInstance));
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    
    // 如果没有设置前馈参数，默认置零
    // 由于memcpy覆盖了整个config的大小，如果config没有Kff，这里的Kff会变成乱码或零，
    // 但在电机控制器的设定里，往往并没有传递Kff，所以我们安全起见确保它们是0
    // (如果使用者想要Kff，后续应通过额外函数设置，或者修改PID_Init_Config_s)
    
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回计算后的输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

    // 保存本次测量值和期望值
    pid->Measure = measure;
    pid->Ref = ref;
    
    // 计算真实误差
    float real_err = pid->Ref - pid->Measure;
    
    // ========================================================
    // 软死区处理 (来自 2006test 的精髓)
    // ========================================================
    uint8_t in_deadband = (fabsf(real_err) < pid->DeadBand);
    if (in_deadband)
    {
        // 如果在死区内，将计算用的误差视为0，但不清空积分Iout和输出，保持稳态力矩
        pid->Err = 0.0f;
    }
    else
    {
        pid->Err = real_err;
    }

    // 比例项
    pid->Pout = pid->Kp * pid->Err;

    // 前馈项与加速度前馈 (可选，如果启用了前馈并且设置了Kff)
    float ff_term = pid->Kff * pid->Ref;
    float accel_term = 0.0f;
    if (pid->dt > 0.000001f) {
        float acceleration = (pid->Ref - pid->Last_Ref) / pid->dt;
        accel_term = pid->Kaff * acceleration;
    }

    // 微分项计算
    if (pid->Improve & PID_Derivative_On_Measurement)
    {
        // 微分先行: 无论是否在死区，都基于测量值计算微分
        f_Derivative_On_Measurement(pid);
        if (in_deadband) {
            pid->Dout = 0.0f; // 死区内仍然抑制微分噪声
        }
    }
    else
    {
        // 传统误差微分
        if (!in_deadband) {
            pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        } else {
            pid->Dout = 0.0f;
        }
    }

    // 微分滤波
    if (pid->Improve & PID_DerivativeFilter)
        f_Derivative_Filter(pid);

    // ========================================================
    // 积分项计算与智能抗饱和 (Clamp Anti-windup)
    // ========================================================
    // 默认积分计算
    pid->ITerm = pid->Ki * pid->Err * pid->dt;
    
    // 梯形积分和变速积分（仅对当次计算出的 ITerm 进行调整）
    if (pid->Improve & PID_Trapezoid_Intergral)
        f_Trapezoid_Intergral(pid);
    if (pid->Improve & PID_ChangingIntegrationRate)
        f_Changing_Integration_Rate(pid);

    // 条件积分抗饱和 (Clamp): 如果启用
    if (pid->Improve & PID_Clamp_Anti_Windup)
    {
        float current_output_pre_I = pid->Pout + pid->Iout + pid->Dout + ff_term + accel_term;
        uint8_t should_update_integral = 0;
        
        // 如果尚未达到输出极限，或者（已达极限且积分方向有助于退出饱和）
        if (fabsf(current_output_pre_I) <= pid->MaxOut) 
        {
            float potential_output = current_output_pre_I + pid->ITerm;
            if (fabsf(potential_output) <= pid->MaxOut) {
                should_update_integral = 1;
            } else {
                if ((potential_output > pid->MaxOut && pid->Err < 0.0f) ||
                    (potential_output < -pid->MaxOut && pid->Err > 0.0f)) {
                    should_update_integral = 1;
                }
            }
        } 
        else 
        {
            if ((current_output_pre_I > pid->MaxOut && pid->Err < 0.0f) ||
                (current_output_pre_I < -pid->MaxOut && pid->Err > 0.0f)) {
                should_update_integral = 1;
            }
        }

        if (!should_update_integral) {
            pid->ITerm = 0.0f; // 钳位，不累加当前积分
        }
    }

    // 积分限幅 (硬限幅，如果启用了PID_Integral_Limit)
    if (pid->Improve & PID_Integral_Limit)
        f_Integral_Limit(pid);

    // 累加积分
    pid->Iout += pid->ITerm;

    // ========================================================
    // 最终输出合成
    // ========================================================
    pid->Output = pid->Pout + pid->Iout + pid->Dout + ff_term + accel_term;

    // 输出滤波
    if (pid->Improve & PID_OutputFilter)
        f_Output_Filter(pid);

    // 输出限幅
    f_Output_Limit(pid);

    // 保存历史数据用于下次计算
    // 注意: 在软死区处理下，只有在死区外才更新 Last_Err，以防止离开死区时的 D-kick
    if (!in_deadband) {
        pid->Last_Err = pid->Err;
    }
    
    pid->Last_Measure = pid->Measure;
    pid->Last_Ref = pid->Ref;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}