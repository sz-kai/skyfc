/**
 ******************************************************************************
 * @file    attitude_control.h
 * @author  kai
 * @version V1.0.0
 * @data    2025/12/03
 * @brief   姿态控制算法头文件
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

#include "pro_include.h"
#include "pid.h"



/**
 * @brief 姿态控制句柄
 *
 */
typedef struct
{
    /*外环角度控制*/
    PID_Handle_t roll_pid_handle;
    PID_Handle_t pitch_pid_handle;
    PID_Handle_t yaw_pid_handle;
    /*内环角速度控制*/
    PID_Handle_t roll_rate_pid_handle;
    PID_Handle_t pitch_rate_pid_handle;
    PID_Handle_t yaw_rate_pid_handle;
} attitude_control_handle_t;



void attitude_control_init(attitude_control_handle_t *handle);

void attitude_control_outer_loop(attitude_control_handle_t *handle,
                                 const angle_setpoint_t *setpoint, const Euler_u *measure, const float dt,
                                 rate_setpoint_t *rate_sp);

void attitude_control_inner_loop(attitude_control_handle_t *handle,
                                 const rate_setpoint_t *setpoint, const Axis3_f_u *measure, const float dt, const float _thrust,
                                 control_torque_t *out_torque);

#endif /* __ATTITUDE_CONTROL_H */
