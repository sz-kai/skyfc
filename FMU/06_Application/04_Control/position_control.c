/**
 ******************************************************************************
 * @file    position_control.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/12/26
 * @brief   位置控制模块
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "position_control.h"

// /**
//  * @brief 私有函数声明
//  *
//  */
// static void pos_ctrl_get_ground_height(position_control_handle_t *handle);
// static void pos_ctrl_height(position_control_handle_t *handle);
// static void pos_ctrl_horizontal(position_control_handle_t *handle, angle_setpoint_t *angle_sp);

/**
 * @brief 位置控制初始化
 *
 */
void position_control_init(position_control_handle_t *handle, const pos_vel_t *pos)
{
    if (handle == NULL || pos == NULL)
    {
        return;
    }
    handle->ctrl_state = 0;
    memset(handle->ground_height_buffer, 0, sizeof(handle->ground_height_buffer));
    handle->ground_height_buffer_index = 0;
    handle->get_ground_height = false;

    /*控制参数初始化*/
    handle->alt_k1 = 0.0f;
    handle->alt_k2 = 0.0f;
    handle->alt_k3 = 0.0f;

    /*状态变量初始化*/
    handle->ground_height = 0.0f;
    handle->start_pos_x = pos->pos.x;
    handle->start_pos_y = pos->pos.y;
    handle->ver_v_sp = 0.0f;
    handle->ver_h_sp = 0.0f;
    handle->hor_vx_sp = 0.0f;
    handle->hor_vy_sp = 0.0f;
    handle->hor_px_sp = 0.0f;
    handle->hor_py_sp = 0.0f;
    // handle->throttle_output = 0.0f;
    handle->hover_thrust = 0.0f;
    handle->flight_state = POS_CTRL_STATE_LAND;
    handle->flight_mode = FLIGHT_HOVER;

    handle->integral_x = 0.0f;
    handle->integral_y = 0.0f;
}

/**
 * @brief 获取地面高度
 *
 * @param handle 位置控制句柄
 * @param height 当前高度
 */
void pos_ctrl_get_ground_height(position_control_handle_t *handle, float height)
{
    if (handle == NULL)
    {
        return;
    }
    // float height;
    // /*获取高度数据*/
    // get_baro(&height);
    handle->ground_height_buffer[handle->ground_height_buffer_index++] = height;
    if (handle->ground_height_buffer_index < GROUND_HEIGHT_SAMPLE_SIZE)
    {
        return;
    }
    handle->ground_height_buffer_index = 0;
    /*计算平均值*/
    float sum = 0.0f;
    for (int i = 0; i < GROUND_HEIGHT_SAMPLE_SIZE; i++)
    {
        sum += handle->ground_height_buffer[i];
    }
    handle->ground_height = sum / GROUND_HEIGHT_SAMPLE_SIZE;
    handle->ground_height_buffer_index = 0;
    handle->get_ground_height = true;
}

/**
 * @brief 期望高度计算
 *
 * @param handle  位置控制句柄
 * @param rc_data 遥控数据
 * @param height  当前高度
 */
void pos_ctrl_height_setpoint(position_control_handle_t *handle,
                              const rc_control_t *rc_data, float height)
{
    if (handle == NULL || rc_data == NULL)
    {
        return;
    }
    // /*1. 获取rc数据*/
    // rc_map_data_t rc_data;
    // get_rc(&rc_data);
    // float height;
    // get_baro(&height);
    /*计算垂直方向期望速度*/
    if (rc_data->throttle > RC_DEAD_ZONE)
    {
        handle->ver_v_sp = POS_CTRL_MAX_ASCEND_VEL *
                           (rc_data->throttle - RC_DEAD_ZONE) / (1.0f - RC_DEAD_ZONE);
    }
    else if (rc_data->throttle < -RC_DEAD_ZONE)
    {
        handle->ver_v_sp = POS_CTRL_MAX_ASCEND_VEL *
                           (rc_data->throttle + RC_DEAD_ZONE) / (1.0f - RC_DEAD_ZONE);
    }
    else
    {
        handle->ver_v_sp = 0.0f;
    }
    /*2. 计算期望高度（积分）*/
    if (handle->flight_state == POS_CTRL_STATE_TAKEOFF)
    {
        /*这里区分起飞阶段与正常飞行阶段期望位置的积分间隔是为了使起飞阶段的位置误差更大，起飞响应更灵敏*/
        handle->ver_h_sp += handle->ver_v_sp * POS_CTRL_TAKEOFF_INTERVAL;
    }
    else if (handle->flight_state == POS_CTRL_STATE_NORMAL)
    {
        handle->ver_h_sp += handle->ver_v_sp * POS_CTRL_INTERVAL;
    }
    /*3. 限制期望高度（基于当前高度与输出限幅动态限制）*/
    /* 这里限制期望高度一方面是因为期望高度是通过积分得到了，防止一直积分导致期望高度过大 */
    float max_height = (POS_CTRL_HEIGHT_MAX_OUTPUT - handle->hover_thrust) / handle->alt_k1 +
                       (height - handle->ground_height);
    float min_height = (POS_CTRL_HEIGHT_MIN_OUTPUT - handle->hover_thrust) / handle->alt_k1 +
                       (height - handle->ground_height);
    handle->ver_h_sp = LIMIT(handle->ver_h_sp, min_height, max_height);
    if (handle->flight_state == POS_CTRL_STATE_TAKEOFF)
    {
        handle->ver_h_sp = 0.0f;
    }
}

/**
 * @brief 高度控制
 *
 * @param handle 位置控制句柄
 * @param pos_vel 当前位置速度
 * @param acc 当前加速度
 * @param throttle_output 油门输出
 */
void pos_ctrl_height(position_control_handle_t *handle,
                     const pos_vel_t *pos_vel, const Axis3_f_u *acc,
                     float thrust)
{
    if (handle == NULL || pos_vel == NULL || acc == NULL)
    {
        return;
    }
    // pos_vel_t pos_vel;
    // Axis3_f_u acc;
    /* 1. 获取期望值与测量值 */

    // get_position(&pos_vel);
    // get_acc(&acc);
    /* 2. 计算误差 */
    float error_pos = handle->ver_h_sp - pos_vel->pos.z;
    float error_vel = handle->ver_v_sp - pos_vel->vel.z;
    /* 3. 计算控制量与油门 */
    float u_height = handle->alt_k1 * error_pos + handle->alt_k2 * error_vel + handle->alt_k3 * acc->z;
    thrust = handle->hover_thrust + u_height;
    /* 4. 限幅 */
    thrust = LIMIT(thrust, POS_CTRL_HEIGHT_MIN_OUTPUT, POS_CTRL_HEIGHT_MAX_OUTPUT);
}

/**
 * @brief 获取期望水平位置
 *
 * @param handle 位置控制句柄
 * @param rc 遥控数据
 * @param euler 当前欧拉角
 */
void pos_ctrl_horizontal_setpoint(position_control_handle_t *handle,
                                  const rc_control_t *rc, const Euler_u *euler)
{
    if (handle == NULL || rc == NULL || euler == NULL)
    {
        return;
    }

    /*1. 获取rc数据*/
    // rc_map_data_t rc_data_temp;
    // get_rc(&rc_data_temp);

    /*2. 计算水平方向期望速度，中间坐标系下*/
    float setpoint_vel_x = 0.0f;
    float setpoint_vel_y = 0.0f;
    if (rc->roll > RC_DEAD_ZONE)
    {
        setpoint_vel_x = POS_CTRL_MAX_HOR_VEL_X *
                         (rc->roll - RC_DEAD_ZONE) / (1.0f - RC_DEAD_ZONE);
    }
    else if (rc->roll < -RC_DEAD_ZONE)
    {
        setpoint_vel_x = POS_CTRL_MAX_HOR_VEL_Y *
                         (rc->roll + RC_DEAD_ZONE) / (1.0f - RC_DEAD_ZONE);
    }
    else
    {
        setpoint_vel_x = 0.0f;
    }

    if (rc->pitch > RC_DEAD_ZONE)
    {
        setpoint_vel_y = POS_CTRL_MAX_HOR_VEL_X *
                         (rc->pitch - RC_DEAD_ZONE) / (1.0f - RC_DEAD_ZONE);
    }
    else if (rc->pitch < -RC_DEAD_ZONE)
    {
        setpoint_vel_y = POS_CTRL_MAX_HOR_VEL_Y *
                         (rc->pitch + RC_DEAD_ZONE) / (1.0f - RC_DEAD_ZONE);
    }
    else
    {
        setpoint_vel_y = 0.0f;
    }

    /*3. 坐标变换，中间坐标系到地理坐标系*/
    /*遥控器控制基于中间坐标系，相对于地理系只有偏航，可以控制飞机向前、向左飞行*/
    /*但飞控关注的是地理坐标系，即往北飞多少、往东飞多少*/
    // Euler_u euler_temp;
    // get_euler(&euler_temp);
    handle->hor_vx_sp = setpoint_vel_x * cosf(euler->yaw) - setpoint_vel_y * sinf(euler->yaw);
    handle->hor_vy_sp = setpoint_vel_x * sinf(euler->yaw) + setpoint_vel_y * cosf(euler->yaw);
}

/**
 * @brief 非线性P控制
 *
 * @param vel_error 位置误差
 * @param vel_kp 增益
 * @return float P控制量
 */
static float pos_ctrl_nonlinear_p_caculate(float vel_error, float vel_kp)
{
    float breakpoint_vel = 0.5f; // 阶梯p系数的拐点速度误差
    float param = 1.5f;          // 速度误差较小部分，p的放大倍数

    if (fabs(vel_error) <= breakpoint_vel)
        return (param * vel_error * vel_kp);
    else if (vel_error > breakpoint_vel)
        return (vel_error * vel_kp + (param - 1.0f) * breakpoint_vel * vel_kp);
    else if (vel_error < -breakpoint_vel)
        return (vel_error * vel_kp - (param - 1.0f) * breakpoint_vel * vel_kp);
    else
        return (vel_error * vel_kp);
}

/**
 * @brief 水平位置控制
 *
 * @param handle 位置控制句柄
 * @param pos_vel 当前位置速度
 * @param acc 当前加速度
 * @param euler 当前欧拉角
 * @param angle_sp 角度期望值输出
 */
void pos_ctrl_horizontal(position_control_handle_t *handle,
                         const pos_vel_t *pos_vel, const Axis3_f_u *acc, const Euler_u *euler,
                         angle_setpoint_t *angle_sp)
{
    if (handle == NULL|| pos_vel == NULL || acc == NULL || euler == NULL || angle_sp == NULL)
    {
        return;
    }
    // pos_vel_t pos_vel_temp;
    // get_position(&pos_vel_temp);

    /*1. 飞行模式切换*/
    if (handle->flight_mode == FLIGHT_HOVER)
    {
        /*悬停模式时，当期望速度大于阈值时，切换为移动模式*/
        if (handle->hor_vx_sp * handle->hor_vx_sp + handle->hor_vy_sp * handle->hor_vy_sp >
            FLIGHT_MODE_SWITCH1 * FLIGHT_MODE_SWITCH1)
        {
            handle->flight_mode = FLIGHT_MANUAL;
        }
    }
    else if (handle->flight_mode == FLIGHT_MANUAL)
    {

        /*移动模式，当期望速度与实际速度小于阈值时，切换为悬停模式*/
        if (handle->hor_vx_sp * handle->hor_vx_sp + handle->hor_vy_sp * handle->hor_vy_sp <
                FLIGHT_MODE_SWITCH2 * FLIGHT_MODE_SWITCH2 &&
            pos_vel->vel.x * pos_vel->vel.x + pos_vel->vel.y * pos_vel->vel.y <
                FLIGHT_MODE_SWITCH3 * FLIGHT_MODE_SWITCH3)
        {
            handle->flight_mode = FLIGHT_HOVER;
            /*刹车预判，参考例程代码计算，并非标准刹车补偿公式，应该是一种经验公式*/
            /*Pos_{target} = Pos_{current} + \frac{V_{current}}{Kp_{pos}}*/
            /*这里将Pos_{current}hor_px_sp，但其实hor_px_sp储存的是上一时刻的飞机位置*/
            /*但仍有一个问题，由于使用的是上一时刻的位置，计算出的期望位置有一个时间步的滞后误差*/
            // handle->hor_px_sp += pos_ctrl_nonlinear_p_caculate(pos_vel_temp.vel.x, handle->vel_kp) / handle->pos_kp;
            // handle->hor_py_sp += pos_ctrl_nonlinear_p_caculate(pos_vel_temp.vel.y, handle->vel_kp) / handle->pos_kp;
            handle->hor_px_sp = pos_vel->pos.x - handle->start_pos_x +
                                pos_ctrl_nonlinear_p_caculate(pos_vel->vel.x, handle->vel_kp) / handle->pos_kp;
            handle->hor_py_sp = pos_vel->pos.y - handle->start_pos_y +
                                pos_ctrl_nonlinear_p_caculate(pos_vel->vel.y, handle->vel_kp) / handle->pos_kp;
        }
    }
    /*误差计算*/
    if (handle->flight_mode == FLIGHT_HOVER)
    {
        /*悬停模式时，期望位置为期望速度积分*/
        /*这里有些奇怪，因为之前已经计算了刹车补偿期望位置*/
        /*不过这时期望速度非常小，积分后几乎为0，可能是在对刹车距离进行微调*/
        handle->hor_px_sp += handle->hor_vx_sp * POS_CTRL_INTERVAL;
        handle->hor_py_sp += handle->hor_vy_sp * POS_CTRL_INTERVAL;
    }
    else if (handle->flight_mode == FLIGHT_MANUAL)
    {
        /*手动模式时，只进行速度控制*/
        handle->hor_px_sp = pos_vel->pos.x - handle->start_pos_x;
        handle->hor_py_sp = pos_vel->pos.y - handle->start_pos_y;
    }
    float error_pos_x, error_pos_y, error_vel_x, error_vel_y;
    error_pos_x = handle->hor_px_sp - (pos_vel->pos.x - handle->start_pos_x);
    error_pos_y = handle->hor_py_sp - (pos_vel->pos.y - handle->start_pos_y);
    error_vel_x = handle->hor_vx_sp - pos_vel->vel.x;
    error_vel_y = handle->hor_vy_sp - pos_vel->vel.y;

    /*位置P控制*/
    float pos_x_p_item = error_pos_x * handle->pos_kp;
    float pos_y_p_item = error_pos_y * handle->pos_kp;

    /*速度P控制*/
    float vel_x_p_item = pos_ctrl_nonlinear_p_caculate(error_vel_x, handle->vel_kp);
    float vel_y_p_item = pos_ctrl_nonlinear_p_caculate(error_vel_y, handle->vel_kp);
    /*速度I控制*/
    /*未饱和且手动状态下使能*/
    float vel_x_i_item, vel_y_i_item = 0.0f;
    if (!handle->saturation_xy && handle->flight_mode == FLIGHT_MANUAL)
    {
        handle->integral_x += error_vel_x * POS_CTRL_INTERVAL;
        handle->integral_y += error_vel_y * POS_CTRL_INTERVAL;
        vel_x_i_item = handle->integral_x * handle->vel_ki;
        vel_y_i_item = handle->integral_y * handle->vel_ki;
    }
    /*速度D控制*/
    // Axis3_f_u acc_temp;
    // get_acc(&acc_temp);
    float vel_x_d_item = acc->x * handle->vel_kd;
    float vel_y_d_item = acc->y * handle->vel_kd;

    /*速度环输出*/
    float u_vel_x = vel_x_p_item + vel_x_i_item + vel_x_d_item;
    float u_vel_y = vel_y_p_item + vel_y_i_item + vel_y_d_item;
    /*限幅*/
    /*等比例缩放限幅以避免改变输出矢量方向*/
    float att_saturation = sqrtf(u_vel_x * u_vel_x + u_vel_y * u_vel_y);
    if (att_saturation > POS_CTRL_VEL_MAX_OUTPUT)
    {
        handle->saturation_xy = true;
        u_vel_x = u_vel_x / att_saturation * POS_CTRL_VEL_MAX_OUTPUT;
        u_vel_y = u_vel_y / att_saturation * POS_CTRL_VEL_MAX_OUTPUT;
    }
    else
    {
        handle->saturation_xy = false;
    }
    /*总输出*/
    float u_x = vel_x_p_item + u_vel_x;
    float u_y = vel_y_p_item + u_vel_y;
    /*坐标变换(到中间坐标系)*/
    // Euler_u euler_temp;
    // get_euler(&euler_temp);
    angle_sp->roll = (u_x * cosf(euler->yaw) + u_y * sinf(euler->yaw));
    angle_sp->pitch = (-u_x * sinf(euler->yaw) + u_y * cosf(euler->yaw));
    /*限幅*/
    angle_sp->roll = LIMIT(angle_sp->roll, -SP_ROLL_MAX, SP_ROLL_MAX);
    angle_sp->pitch = LIMIT(angle_sp->pitch, -SP_PITCH_MAX, SP_PITCH_MAX);
}
