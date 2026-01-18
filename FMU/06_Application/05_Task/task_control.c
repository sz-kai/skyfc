/**
 ******************************************************************************
 * @file    task_control.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/12/10
 * @brief   控制任务源文件
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "task_control.h"
#include "task_guidance.h"
#include "task_navigation.h"
#include "attitude_control.h"
#include "position_control.h"
#include "task_sensor.h"



/*姿态控制外环更新时间间隔*/
#define ATT_CTRL_OUTER_LOOP_UPDATE_INTERVAL (1.0f / ATT_CTRL_OUTER_LOOP_UPDATE_FREQ)
/*姿态控制内环更新时间间隔*/
#define ATT_CTRL_INNER_LOOP_UPDATE_INTERVAL (1.0f / ATT_CTRL_INNER_LOOP_UPDATE_FREQ)

/**
 * @brief 控制句柄
 *
 */
typedef struct
{
    /*起飞位置*/
} control_handle_t;

void task_control_init(void)
{
    // position_control_init(&pos_ctrl_handle);
}

/**
 * @brief 姿态控制
 *
 */
void task_attitude_control(control_torque_t *out_torque)
{

    /*250Hz外环姿态任务*/
    if (LOOP_FREQ_SET(ATT_CTRL_OUTER_LOOP_UPDATE_FREQ, Tick, LOOP_500_Hz))
    {
        const angle_setpoint_t angle_sp = get_angle_sp();
        const Euler_u att = get_euler();
        attitude_control_outer_loop(&att_ctrl_handle,
                                    &angle_sp, &att, ATT_CTRL_OUTER_LOOP_UPDATE_INTERVAL,
                                    &rate_sp);
    }
    /*500Hz内环姿态任务*/
    if (LOOP_FREQ_SET(ATT_CTRL_INNER_LOOP_UPDATE_FREQ, Tick, LOOP_500_Hz))
    {
        const Axis3_f_u rate = get_rate();
        attitude_control_inner_loop(&att_ctrl_handle,
                                    &rate_sp, &rate, ATT_CTRL_INNER_LOOP_UPDATE_INTERVAL, _thrust,
                                    out_torque);
    }
}

/**
 * @brief 位置控制
 *
 * @param handle - 位置控制句柄
 */
void task_position_control(position_control_handle_t *handle,
                           float thrust, angle_setpoint_t *angle_sp)
{
    if (handle == NULL || angle_sp == NULL)
    {
        return;
    }
    switch (handle->ctrl_state)
    {
    case 0: /*起飞等待阶段*/
    {       /*1. 初始化*/
        const pos_vel_t pos_vel_temp = get_pos_vel();
        position_control_init(handle, &pos_vel_temp);
        /*2. 持续更新地面高度零点*/
        pos_ctrl_get_ground_height(handle, get_baro());
        /*3. 油门推起，获取到地面高度数据后，切换到准备起飞阶段*/
        if (get_rc_ctrl().throttle < RC_DEAD_ZONE && (handle->get_ground_height == true))
        {
            handle->ctrl_state = 1;
        }
        break;
    }
    case 1: /*准备起飞阶段*/
    {       /*开始进行高度控制*/
        /*当速度>0.15,认为飞机离地*/
        if (get_pos_vel().vel.z > 0.15f)
        {
            /*将期望速度清零，按照参考例程进行的编程，可能是为了防止飞机在起飞时过冲*/
            /*但这里也只是在这一时间步将期望速度清零，可能对飞机无影响或者短暂顿一下*/
            handle->flight_state = POS_CTRL_STATE_TAKEOFF;
            handle->ctrl_state = 2;
        }
        break;
    }
    case 2: /*正常起飞-过渡阶段*/
    {       /*飞机离地后，飞手应该将杆量回零*/
        rc_control_t rc_data_temp = get_rc_ctrl();
        if (rc_data_temp.throttle < RC_DEAD_ZONE && rc_data_temp.throttle > -RC_DEAD_ZONE)
        {
            handle->flight_state = POS_CTRL_STATE_NORMAL;
            handle->ctrl_state = 3;
        }
        break;
    }
    case 3:
        handle->flight_state = POS_CTRL_STATE_NORMAL;
        break;
    default:
        break;
    }
    /*当位置控制状态机大于等于1时（准备起飞阶段或以上），开始进行高度控制*/
    /**/
    const rc_control_t rc_ctrl_temp = get_rc_ctrl();
    const pos_vel_t pos_vel_temp = get_pos_vel();
    const Axis3_f_u acc_temp = get_acc();
    if (handle->ctrl_state == 1)
    {
        /*准备起飞阶段，只进行高度控制*/
        pos_ctrl_height_setpoint(handle, &rc_ctrl_temp, pos_vel_temp.pos.z);

        pos_ctrl_height(handle, &pos_vel_temp, &acc_temp, thrust);
        /*更新期望姿态？*/
    }
    else if (handle->ctrl_state == 2 || handle->ctrl_state == 3)
    {
        /*起飞过渡阶段与正常飞行阶段，同时进行高度与水平位置控制*/
        // rc_control_t rc_ctrl_temp;
        pos_ctrl_height_setpoint(handle, &rc_ctrl_temp, pos_vel_temp.pos.z);
        pos_ctrl_height(handle, &pos_vel_temp, &acc_temp, thrust);

        // rc_control_t rc = get_rc_ctrl();
        Euler_u euler = get_euler();
        pos_ctrl_horizontal_setpoint(handle, &rc_ctrl_temp, &euler);
        pos_ctrl_horizontal(handle, &pos_vel_temp, &acc_temp, &euler, angle_sp);
    }
}

void task_control(void)
{
    /*50Hz位置控制任务*/
    if (LOOP_FREQ_SET(POS_CTRL_UPDATE_FREQ, Tick, LOOP_500_Hz))
    {
        task_position_control(&pos_ctrl_handle, _thrust, &angle_sp);
    }

    task_attitude_control(&_control_torque);
}
