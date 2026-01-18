/**
 ******************************************************************************
 * @file    scheduler.c
 * @author  kai
 * @version
 * @data    2026/01/17
 * @brief   调度器模块
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "scheduler.h"
#include "attitude_control.h"
#include "position_control.h"

attitude_control_handle_t att_ctrl_handle;
position_control_handle_t pos_ctrl_handle;
/**
 * @brief 姿态期望值(设定点)
 *
 */
extern angle_setpoint_t angle_sp;

/**
 * @brief 角速度期望值(设定点)
 *
 */
extern rate_setpoint_t rate_sp;

/**
 * @brief 升力
 *
 */
static float _thrust;

/**
 * @brief 控制力矩
 *
 */
static control_torque_t _torque;

/**
 * @brief 系统状态，包括初始化\锁定状态、待机\怠速状态、运行\飞行状态
 *
 */
typedef enum
{
    INIT_STATE = 0,
    STANDBY_STATE,
    RUNNING_STATE
} system_state_t;

/**
 * @brief 调度器句柄
 *
 */
typedef struct
{
    system_state_t system_state;
} scheduler_handle_t;

/**
 * @brief 控制算法调度，根据系统状态，调度不同的控制算法
 *
 */
void scheduler_ctrl(scheduler_handle_t *handle)
{
    switch (handle->system_state)
    {
    case INIT_STATE:
    /*初始化状态，对应飞机刚上电，或者降落并上锁*/
        
        break;

    case STANDBY_STATE:
        /* code */
        break;

    case RUNNING_STATE:
        /* code */
        break;

    default:
        break;
    }
}
