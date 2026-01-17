/**
 ******************************************************************************
 * @file    task_navigation.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/09/30
 * @brief   组合导航任务
 ******************************************************************************
 * @attention
 *
 * 组合导航任务
 *
 ******************************************************************************
 */
#include "task_navigation.h"

static Attitude_Estimate_t att_est; /*姿态解算结构体*/
static Position_Estimate_t pos_est; /*位置估计结构体*/

static attitude_t att;
// static pos_vel_t pos_vel;

/**
 * @brief  组合导航任务初始化
 * @note
 * @param  无
 * @retval 无
 */
void Task_IntegratedNavigation_Init(void)
{
    att_est.attitude_estimate_init_flag = false;

    pos_est.init_flag = false;
    pos_est.baro_offset = 0.0f;
    pos_est.baro_init_count = 0;
}

/**
 * @brief  组合导航任务
 * @note
 * @param  无
 * @retval 无
 */

void Task_IntegratedNavigation(void)
{
    /*姿态解算，更新三轴欧拉角*/
    attitude_estimate_update(&att_est, &flgt_ctl.sensor);
    att_est_get_data(&att_est, &flgt_ctl.attitude);
    /*位置估计，更新位置、速度，50hz*/
    if (LOOP_FREQ_SET(LOOP_50_Hz, Tick, LOOP_500_Hz))
    {
        position_estimate_update(&pos_est, &flgt_ctl);
        position_estimate_get_pos_vel(&pos_est, &flgt_ctl.pos_vel);
    }
}

/**
 * @brief 获取姿态估计结果
 *
 */
const attitude_t *get_attitude_estimate(void)
{
    /*关闭中断，确保数据一致性*/
    __disable_irq();
    for (int i = 0; i < 3; i++)
    {
        att.euler.axis_arr[i] = att_est.euler.axis_arr[i];
        att.gyro.axis_arr[i] = flgt_ctl.sensor.gyro.axis_arr[i];
        // for (int j = 0; j < 3; j++)
        // {
        //     att_est.rotation_matrix[i][j] = att_est->Mahony_Param.DCM[i][j];
        // }
    }
    /*开启中断*/
    __enable_irq();
    return &att;
}



/**
 * @brief 获取姿态
 * 
 */
Euler_u get_euler(void)
{
    Euler_u euler;
    __disable_irq();
    euler = att_est.euler;
    /*开启中断*/
    __enable_irq();
    return euler;
}

/**
 * @brief 获取角速度
 * 
 */
Axis3_f_u get_rate(void)
{
    Axis3_f_u rate;
    __disable_irq();
    rate = flgt_ctl.sensor.gyro;
    /*开启中断*/
    __enable_irq();
    return rate;
}

/**
 * @brief 获取旋转矩阵
 * 
 */
void get_rotation_matrix(float rot_mat[3][3])
{
    __disable_irq();
    memcpy(rot_mat, att_est.Mahony_Param.DCM, sizeof(att_est.Mahony_Param.DCM));
    /*开启中断*/
    __enable_irq();
}

/**
 * @brief 获取位置
 * 
 */
pos_vel_t get_pos_vel(void)
{
    pos_vel_t pos_vel_temp;
    __disable_irq();
    pos_vel_temp.pos.x = pos_est.est_x[0];
    pos_vel_temp.pos.y = pos_est.est_y[0];
    pos_vel_temp.pos.z = pos_est.est_z[0];
    pos_vel_temp.vel.x = pos_est.est_x[1];
    pos_vel_temp.vel.y = pos_est.est_y[1];
    pos_vel_temp.vel.z = pos_est.est_z[1];
    /*开启中断*/
    __enable_irq();
    return pos_vel_temp;
}





