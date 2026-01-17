/**
  ******************************************************************************
  * @file    task_guidance.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/12/08
  * @brief   制导任务头文件
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

#ifndef __TASK_GUIDANCE_H
#define __TASK_GUIDANCE_H
#include "pro_include.h"


/**
 * @brief 导航任务调用频率(Hz)，在计算期望偏航角时使用
 * 
 */
#define GUIDANCE_FREQ_HZ 100.0f

/*===============================================================================================*/
/*=========                                  函数声明                                   ==========*/
/*===============================================================================================*/

/**
 * @brief 制导任务，获取期望值
 *
 */
void task_guidance(void);

/**
 * @brief 获取期望点
 *
 * @retval const setpoint_t* - 指向setpoint的指针
 */
const angle_setpoint_t *get_setpoint(void);

/**
 * @brief 获取rc数据
 * 
 * @param map_data - 提供获取映射后rc数据的缓冲区，指向rc_map_data_t的指针
 */
// void get_rc(rc_map_data_t *map_data);
rc_control_t get_rc_ctrl(void);
rc_switch_t get_rc_switch(void);
angle_setpoint_t get_angle_sp(void);

#endif /* __TASK_GUIDANCE_H */
