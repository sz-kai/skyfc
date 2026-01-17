/**
 ******************************************************************************
 * @file    position_control.h
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

#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__
#include "pro_include.h"

/*地面高度采样缓冲区大小*/
#define GROUND_HEIGHT_SAMPLE_SIZE 10

/*位置控制更新间隔*/
#define POS_CTRL_INTERVAL 1.0f / POS_CTRL_UPDATE_FREQ
/*起飞状态POS_CTRL_STATE_TAKEOFF期望位置积分步长，较大值(相对于POS_CTRL_INTERVAL)可以使起飞响应更快*/
#define POS_CTRL_TAKEOFF_INTERVAL 0.04f

/*飞行模式切换阈值(期望速度大于该阈值时飞行模式)*/
/*切换为移动模式，当期望速度范数大于该阈值时*/
#define FLIGHT_MODE_SWITCH1 0.05f
/*切换为悬停模式，当期望速度范数小于该阈值时*/
#define FLIGHT_MODE_SWITCH2 0.02f
/*切换为悬停模式，当实际速度范数小于该阈值时*/
#define FLIGHT_MODE_SWITCH3 0.5f

/*速度环限幅（弧度，间接对应姿态角设定点）*/
#define POS_CTRL_VEL_MAX_OUTPUT SP_PITCH_MAX * 0.8f

/**
 * @brief 飞行状态
 *
 */
typedef enum
{
  POS_CTRL_STATE_LAND = 0,
  POS_CTRL_STATE_TAKEOFF,
  POS_CTRL_STATE_NORMAL
} pos_ctrl_state_t;

/**
 * @brief 飞行模式(悬停、手动)
 *
 */
typedef enum
{
  FLIGHT_HOVER = 0,
  FLIGHT_MANUAL
} flight_mode_t;

/**
 * @brief 位置控制句柄
 *
 */
typedef struct
{
  /*控制参数*/
  float alt_k1;
  float alt_k2;
  float alt_k3;
  float pos_kp;
  float vel_kp;
  float vel_ki;
  float vel_kd;
  /*状态变量*/
  float ground_height; /*地面高度*/
  float start_pos_x;   /*起点位置x*/
  float start_pos_y;   /*起点位置y*/
  /*期望值，地理坐标系*/
  float ver_v_sp;  /*垂直方向期望速度*/
  float ver_h_sp;  /*期望高度*/
  float hor_vx_sp; /*水平方向x期望速度*/
  float hor_vy_sp; /*水平方向y期望速度*/
  float hor_px_sp; /*水平方向x期望位置*/
  float hor_py_sp; /*水平方向y期望位置*/
  // float throttle_output;           /*油门*/
  float hover_thrust; /*悬停升力*/

  float integral_x; /*积分累计*/
  float integral_y; /*积分累计*/

  bool saturation_xy; /*饱和标志*/

  uint8_t ctrl_state;                                    /*位置控制状态机*/
  float ground_height_buffer[GROUND_HEIGHT_SAMPLE_SIZE]; /*地面高度采样缓冲区*/
  uint8_t ground_height_buffer_index;                    /*地面高度采样缓冲区索引*/
  bool get_ground_height;                    /*获取到地面高度数据*/

  pos_ctrl_state_t flight_state; /*飞行状态*/
  flight_mode_t flight_mode;     /*飞行模式*/

} position_control_handle_t;

void position_control_init(position_control_handle_t *handle, const pos_vel_t *pos);
void pos_ctrl_get_ground_height(position_control_handle_t *handle, float height);
void position_control_update(position_control_handle_t *handle, angle_setpoint_t *angle_sp);
void pos_ctrl_height_setpoint(position_control_handle_t *handle,
                              const rc_control_t *rc_data, float height);
void pos_ctrl_height(position_control_handle_t *handle,
                     const pos_vel_t *pos_vel, const Axis3_f_u *acc,
                     float thrust);
void pos_ctrl_horizontal_setpoint(position_control_handle_t *handle,
                                  const rc_control_t *rc, const Euler_u *euler);
void pos_ctrl_horizontal(position_control_handle_t *handle,
                         const pos_vel_t *pos_vel, const Axis3_f_u *acc, const Euler_u *euler,
                         angle_setpoint_t *angle_sp);

#endif
