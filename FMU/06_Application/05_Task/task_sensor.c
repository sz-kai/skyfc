/**
 ******************************************************************************
 * @file    task_sensor.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/29
 * @brief   传感器任务
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "task_sensor.h"
#include "imu_processing.h"
#include "bsp_systick.h"
#include "filter.h"
#include "task_datalog.h"
#include "hmc5883l.h"
#include "mag_processing.h"
#include "baro_processing.h"
#include "gps.h"

/*传感器处理句柄*/
typedef struct
{
    LPF2_State_t gyro_filter[3];
    LPF2_State_t acc_filter[3];
    ms5611_handle_t baro_handle;
    baro_proc_handle_t baro_proc_handle;
} Sensor_Handle_t;


Sensor_Handle_t sensor_handle;

/**
 * @brief  传感器任务初始化
 * @note
 * @param  无
 * @retval 无
 */
void Task_Sensor_Init(void)
{
    /*获取气压计句柄*/
    sensor_handle.baro_handle = ms5611_get_handle();
    /*初始化imu数据处理*/
    /*获取imu数据校验值,对于磁力计数据的校准中，用到了imu数据，所以需要先获取到imu数据校验值*/
    IMU_Processing_Init(&imu_raw_data);
    for (int i = 0; i < 3; i++)
    {
        LPF2_Init(&sensor_handle.gyro_filter[i], GYRO_FILTER_CUTOFF_FREQUENCY, GYRO_FILTER_SAMPLE_FREQUENCY);
        LPF2_Init(&sensor_handle.acc_filter[i], ACC_FILTER_CUTOFF_FREQUENCY, ACC_FILTER_SAMPLE_FREQUENCY);
    }
    MAG_Processing_Init();

    ms5611_init(sensor_handle.baro_handle);
    baro_processing_init(&sensor_handle.baro_proc_handle, CONFIG_BARO_FILTER_CUTOFF_FREQUENCY, CONFIG_BARO_FILTER_SAMPLE_FREQUENCY);
}

// data_buf_t cal_data_buf;
// data_buf_t filter_data_buf;
// data_buf_i_t raw_data_buf;

static void IMU_Write_RingBuff(void)
{
#if LOG_RECORD_IMU_RAW
    log_imu_raw_t raw_data;
    raw_data.acc_x = imu_raw_data.acc.x;
    raw_data.acc_y = imu_raw_data.acc.y;
    raw_data.acc_z = imu_raw_data.acc.z;
    raw_data.gyro_x = imu_raw_data.gyro.x;
    raw_data.gyro_y = imu_raw_data.gyro.y;
    raw_data.gyro_z = imu_raw_data.gyro.z;
#endif
#if LOG_RECORD_IMU_CAL
    /* 准备校准数据 */
    log_imu_cal_t cal_data;
    cal_data.acc_x = imu_cal_data.acc.x;
    cal_data.acc_y = imu_cal_data.acc.y;
    cal_data.acc_z = imu_cal_data.acc.z;
    cal_data.gyro_x = imu_cal_data.gyro.x;
    cal_data.gyro_y = imu_cal_data.gyro.y;
    cal_data.gyro_z = imu_cal_data.gyro.z;
#endif
#if LOG_RECORD_IMU_FILTER
    /* 准备滤波数据 */
    log_imu_filter_t filter_data;
    filter_data.acc_x = imu_filter_data.acc.x;
    filter_data.acc_y = imu_filter_data.acc.y;
    filter_data.acc_z = imu_filter_data.acc.z;
    filter_data.gyro_x = imu_filter_data.gyro.x;
    filter_data.gyro_y = imu_filter_data.gyro.y;
    filter_data.gyro_z = imu_filter_data.gyro.z;
#endif
#if 0
    uint32_t tick = GetTick();
    // raw_data_buf.tick = tick;
    // raw_data_buf.acc_x = imu_raw_data.acc.x;
    // raw_data_buf.acc_y = imu_raw_data.acc.y;
    // raw_data_buf.acc_z = imu_raw_data.acc.z;
    // raw_data_buf.gyro_x = imu_raw_data.gyro.x;
    // raw_data_buf.gyro_y = imu_raw_data.gyro.y;
    // raw_data_buf.gyro_z = imu_raw_data.gyro.z;

    cal_data_buf.tick = tick;
    cal_data_buf.acc_x = imu_cal_data.acc.x;
    cal_data_buf.acc_y = imu_cal_data.acc.y;
    cal_data_buf.acc_z = imu_cal_data.acc.z;
    cal_data_buf.gyro_x = imu_cal_data.gyro.x;
    cal_data_buf.gyro_y = imu_cal_data.gyro.y;
    cal_data_buf.gyro_z = imu_cal_data.gyro.z;

    filter_data_buf.tick = tick;
    filter_data_buf.acc_x = imu_filter_data.acc.x;
    filter_data_buf.acc_y = imu_filter_data.acc.y;
    filter_data_buf.acc_z = imu_filter_data.acc.z;
    filter_data_buf.gyro_x = imu_filter_data.gyro.x;
    filter_data_buf.gyro_y = imu_filter_data.gyro.y;
    filter_data_buf.gyro_z = imu_filter_data.gyro.z;
#endif
#if LOG_RECORD_IMU_RAW
    /* 使用封装函数推送原始数据日志 */
    Push_Log_Packet_To_RingBuff(LOG_ID_IMU_RAW, &raw_data, sizeof(raw_data), &SD_W_RingBuffMgr);
#endif
#if LOG_RECORD_IMU_CAL
    /* 使用封装函数推送校准数据日志 */
    Push_Log_Packet_To_RingBuff(LOG_ID_IMU_CAL, &cal_data, sizeof(cal_data), &SD_W_RingBuffMgr);
#endif
#if LOG_RECORD_IMU_FILTER
    /* 使用封装函数推送滤波数据日志 */
    Push_Log_Packet_To_RingBuff(LOG_ID_IMU_FILTER, &filter_data, sizeof(filter_data), &SD_W_RingBuffMgr);
#endif
}

/**
 * @brief  IMU任务部分
 * @note
 * @param  无
 * @retval 无
 */
static void Task_IMU(void)
{
    /*获取imu原始数据*/
    MPU6000_GetData(&imu_raw_data.acc, &imu_raw_data.gyro);
    /*imu数据校准、转换*/
    IMU_Processing(&imu_raw_data, &imu_cal_data);
    // printf("%f,", imu_cal_data.acc.x);
    // printf("%f,", imu_cal_data.acc.y);
    // printf("%f,", imu_cal_data.acc.z);
    // printf("%f,", imu_cal_data.gyro.x);
    // printf("%f,", imu_cal_data.gyro.y);
    // printf("%f,", imu_cal_data.gyro.z);
    /*数据滤波*/
    // imu_filter_data.gyro.x = LPF2_Update(&gyro_filter[0], imu_cal_data.gyro.x);
    // imu_filter_data.gyro.y = LPF2_Update(&gyro_filter[1], imu_cal_data.gyro.y);
    // imu_filter_data.gyro.z = LPF2_Update(&gyro_filter[2], imu_cal_data.gyro.z);
    // imu_filter_data.acc.x = LPF2_Update(&acc_filter[0], imu_cal_data.acc.x);
    // imu_filter_data.acc.y = LPF2_Update(&acc_filter[1], imu_cal_data.acc.y);
    // imu_filter_data.acc.z = LPF2_Update(&acc_filter[2], imu_cal_data.acc.z);
    imu_filter_data.gyro.x = LPF2_Update(&sensor_handle.gyro_filter[0], imu_cal_data.gyro.x);
    /**/
    imu_filter_data.gyro.y = LPF2_Update(&sensor_handle.gyro_filter[1], imu_cal_data.gyro.y);
    imu_filter_data.gyro.z = LPF2_Update(&sensor_handle.gyro_filter[2], imu_cal_data.gyro.z);
    imu_filter_data.acc.x = LPF2_Update(&sensor_handle.acc_filter[0], imu_cal_data.acc.x);
    imu_filter_data.acc.y = LPF2_Update(&sensor_handle.acc_filter[1], imu_cal_data.acc.y);
    imu_filter_data.acc.z = LPF2_Update(&sensor_handle.acc_filter[2], imu_cal_data.acc.z);
    for (int i = 0; i < 3; i++)
    {
        flgt_ctl.sensor.gyro.axis_arr[i] = imu_filter_data.gyro.axis_arr[i];
        flgt_ctl.sensor.acc.axis_arr[i] = imu_filter_data.acc.axis_arr[i];
    }
    // printf("%f,", imu_cal_data.acc.x);
    // printf("%f,", imu_cal_data.acc.y);
    // printf("%f,", imu_cal_data.acc.z);
    // printf("%f,", imu_cal_data.gyro.x);
    // printf("%f,", imu_cal_data.gyro.y);
    // printf("%f\n", imu_cal_data.gyro.z);
    /*将数据写入环形缓冲区*/
    IMU_Write_RingBuff();
}



static void MAG_Write_RingBuff(void)
{
#if LOG_RECORD_MAG_RAW
    /* 准备原始数据 */
    log_mag_raw_t log_mag_raw_data;
    log_mag_raw_data.mag_x = mag_raw_data.x;
    log_mag_raw_data.mag_y = mag_raw_data.y;
    log_mag_raw_data.mag_z = mag_raw_data.z;
#endif
#if LOG_RECORD_MAG_GAUSS
    /* 准备转换数据 */
    log_mag_gauss_t log_mag_gauss_data;
    log_mag_gauss_data.mag_x = mag_gauss_data.x;
    log_mag_gauss_data.mag_y = mag_gauss_data.y;
    log_mag_gauss_data.mag_z = mag_gauss_data.z;
#endif
#if LOG_RECORD_MAG_CAL
    /* 准备校准数据 */
    log_mag_cal_t log_mag_cal_data;
    log_mag_cal_data.mag_x = mag_cal_data.x;
    log_mag_cal_data.mag_y = mag_cal_data.y;
    log_mag_cal_data.mag_z = mag_cal_data.z;
#endif
    // #if 1
    //     uint32_t tick = GetTick();
    //     cal_data_buf.tick = tick;
    //     cal_data_buf.mag_x = mag_gauss_data.x;
    //     cal_data_buf.mag_y = mag_gauss_data.y;
    //     cal_data_buf.mag_z = mag_gauss_data.z;

    //     raw_data_buf.tick = tick;
    //     raw_data_buf.mag_x = mag_raw_data.x;
    //     raw_data_buf.mag_y = mag_raw_data.y;
    //     raw_data_buf.mag_z = mag_raw_data.z;
    // #endif

    /* 使用封装函数推送校准数据日志 */
#if LOG_RECORD_MAG_RAW
    Push_Log_Packet_To_RingBuff(LOG_ID_MAG_RAW, &log_mag_raw_data, sizeof(log_mag_raw_data), &SD_W_RingBuffMgr);
#endif
#if LOG_RECORD_MAG_GAUSS
    Push_Log_Packet_To_RingBuff(LOG_ID_MAG_GAUSS, &log_mag_gauss_data, sizeof(log_mag_gauss_data), &SD_W_RingBuffMgr);
#endif
#if LOG_RECORD_MAG_CAL
    Push_Log_Packet_To_RingBuff(LOG_ID_MAG_CAL, &log_mag_cal_data, sizeof(log_mag_cal_data), &SD_W_RingBuffMgr);
#endif
}

/**
 * @brief  磁力计任务部分
 * @note
 * @param  无
 * @retval 无
 */
void Task_MAG(void)
{
    /*50hz执行一次*/
    static uint32_t mag_tick = 0;
    if (LOOP_FREQ_SET(LOOP_50_Hz, mag_tick, LOOP_500_Hz))
    {
        hmc5883l_read_raw_data(&mag_raw_data);
        hmc5883l_convert_to_gauss(&mag_gauss_data, &mag_raw_data);
        MAG_Processing(&mag_cal_data, &mag_gauss_data);
        /*检查数据是否有效*/
        if (mag_raw_data.x > 30000 || mag_raw_data.x < -30000 || mag_raw_data.y > 30000 || mag_raw_data.y < -30000 || mag_raw_data.z > 30000 || mag_raw_data.z < -30000)
        {
            printf("mag_raw_data is invalid\n");
        }
        for (int i = 0; i < 3; i++)
        {
            flgt_ctl.sensor.mag.axis_arr[i] = mag_cal_data.axis_arr[i];
        }
        MAG_Write_RingBuff();
    }
    mag_tick++;
}

// /**
//   * @brief  气压计任务部分
//   * @note
//  * @param  无
//  * @retval 无
//  */
// void Task_BARO(void)
// {
//     /*50hz执行一次*/
//     static uint32_t baro_tick = 0;
//     /*4096采样率下，大概9.04 ms可完成一次转换，需要包括温度与压力两次转换*/
//     if (LOOP_FREQ_SET(LOOP_100_Hz, baro_tick, LOOP_500_Hz))
//     {
//         if(ms5611_read(&baro_handle))
//         {
//             baro_raw_data.pressure = baro_handle.last_data.pressure_pa;
//             baro_raw_data.temperature = baro_handle.last_data.temperature_degC;
//         }
//         baro_processing_update(&baro_proc_handle, &baro_raw_data);
//         flgt_ctl.sensor.baro_alt = baro_processing_get_altitude(&baro_proc_handle);
//     }
// }

// /*传感器处理句柄*/
// typedef struct
// {
//     LPF2_State_t gyro_filter[3];
//     LPF2_State_t acc_filter[3];
//     ms5611_handle_t baro_handle;
//     baro_proc_handle_t baro_proc_handle;
// } Sensor_Handle_t;
// Sensor_Handle_t sensor_handle;
// /**
//   * @brief  气压计任务部分
//   * @note
//  * @param  无
//  * @retval 无
//  */
// void Task_BARO(void)
// {
//     /*50hz执行一次*/
//     static uint32_t baro_tick = 0;
//     /*4096采样率下，大概9.04 ms可完成一次转换，需要包括温度与压力两次转换*/
//     if (LOOP_FREQ_SET(LOOP_100_Hz, baro_tick, LOOP_500_Hz))
//     {
//         if(ms5611_read(&sensor_handle.baro_handle))
//         {
//             baro_raw_data.pressure = sensor_handle.baro_handle.last_data.pressure_pa;
//             baro_raw_data.temperature = sensor_handle.baro_handle.last_data.temperature_degC;
//         }
//         baro_processing_update(&sensor_handle.baro_proc_handle, &baro_raw_data);
//         flgt_ctl.sensor.baro_alt = baro_processing_get_altitude(&sensor_handle.baro_proc_handle);
//     }
// }

/**
 * @brief  气压计任务部分
 * @note
 * @param  无
 * @retval 无
 */
void Task_BARO(void)
{
    /*50hz执行一次*/
    static uint32_t baro_tick = 0;
    ms5611_data_t baro_raw_data_temp;
    /*4096采样率下，大概9.04 ms可完成一次转换，需要包括温度与压力两次转换*/
    if (LOOP_FREQ_SET(LOOP_100_Hz, baro_tick, LOOP_500_Hz))
    {
        if (ms5611_read(sensor_handle.baro_handle))
        {
            ms5611_get_data(sensor_handle.baro_handle, &baro_raw_data_temp);
            baro_raw_data.pressure = baro_raw_data_temp.pressure_pa;
            baro_raw_data.temperature = baro_raw_data_temp.temperature_degC;
            baro_processing_update(&sensor_handle.baro_proc_handle, &baro_raw_data);
            flgt_ctl.sensor.baro_alt = baro_processing_get_altitude(&sensor_handle.baro_proc_handle);
        }
    }
}

extern struct vehicle_gps_position_s m_gps_position;
extern uint8_t _got_posllh; /*解析到位置数据标志*/
extern uint8_t _got_velned; /*解析到速度数据标志*/

/**
 * @brief  gps任务部分
 * @note
 * @param  无
 * @retval 无
 */
void Task_GPS(void)
{
    /*50hz执行一次*/
    static uint32_t gps_tick = 0;
    GPS_Data_t gps_data;
    if (LOOP_FREQ_SET(LOOP_50_Hz, gps_tick, LOOP_500_Hz))
    {
        if (_got_posllh || _got_velned)
        {
            flgt_ctl.sensor.gps.lat = m_gps_position.lat;
            flgt_ctl.sensor.gps.lon = m_gps_position.lon;
            flgt_ctl.sensor.gps.alt = m_gps_position.alt;
            flgt_ctl.sensor.gps.vel_n_m_s = m_gps_position.vel_n_m_s;
            flgt_ctl.sensor.gps.vel_e_m_s = m_gps_position.vel_e_m_s;
            flgt_ctl.sensor.gps.vel_d_m_s = m_gps_position.vel_d_m_s;
            flgt_ctl.sensor.gps._got_posllh = _got_posllh;
            flgt_ctl.sensor.gps._got_velned = _got_velned;
            flgt_ctl.sensor.gps.eph = m_gps_position.eph;
            flgt_ctl.sensor.gps.epv = m_gps_position.epv;
            _got_posllh = 0;
            _got_velned = 0;
        }
        else
        {
            flgt_ctl.sensor.gps._got_posllh = 0;
            flgt_ctl.sensor.gps._got_velned = 0;
        }
    }
    gps_tick++;
}

/**
 * @brief  传感器任务
 * @note
 * @param  无
 * @retval 无
 */
void Task_Sensor(void)
{
    Task_IMU();
    Task_MAG();
    Task_BARO();
    Task_GPS();
}

// /**
//  * @brief 获取加速度数据
//  * 
//  */
// void get_acc(Axis3_f_u *acc)
// {
//     *acc = imu_filter_data.acc;
// }

Axis3_f_u get_acc(void)
{
    __disable_irq();
    Axis3_f_u acc = imu_filter_data.acc;
    __enable_irq();
    return acc;
}

/**
 * @brief 获取角速度数据
 * 
 */
Axis3_f_u get_gyro(void)
{
    __disable_irq();
    Axis3_f_u gyro = imu_filter_data.gyro;
    __enable_irq();
    return gyro;
}

/**
 * @brief 获取磁力计数据
 * 
 */
MAG_Data_t get_mag(void)
{
    __disable_irq();
    MAG_Data_t mag =mag_cal_data;
    __enable_irq();
    return mag;
}

/**
 * @brief 获取气压计高度数据
 * 
 */
float get_baro(void)
{
    __disable_irq();
    float alt = sensor_handle.baro_proc_handle.altitude_m;
    __enable_irq();
    return alt;
}

// /**
//  * @brief 获取GPS数据
//  * 
//  */
// void get_gps(GPS_Data_t *gps)
// {
//     *gps = flgt_ctl.sensor.gps;
// }
