//
// Created by 13466 on 2026/9/25.
//

#ifndef YMOUSED1_IMU_H
#define YMOUSED1_IMU_H

#include <stdint.h>

/**
 * @brief JY901S 一帧姿态数据(串口主动推送,解析后写入)
 * @note  raw 字段为数据帧原始值,工程量同步换算;单写多读,其他任务直接读取
 */
typedef struct
{
    //加速度原始值(±16g 量程)
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    //角速度原始值(±2000°/s 量程)
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    //温度原始值(T = raw/100 ℃)
    int16_t temperature_raw;
    //角度原始值(±180°)
    int16_t angle_x_raw;          //roll
    int16_t angle_y_raw;          //pitch
    int16_t angle_z_raw;          //yaw

    //换算后的工程量
    float roll_deg;               //横滚角,°(±180)
    float pitch_deg;              //俯仰角,°(±90)
    float yaw_deg;                //航向角,°(±180,不累计展开)
    float gyro_dps[3];            //角速度 x/y/z,°/s
    float acc_g[3];               //加速度 x/y/z,g
    float temp_c;                 //温度,℃

    //状态
    uint8_t online;               //模块在线标志(帧流超时即离线)
    uint32_t err_cnt;             //校验/接收错误总次数
} imu_data_t;

extern imu_data_t g_imu;

#endif //YMOUSED1_IMU_H
