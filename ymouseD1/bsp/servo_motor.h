#ifndef YMOUSED1_SERVO_MOTOR_H
#define YMOUSED1_SERVO_MOTOR_H

#include <stdint.h>

#include "main.h"

/**
 * @brief 一路舵机的配置与运行时状态
 *
 * 角度 -> 脉宽 是线性映射:
 *   脉宽 = min_pulse_us + (角度 - min_angle) * (max_pulse_us - min_pulse_us)
 *                          / (max_angle - min_angle)
 */
typedef struct {
  TIM_HandleTypeDef *htim; /* 输出 PWM 的定时器,例如 &htim3 */
  uint32_t channel;        /* HAL 通道号,必须用 TIM_CHANNEL_x,不要写裸数字 */
  uint16_t min_pulse_us;   /* 最小角度对应的脉宽,单位 us(常见 500) */
  uint16_t max_pulse_us;   /* 最大角度对应的脉宽,单位 us(常见 2500) */
  uint8_t min_angle;       /* 最小角度(一般为 0) */
  uint8_t max_angle;       /* 最大角度(一般为 180) */
  uint16_t cur_pulse_us;   /* 当前脉宽,运行时由 ServoMotor_SetPosition() 更新 */
} servo_motor_t;

/**
 * @brief  启动该路舵机的 PWM 输出
 * @param  servo 舵机对象;不能为 NULL,且 htim 必须已初始化
 * @retval 0 成功, -1 参数错误
 */
int ServoMotor_Init(servo_motor_t *servo);

/**
 * @brief  设置舵机角度
 * @param  servo 舵机对象
 * @param  angle 目标角度(度),超出 [min_angle, max_angle] 会被限幅
 * @retval 0 成功, -1 参数错误
 */
int ServoMotor_SetPosition(servo_motor_t *servo, int angle);

#endif /* YMOUSED1_SERVO_MOTOR_H */
