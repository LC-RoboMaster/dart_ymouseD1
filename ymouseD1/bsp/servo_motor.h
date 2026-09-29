#ifndef YMOUSED1_SERVO_MOTOR_H
#define YMOUSED1_SERVO_MOTOR_H

#include <stdint.h>
#include "main.h"

/**
 * @brief 舵机结构体
 */
typedef struct {
  TIM_HandleTypeDef *htim; // 定时器指针
  uint8_t servo_motor_id;
  uint32_t channel;
  uint16_t min_pluse; // 最小脉宽 us（对应最小角度）
  uint16_t max_pluse; // 最大脉宽 us
  uint8_t min_angle;  // 最小角度（一般 0）
  uint8_t max_angle;  // 最大角度（一般 180）
  uint16_t cur_pluse; // 当前脉宽（运行时状态）
} servo_motor_t;

int ServoMotor_SetPosition(servo_motor_t *servo_motor, int angle);


#endif /* YMOUSED1_SERVO_MOTOR_H */