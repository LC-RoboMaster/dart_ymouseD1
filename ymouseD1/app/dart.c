/**
  ******************************************************************************
  * @file    dart.c
  * @brief   飞镖机构的 4 路舵机
  *
  *          4 路舵机都挂在 TIM3 的 CH1~CH4 上:
  *            舵机1 -> PB4 (TIM3_CH1)
  *            舵机2 -> PB5 (TIM3_CH2)
  *            舵机3 -> PB0 (TIM3_CH3)
  *            舵机4 -> PB1 (TIM3_CH4)
  ******************************************************************************
  */

#include "dart.h"

#include "servo_motor.h"
#include "tim.h"

#define DART_SERVO_COUNT 4U

/* 4 路舵机的参数表:改参数只动这张表 */
static servo_motor_t s_servo[DART_SERVO_COUNT] = {
  { .htim = &htim3,
    .channel = TIM_CHANNEL_1,
    .min_angle = 0U,
    .max_angle = 180U,
    .min_pulse_us = 500U,
    .max_pulse_us = 2500U,
  },
  { .htim = &htim3, .channel = TIM_CHANNEL_2,
    .min_angle = 0U, .max_angle = 180U,
    .min_pulse_us = 500U, .max_pulse_us = 2500U },
  { .htim = &htim3, .channel = TIM_CHANNEL_3,
    .min_angle = 0U, .max_angle = 180U,
    .min_pulse_us = 500U, .max_pulse_us = 2500U },
  { .htim = &htim3, .channel = TIM_CHANNEL_4,
    .min_angle = 0U, .max_angle = 180U,
    .min_pulse_us = 500U, .max_pulse_us = 2500U },
};

void dart_init(void)
{
  uint8_t i;
  for (i = 0; i < DART_SERVO_COUNT; i++) {
    ServoMotor_Init(&s_servo[i]);
    ServoMotor_SetPosition(&s_servo[i], 90);
  }

}

void dart_stop(void)
{
  ServoMotor_SetPosition(&s_servo[0], 0);
  ServoMotor_SetPosition(&s_servo[1], 0);
  ServoMotor_SetPosition(&s_servo[2], 0);
  ServoMotor_SetPosition(&s_servo[3], 0);
}

void servo_motor_angleset(int angle0, int angle1, int angle2, int angle3)
{
  uint8_t i ;
  int angle[] = {angle0, angle1, angle2, angle3};

for (i = 0; i < DART_SERVO_COUNT; i++)
{
  ServoMotor_SetPosition(&s_servo[i], angle[i]);
}
}

void dart_control(void)
{
  servo_motor_angleset(90, 90, 90, 90);
}

void dart_task(void)
{
  dart_init();
  dart_control();
}
