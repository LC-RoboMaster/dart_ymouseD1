/**
  ******************************************************************************
  * @file    dart_motor.c
  * @brief   飞镖机构的 4 路舵机
  *
  *          4 路舵机都挂在 TIM3 的 CH1~CH4 上:
  *            舵机1 -> PB4 (TIM3_CH1)
  *            舵机2 -> PB5 (TIM3_CH2)
  *            舵机3 -> PB0 (TIM3_CH3)
  *            舵机4 -> PB1 (TIM3_CH4)
  ******************************************************************************
  */

#include "dart_motor.h"

#include "servo_motor.h"
#include "tim.h"

#define DART_SERVO_COUNT 4U

/* 4 路舵机的参数表:改参数只动这张表 */
static servo_motor_t s_servo[DART_SERVO_COUNT] = {
  { .htim = &htim3, .channel = TIM_CHANNEL_1,
    .min_angle = 0U, .max_angle = 180U,
    .min_pulse_us = 500U, .max_pulse_us = 2500U },
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

  for (i = 0U; i < DART_SERVO_COUNT; i++) {
    (void)ServoMotor_Init(&s_servo[i]);
  }
}

void dart_motor_task(void)
{
  /* TODO: dart_control() 还没实现
   *
   * 这里必须立即返回。本函数由最高优先级的 master_task 的循环驱动,
   * 一旦在里面写死循环,imu / message / defaultTask 会全部被饿死
   * (曾踩过:那里写了个 for(;;) 空循环,导致 IMU 永远读不到数据、
   * 看门狗也没被启动)。*/
}

void dart_stop(void)
{
  ServoMotor_SetPosition(&s_servo[0], 5);
  ServoMotor_SetPosition(&s_servo[1], 0);
  ServoMotor_SetPosition(&s_servo[2], 0);
  ServoMotor_SetPosition(&s_servo[3], 0);
}

void servo_motor_angleset(int angle1, int angle2, int angle3, int angle4)
{
  ServoMotor_SetPosition(&s_servo[0], angle1);
  ServoMotor_SetPosition(&s_servo[1], angle2);
  ServoMotor_SetPosition(&s_servo[2], angle3);
  ServoMotor_SetPosition(&s_servo[3], angle4);
}
