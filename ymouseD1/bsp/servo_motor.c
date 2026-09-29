#include "servo_motor.h"
#include "main.h"
#include "robot_data.h"
#include "tim.h"

/**
 * @brief  舵机初始化函数
 * @param  servo_motor: 舵机结构体指针
 * @retval 0:初始化成功
 */
int ServoMotor_Init(servo_motor_t *servo_motor) {
  if (servo_motor == NULL || servo_motor->htim == NULL) {
    return -1; // 参数错误
  }

  // 启动定时器 PWM 输出
  HAL_TIM_PWM_Start(servo_motor->htim, servo_motor->channel);

  return 0; // 初始化成功
}
/**
 *@brief 舵机控制函数
 *@param
 *
 */
int ServoMotor_SetPosition(servo_motor_t *servo_motor, int angle){
    if (servo_motor == NULL || servo_motor->htim == NULL)
    {
        return -1;
    }
    //角度转换脉宽
    uint16_t cur_pluse= servo_motor->min_pluse +
                        ((uint32_t)(angle - servo_motor->min_angle) *
                        (servo_motor->max_pluse - servo_motor->min_pluse)) /
                        (servo_motor->max_angle - servo_motor->min_angle);

    //pwm输出
    __HAL_TIM_SET_COMPARE(servo_motor->htim, servo_motor->channel, cur_pluse);


}