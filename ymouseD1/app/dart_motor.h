#ifndef YMOUSED1_DART_MOTOR_H
#define YMOUSED1_DART_MOTOR_H

/* 4 路舵机(飞镖机构)的控制接口 */

/** @brief 初始化 4 路舵机并启动 PWM(在 master_task 里调用一次) */
void dart_init(void);

/**
 * @brief 控制任务的一次执行,由 master_task 的循环驱动
 * @note  必须先调用 dart_init();且本函数必须立即返回 ——
 *        master 是最高优先级任务,它不返回会把其它任务饿死
 */
void dart_motor_task(void);

/** @brief 全部回到初始位置:舵机1 到 5°,其余到 0° */
void dart_stop(void);

/** @brief 一次设置 4 路舵机的角度,单位度 */
void servo_motor_angleset(int angle1, int angle2, int angle3, int angle4);

#endif /* YMOUSED1_DART_MOTOR_H */
