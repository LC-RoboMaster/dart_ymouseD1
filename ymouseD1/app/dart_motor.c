//
// Created by 13466 on 2026/9/25.
//

#include "dart_motor.h"
#include "robomaster.h"
#include "robot_data.h"
#include "servo_motor.h"
#include "tim.h"
void dart_init();
void dart_control();
servo_motor_t servo_motor1;
servo_motor_t servo_motor2;
servo_motor_t servo_motor3;
servo_motor_t servo_motor4;
void dart_motor_task()
{


    /* TODO: dart_control() 还没实现
     *
     * 注意:这里原来是个 `for (;;) { void dart_control(); }` —— 内层那句是
     * "函数声明"而不是函数调用,所以循环体是空的,等于一个空死循环。
     * 而本函数由最高优先级的 master_task 调用,一旦不返回就会把
     * imu / message / defaultTask 全部饿死(表现为 IMU 永远读不到数据、
     * 看门狗也没被启动)。所以这个循环不能留,改由 master_task 的循环驱动。
     */
}
void dart_init()
{
    servo_motor_t  servo_motor1 ={
        .htim = &htim3,
        .channel = 1,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
    servo_motor_t servo_motor2 ={
        .htim = &htim3,
        .channel = 2,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
    servo_motor_t servo_motor3={
        .htim = &htim3,
        .channel = 3,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
    servo_motor_t servo_motor4 ={
        .htim = &htim3,
        .channel = 4,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
}
void dart_stop()
{
    ServoMotor_SetPosition( &servo_motor1,5);
    ServoMotor_SetPosition( &servo_motor2,0);
    ServoMotor_SetPosition( &servo_motor3,0);
    ServoMotor_SetPosition( &servo_motor4,0);
}
void servo_motor_angleset(int angle1,int angle2,int angle3,int angle4)
{
    ServoMotor_SetPosition( &servo_motor1,angle1);
    ServoMotor_SetPosition( &servo_motor2,angle2);
    ServoMotor_SetPosition( &servo_motor3,angle3);
    ServoMotor_SetPosition( &servo_motor4,angle4);
}