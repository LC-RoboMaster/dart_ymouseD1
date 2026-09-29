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
    dart_init();
    for (;;)
    {
        void dart_control();

    }

}
void dart_init()
{
    servo_motor_t  servo_motor1 ={
        .htim = &htim1,
        .channel = 1,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
    servo_motor_t servo_motor2 ={
        .htim = &htim1,
        .channel = 2,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
    servo_motor_t servo_motor3={
        .htim = &htim1,
        .channel = 3,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
    servo_motor_t servo_motor4 ={
        .htim = &htim1,
        .channel = 4,
        .max_angle = 180,
        .min_angle = 0,
        .max_pluse = 2500,
        .min_pluse = 500,
    };
}
void dart_stop()
{
    ServoMotor_SetPosition( &servo_motor1,0);
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