#include "cmsis_os2.h"
#include "main.h"
#include "servo_motor.h"
#include "dart.h"
#include "imu.h"

void master_task(void *argument) {
  (void)argument;

  /* USER CODE BEGIN 5 */
    dart_init(); //镖体初始化
  /* Infinite loop */
  for (;;) {


    dart_task();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

