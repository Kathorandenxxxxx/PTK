#include "user_taskInit.h"
#include "imu_task.h"
#include "stepper.h"
#include "pid.h"
#include "ramp.h"
#include "mpu6050.h"

#define PID_DEADBAND             6.0f // 死区范围 (步)
#define STEP_ACCEL_MAX          6000.0f   /* 步/s²，加减速斜坡上限 */
/* 机械行程硬限位 */
#define MOTOR_ANGLE_MIN  (-180.0f)
#define MOTOR_ANGLE_MAX  ( 180.0f)
#define MOTOR_STEP_MIN   ((int32_t)(MOTOR_ANGLE_MIN * (float)STEP_PER_REV / 360.0f))
#define MOTOR_STEP_MAX   ((int32_t)(MOTOR_ANGLE_MAX * (float)STEP_PER_REV / 360.0f))

static PID_t s_pid;
static Ramp_t s_ramp;
static Stepper_t s_motor;


/* 只读访问接口，供显示层等外部消费者使用 */
// float Motor_GetTargetSteps(void)
// {
  
// }

int32_t Motor_GetCurrentSteps(void)
{
  return Stepper_GetSteps(&s_motor);
}

void stepper_Task(void *argument) {
  imu_pose_t pose = {0.0f, 0u};
  uint32_t last_tick;

  Stepper_Init(&s_motor);
  Stepper_SetLimit(&s_motor, MOTOR_STEP_MIN, MOTOR_STEP_MAX);
  PID_Init(&s_pid, 5.0f, 0.0f, 0.0f, 500.0f, STEP_FREQ_MAX_HZ, PID_DEADBAND);
  Ramp_Init(&s_ramp, STEP_ACCEL_MAX);
  
  while (osMessageQueueGet(imuPoseQueueHandle, &pose, NULL, osWaitForever) != osOK) { }
  last_tick = osKernelGetTickCount();
  
  while (1) {
    osStatus_t st = osMessageQueueGet(imuPoseQueueHandle, &pose, NULL, 50U);

    uint32_t now = osKernelGetTickCount();
    float dt = (float)(now - last_tick) * 0.001f;
    last_tick = now;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f)  dt = 0.05f;      // 长时间阻塞后不让积分爆掉

    if (st != osOK) 
    {                 // IMU 失联 → 安全停机，不是保持最后指令继续转
      PID_Reset(&s_pid);
      Ramp_Reset(&s_ramp, 0.0f);
      // Stepper_SetSpeed(0.0f, &rollmoto_state);
      Stepper_SetSpeed(&s_motor, 0.0f);
      continue;
    }
    float angle = pose.roll;
    if(angle > MOTOR_ANGLE_MAX) angle = MOTOR_ANGLE_MAX;
    if(angle < MOTOR_ANGLE_MIN) angle = MOTOR_ANGLE_MIN;
    float v_cmd = PID_Update(&s_pid, angle, dt);
    float v_actual = Ramp_Update(&s_ramp, v_cmd, dt);
    Stepper_SetSpeed(&s_motor, v_actual);
  }
}


