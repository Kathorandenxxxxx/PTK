#include "user_taskInit.h"
#include "mpu6050.h"
#include "imu_task.h"
#include "task.h"

static imu_pose_t s_pose;

static mpu6050_attitude_t att = {
  .gyro_x      = 0,
  .gyro_y      = 0,
  .gyro_z      = 0,
  .accel_x     = 0,
  .accel_y     = 0,
  .accel_z     = 0,
  .gx_offset   = 0,
  .gy_offset   = 0,
  .gz_offset   = 0,
  .pitch       = 0.0f,
  .roll        = 0.0f,
  .yaw         = 0.0f,
  .last_time   = 0
};

/* roll 是积分出来的，会越过 ±180 突跳 → 目标步数会瞬间跳 ±4096，电机疯转一圈。
   这里先做 unwrap 变成连续角，再限幅到机械行程。 */
static float IMU_UnwrapRoll(float roll)
{
    static float prev = 0.0f;
    static float cont = 0.0f;

    float d = roll - prev;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    cont += d;
    prev  = roll;

    return cont;
}

static void IMU_PublishPose(void)
{
    imu_pose_t pose;
    pose.roll = IMU_UnwrapRoll(att.roll);  
    pose.tick  = att.last_time;

    taskENTER_CRITICAL();          // ~1us，保证多字段一致
    s_pose = pose;
    taskEXIT_CRITICAL();

    // 长度 1 的邮箱，覆盖式：宁可丢旧帧，也不能让控制环用过期 roll
    if (osMessageQueuePut(imuPoseQueueHandle, &pose, 0U, 0U) != osOK) {
        imu_pose_t stale;
        (void)osMessageQueueGet(imuPoseQueueHandle, &stale, NULL, 0U);  // 丢弃
        (void)osMessageQueuePut(imuPoseQueueHandle, &pose,  0U, 0U);
    }
}

void imu_GetPose(imu_pose_t *out)
{
    taskENTER_CRITICAL(); 
    *out = s_pose; 
    taskEXIT_CRITICAL();
}

void imu_Task(void *argument) {
    uint8_t mpu_init_result = MPU6050_Init();

    osMessageQueuePut(imuInitResultQueueHandle, &mpu_init_result, 0, 0);
    // if(mpu_init_result) return;

    MPU6050_Calibrate_GyroAcc(&att);
    att.last_time = HAL_GetTick(); // get the current time in milliseconds
    
    while (1) 
    {
        MPU6050_KalmanUpdate(&att);
        IMU_PublishPose();
        osDelay(10); // 延时10ms，确保任务周期为10ms
    }
}
