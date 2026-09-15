#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

//定义快照
typedef struct {
    float    roll;
    uint32_t tick;      // 解算时刻，用于接收方算真实 dt / 判新鲜度
} imu_pose_t;

void imu_Task(void *parameter);
void imu_GetPose(imu_pose_t *out); //给低频消费者OLED等
#endif // __IMU_TASK_H__
