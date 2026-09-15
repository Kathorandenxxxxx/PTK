#include "user_taskInit.h"
#include "imu_task.h"
#include "oled.h"
#include "string.h"
#include "stepper.h"
#include "mpu6050.h"


void oledDisplay_Task(void *argument) {
    uint8_t init_result;
    char roll_str[10], tar_step[10], cur_step[10];
    imu_pose_t pose = {0.0f, 0};

    OLED_Init();
    
    osMessageQueueGet(imuInitResultQueueHandle, &init_result, NULL, osWaitForever);
    if(init_result) OLED_ShowString(1, 1, "init failed!");
    else            OLED_ShowString(1, 1, "init success!");
    
    while (1) 
    {
        imu_GetPose(&pose);
        OLED_ShowString(2, 1, "roll:");
        snprintf(roll_str, sizeof(roll_str), "%.1f  ", pose.roll);
        OLED_ShowString(2, 6, roll_str);

        // OLED_ShowString(3, 1, "tarstp:");
        // sprintf(tar_step, "%.1f  ", rollmoto_state.target_steps);
        // OLED_ShowString(3, 8, tar_step);
        
        OLED_ShowString(4, 1, "curstp:");
        snprintf(cur_step, sizeof(cur_step), "%ld  ", Motor_GetCurrentSteps());
        OLED_ShowString(4, 8, cur_step);
        osDelay(200);
    }
}
