#include "gpio.h"
#include "user_taskInit.h"
#include "display_task.h"
#include "imu_task.h"
#include "motor_task.h"

osMessageQueueId_t imuInitResultQueueHandle;
osMessageQueueId_t imuPoseQueueHandle;

osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledDisplay_Task",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imu_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "stepper_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


void ledTask(void *argument);
static void App_assert(void);


void userTaskInit(void)
{
    /* add queues, ... */
    imuInitResultQueueHandle =  osMessageQueueNew(1, sizeof(uint8_t), NULL);
    imuPoseQueueHandle =        osMessageQueueNew(1, sizeof(imu_pose_t), NULL);

    /* add threads, ... */
    ledTaskHandle =             osThreadNew(ledTask, NULL, &ledTask_attributes);
    if(ledTaskHandle == NULL)   App_assert(); 
    oledTaskHandle =            osThreadNew(oledDisplay_Task, NULL, &oledTask_attributes);
    if(oledTaskHandle == NULL)  App_assert();
    imuTaskHandle =             osThreadNew(imu_Task, NULL, &imuTask_attributes);
    if(imuTaskHandle == NULL)   App_assert();
    motorTaskHandle =           osThreadNew(stepper_Task, NULL, &motorTask_attributes);
    if(motorTaskHandle == NULL) App_assert();
}

void ledTask(void *argument)
{
  /* USER CODE BEGIN ledTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
    osDelay(1000);
  }
  /* USER CODE END ledTask */
}

static void App_assert(void)
{
  taskDISABLE_INTERRUPTS();
  for(;;);
}
