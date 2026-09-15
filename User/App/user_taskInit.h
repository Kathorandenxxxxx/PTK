#ifndef __USER_TASKINIT_H_
#define __USER_TASKINIT_H_
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

extern osMessageQueueId_t imuInitResultQueueHandle;
extern osMessageQueueId_t imuPoseQueueHandle;

void userTaskInit(void);

#endif

