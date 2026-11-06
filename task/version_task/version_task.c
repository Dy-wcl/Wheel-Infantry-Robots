#include "version_task.h"
#include "bsp_log.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "task.h"

void version_task(void* argument);
{
  UNUSED(argument);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}