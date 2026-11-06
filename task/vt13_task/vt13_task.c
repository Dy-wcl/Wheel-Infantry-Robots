#include "vt13_task.h"
#include "bsp_log.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "task.h"
#include "usart.h"

vt13_t* vt13 = NULL;

void vt13_task(void* argument) {
  RM_UNUSED(argument);
  LOG_INFO("VT13任务", "任务开始初始化遥控器");
  static vt13_t instance;
  err_t status = vt13_init(&instance, &huart3);
  vt13 = &instance;
  if (status == OK) status = vt13_start(vt13);
  ASSERT(status == OK);
  if (status != OK) {
    LOG_ERROR("VT13任务", "遥控器启动失败，错误码=%d", (int)status);
    vTaskDelete(NULL);
    return;
  }
  LOG_INFO("VT13任务", "遥控器已启动，进入数据更新循环");
  for (;;) vt13_update(vt13, 20u);
}
