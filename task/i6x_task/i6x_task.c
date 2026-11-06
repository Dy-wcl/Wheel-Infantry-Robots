#include "i6x_task.h"
#include "bsp_log.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "i6x.h"
#include "task.h"
#include "usart.h"

//! 全局 I6x 对象指针，供其他业务模块读取遥控器状态。
I6X_t* i6x = NULL;

//! I6x FreeRTOS 任务：初始化 USART1 接收，并周期性刷新在线状态和解析结果。
void i6x_task(void* argument) {
  RM_UNUSED(argument);

  LOG_INFO("I6X任务", "任务开始初始化遥控器");
  static I6X_t i6x_instance;  // 静态存储，保证任务整个生命周期内有效。
  err_t status = I6X_Init(&i6x_instance, &huart4);
  i6x = &i6x_instance;

  i6x->thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) {
    status = I6X_Start(i6x);
  }
  ASSERT(status == OK);
  if (status != OK) {
    LOG_ERROR("I6X任务", "遥控器启动失败，错误码=%d", (int)status);
    vTaskDelete(NULL);
    return;
  }

  LOG_INFO("I6X任务", "遥控器已启动，进入数据更新循环");

  for (;;) {
    I6X_Update(i6x, I6X_OFFLINE_TIMEOUT_MS);
  }
}
