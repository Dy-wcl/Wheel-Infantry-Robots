#include "vofa_task.h"
#include "bsp_log.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "task.h"
#include "usart.h"
#include "vofa.h"

#define VOFA_TASK_UPDATE_TIMEOUT_MS (10u)  //! VOFA 任务收发周期

//! VOFA 上行调试数据：默认发送 speed/angle/mit 三个浮点通道。
float data[3] = {0.0f, 0.0f, 0.0f};

//! 模块内部回调桥接：让 Vofa 驱动保持通用，任务层决定命令如何影响业务数据。
static void OnVofaCommand(const char* name, float value) {
  LOG_DEBUG("VOFA任务", "收到参数 name=%s value=%.3f",
            name != NULL ? name : "<空>", (double)value);
  VofaTask_OnCommand(name, value);
}

// //! 按名称读取 VOFA 最近收到的参数。
// bool VofaTask_GetParameter(const char *name, float *value)
// {
//   return Vofa_GetParameter(name, value);
// }

// //! 读取 speed 参数。
// bool VofaTask_GetSpeed(float *speed)
// {
//   return Vofa_GetSpeed(speed);
// }

// //! 读取 angle 参数。
// bool VofaTask_GetAngle(float *angle)
// {
//   return Vofa_GetAngle(angle);
// }

//! 默认命令处理：把 speed/angle/mit 写入上行调试数组。
//! 业务代码可提供强定义覆盖该弱函数，扩展更多命令。
__attribute__((weak)) void VofaTask_OnCommand(const char* name, float value) {
  if (strcmp(name, "speed") == 0) {
    data[0] = value;
  } else if (strcmp(name, "angle") == 0) {
    data[1] = value;
  } else if (strcmp(name, "mit") == 0) {
    data[2] = value;
  }
}

//! VOFA FreeRTOS 任务：初始化 USART6 收发，并周期性发送 firewater 数据帧。
void vofa_task(void* argument) {
  RM_UNUSED(argument);

  LOG_INFO("VOFA任务", "任务开始初始化 VOFA 串口");
  static Vofa_t vofa;  // 静态存储，保证任务整个生命周期内有效。
  err_t status = Vofa_Init(&vofa, &huart6);
  vofa.thread_alert = xTaskGetCurrentTaskHandle();
  Vofa_SetCommandCallback(&vofa, OnVofaCommand);

  if (status == OK) {
    status = Vofa_Start(&vofa);
  }
  ASSERT(status == OK);
  if (status != OK) {
    LOG_ERROR("VOFA任务", "VOFA 启动失败，错误码=%d", (int)status);
    vTaskDelete(NULL);
    return;
  }

  LOG_INFO("VOFA任务", "VOFA 已启动，进入收发循环");

  for (;;) {
    (void)Vofa_Send(&vofa, data,
                    ARRAY_LEN(data));  // 上行发送给 VOFA+ firewater 波形显示。
    Vofa_Update(&vofa, VOFA_TASK_UPDATE_TIMEOUT_MS);
  }
}
