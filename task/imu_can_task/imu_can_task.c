#include "imu_can_task.h"
#include "bsp_log.h"

#include "FreeRTOS.h"
#include "can_device.h"
#include "comp_utils.h"
#include "imu_can.h"
#include "task.h"

void imu_can_task(void* argument) {
  RM_UNUSED(argument);
  LOG_INFO("IMUCAN任务", "任务开始检查 CAN IMU");
  IMUCAN_t* imu = can_device_get_imu();
  if (imu == NULL) {
    LOG_ERROR("IMUCAN任务", "CAN IMU 对象为空，任务退出");
    vTaskDelete(NULL);
    return;
  }
  if (imu->init_error_ != OK) {
    LOG_ERROR("IMUCAN任务", "CAN IMU 初始化失败，错误码=%d",
              (int)imu->init_error_);
    vTaskDelete(NULL);
    return;
  }

  /* 设备已由 can_device 模块注册；任务只绑定自己的通知句柄并消费数据。 */
  imu->thread_alert = xTaskGetCurrentTaskHandle();
  LOG_INFO("IMUCAN任务", "CAN IMU 已就绪，进入数据更新循环");
  for (;;) imu_can_update(imu, 2u);
}
