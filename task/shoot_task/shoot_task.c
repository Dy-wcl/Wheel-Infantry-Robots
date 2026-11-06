/**
 * @file shoot_task.c
 * @brief 发射控制任务实现
 */
#include "shoot_task.h"
#include "bsp_log.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "shoot_control.h"
#include "task.h"

/* 发射任务状态（用于调试和错误监控） */
volatile err_t shoot_status = PENDING;

/**
 * @brief 发射控制任务入口函数
 * @param argument FreeRTOS 任务参数（未使用）
 *
 * @details 任务执行流程：
 *   1. 使用 main.c 已完成注册并启动的 CAN2 设备
 *   2. 初始化发射电机速度环 PID 参数
 *   3. 周期性调用发射控制函数（2ms 周期）
 */
void shoot_task(void* argument) {
  RM_UNUSED(argument);

  LOG_INFO("发射任务", "任务开始初始化发射控制");
  /* CAN2 及发射电机已由 main.c 完成注册和启动。 */
  shoot_status = OK;

  /* 初始化发射电机速度环 PID */
  shoot_speed_pid_init();
  LOG_INFO("发射任务", "发射机构 PID 已初始化");

  /* 主控制循环 */
  while (1) {
    /* 执行发射模式控制（根据遥控器状态） */
    Shoot_Mode();

    /* 2ms 控制周期 */
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
