/**
 * @file gimbal_task.c
 * @brief 云台控制任务实现
 */
#include "gimbal_task.h"
#include "bsp_log.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "gimbal_control.h"
#include "task.h"

/* 云台任务状态（用于调试和错误监控） */
volatile err_t gimbal_status = PENDING;

/**
 * @brief 云台控制任务入口函数
 * @param argument FreeRTOS 任务参数（未使用）
 *
 * @details 任务执行流程：
 *   1. 使用 main.c 已完成注册并启动的 CAN2 设备
 *   2. 初始化云台电机速度环 PID 参数
 *   3. 周期性调用云台控制函数（2ms 周期）
 */
void gimbal_task(void* argument) {
  RM_UNUSED(argument);

  LOG_INFO("云台任务", "任务开始初始化云台控制");
  /* CAN2 及云台电机已由 main.c 完成注册和启动。 */
  gimbal_status = OK;

  /* 初始化云台电机速度环 PID */
  gimbal_speed_pid_init();
  LOG_INFO("云台任务", "云台 PID 已初始化");

  /* 主控制循环 */
  while (1) {
    /* 执行云台模式控制（根据遥控器状态） */
    Gimbal_Mode();

    /* 2ms 控制周期 */
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
