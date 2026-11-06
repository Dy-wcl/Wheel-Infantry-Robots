/**
 * @file gimbal_control.c
 * @brief 云台控制实现 (VT13 遥控器)
 */
#include "gimbal_control.h"
#include "bsp_log.h"

#include <math.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "can.h"
// #include "chassis_dynamics.h"
#include "comp_cmd.h"
#include "dj_motor_ctrl.h"
#include "pid_incremental.h"
#include "process.h"
#include "vt13.h"

/* VT13 task owns the decoded command and updates it periodically. */
extern vt13_t* vt13;

/* 云台电机总线与实例 */
static dj_motor_bus_t gimbal_bus;
static dj_motor_t gimbal_motors[2];
static err_t gimbal_bus_init_result;
static err_t gimbal_init_result[2];
/* 速度环 PID 实例 */
static PIDInstance_jie pid_speed[2];
static PIDInstance_jie pid_location[2];
/* 云台状态与目标速度 */
// static gimbal_control_state_t gimbal_control_state_;

/**
 * @brief 初始化云台 CAN 总线与两个 M3508 电机
 * @return OK 或错误码
 */
err_t gimbal_control_init(void) {
  LOG_INFO("云台计算", "开始初始化 VT13 云台 CAN 总线和电机");
  /* 获取 CAN1 的 BSP 对象（假设 CAN1 用于云台） */
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN1);
  if (can_id == BSP_CAN_ID_ERROR) {
    LOG_ERROR("云台计算", "未找到 VT13 云台 CAN1 对象");
    return NOT_FOUND;
  }

  STM32CAN_t* can1 = STM32CAN_GetInstance(can_id);
  if (can1 == NULL) {
    LOG_ERROR("云台计算", "VT13 云台 CAN1 控制块为空");
    return PTR_NULL;
  }

  /* 初始化云台总线 */
  gimbal_bus_init_result = dj_motor_bus_init(&gimbal_bus, can1);
  if (gimbal_bus_init_result != OK) {
    LOG_ERROR("云台计算", "VT13 云台总线初始化失败，错误码=%d",
              (int)gimbal_bus_init_result);
    return gimbal_bus_init_result;
  }

  /* 初始化两个 M3508 云台电机（控制组 0x200）
   * 实车电机编号与轮位的对应关系：
   *   偏航电机 YAW = 1 号电机
   *   俯仰电机 PITCH = 2 号电机
   * reversed 参数根据实际机械安装方向设置 */
  gimbal_init_result[0] =
      dj_motor_init(&gimbal_motors[0], &gimbal_bus, DJ_MOTOR_GM6020, 1,
                    false); /* 偏航YAW电机 = 1 号 */
  if (gimbal_init_result[0] != OK) {
    LOG_ERROR("云台计算", "VT13 偏航电机初始化失败，错误码=%d",
              (int)gimbal_init_result[0]);
    return gimbal_init_result[0];
  }

  gimbal_init_result[1] =
      dj_motor_init(&gimbal_motors[1], &gimbal_bus, DJ_MOTOR_GM6020, 2,
                    false); /* 俯仰PITCH电机 = 2 号 */
  if (gimbal_init_result[1] != OK) {
    LOG_ERROR("云台计算", "VT13 俯仰电机初始化失败，错误码=%d",
              (int)gimbal_init_result[1]);
    return gimbal_init_result[1];
  }

  LOG_INFO("云台计算", "VT13 云台 CAN 和两个电机初始化完成");
  return OK;
}
void gimbal_speed_pid_init(void) {
  // 初始化偏航电机速度 PID 参数
  PID_Init_Params_jie(
      &pid_speed[0], 12.0f, 0.0f, 0.0f, 0.0002f, 12000.0f, 3000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  PID_Init_Params_jie(
      &pid_location[0], 12.0f, 0.0f, 0.0f, 0.0002f, 12000.0f, 3000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  // 初始化俯仰电机速度 PID 参数
  PID_Init_Params_jie(
      &pid_speed[1], 8.0f, 0.0f, 0.0f, 0.0002f, 12000.0f, 3000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  PID_Init_Params_jie(
      &pid_location[1], 12.0f, 0.0f, 0.0f, 0.0002f, 12000.0f, 3000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  LOG_INFO("云台计算", "VT13 云台 PID 初始化完成");
}

void Gimbal_Mode(void) {
  static bool warning_logged = false;
  if (!warning_logged) {
    LOG_WARN("云台计算", "VT13 云台模式控制尚未实现");
    warning_logged = true;
  }
}
