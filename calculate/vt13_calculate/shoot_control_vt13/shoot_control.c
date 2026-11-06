/**
 * @file shoot_control.c
 * @brief 发射机构控制实现 (VT13 遥控器)
 */
#include "shoot_control.h"
#include "bsp_log.h"

#include <math.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "can.h"
#include "convert.h"
#include "dj_motor_ctrl.h"
#include "pid_incremental.h"
#include "process.h"
#include "vt13.h"

/* VT13 task owns the decoded command and updates it periodically. */
extern vt13_t* vt13;

/* 发射机构电机总线与实例 */
static dj_motor_bus_t shoot_bus;
static dj_motor_t shoot_motors[3];
static err_t shoot_bus_init_result;
static err_t shoot_init_result[3];
/* 速度环 PID 实例：三台电机各一个 */
static PIDInstance_jie pid_speed[3];
/* 位置环 PID 实例：仅拨弹电机 M2006 需要位置环 */
static PIDInstance_jie pid_location;

/**
 * @brief 初始化发射 CAN 总线与三台发射电机
 * @return OK 或错误码
 */
err_t shoot_control_init(void) {
  LOG_INFO("发射计算", "开始初始化 VT13 发射 CAN 总线和电机");
  /* 获取 CAN1 的 BSP 对象（假设 CAN1 用于发射机构，与云台同总线）
   * CAN2 上底盘已占用 0x200 控制组的 1-4 号电机，发射机构不能复用同组 ID */
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN1);
  if (can_id == BSP_CAN_ID_ERROR) {
    LOG_ERROR("发射计算", "未找到 VT13 发射 CAN1 对象");
    return NOT_FOUND;
  }

  STM32CAN_t* can1 = STM32CAN_GetInstance(can_id);
  if (can1 == NULL) {
    LOG_ERROR("发射计算", "VT13 发射 CAN1 控制块为空");
    return PTR_NULL;
  }

  /* 初始化发射机构总线 */
  shoot_bus_init_result = dj_motor_bus_init(&shoot_bus, can1);
  if (shoot_bus_init_result != OK) {
    LOG_ERROR("发射计算", "VT13 发射总线初始化失败，错误码=%d",
              (int)shoot_bus_init_result);
    return shoot_bus_init_result;
  }

  /* 初始化发射机构电机（控制组 0x200）
   * 实车电机编号与机构的对应关系：
   *   左摩擦轮 = 1 号电机（M3508，单速度环）
   *   右摩擦轮 = 2 号电机（M3508，单速度环）
   *   拨弹电机 = 3 号电机（M2006，速度位置双环）
   * reversed 参数根据实际机械安装方向设置 */
  shoot_init_result[0] =
      dj_motor_init(&shoot_motors[0], &shoot_bus, DJ_MOTOR_M3508, 1,
                    false); /* 左摩擦轮 = 1 号 */
  if (shoot_init_result[0] != OK) {
    LOG_ERROR("发射计算", "VT13 左摩擦轮初始化失败，错误码=%d",
              (int)shoot_init_result[0]);
    return shoot_init_result[0];
  }

  shoot_init_result[1] =
      dj_motor_init(&shoot_motors[1], &shoot_bus, DJ_MOTOR_M3508, 2,
                    false); /* 右摩擦轮 = 2 号 */
  if (shoot_init_result[1] != OK) {
    LOG_ERROR("发射计算", "VT13 右摩擦轮初始化失败，错误码=%d",
              (int)shoot_init_result[1]);
    return shoot_init_result[1];
  }

  shoot_init_result[2] =
      dj_motor_init(&shoot_motors[2], &shoot_bus, DJ_MOTOR_M2006, 3,
                    false); /* 拨弹电机 = 3 号 */
  if (shoot_init_result[2] != OK) {
    LOG_ERROR("发射计算", "VT13 拨弹电机初始化失败，错误码=%d",
              (int)shoot_init_result[2]);
    return shoot_init_result[2];
  }

  LOG_INFO("发射计算", "VT13 发射 CAN 和三台电机初始化完成");
  return OK;
}

void shoot_speed_pid_init(void) {
  // 初始化左摩擦轮电机速度 PID 参数（M3508 单速度环）
  PID_Init_Params_jie(
      &pid_speed[0], 10.0f, 0.0f, 0.0f, 0.0002f, 12000.0f, 3000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  // 初始化右摩擦轮电机速度 PID 参数（M3508 单速度环）
  PID_Init_Params_jie(
      &pid_speed[1], 10.0f, 0.0f, 0.0f, 0.0002f, 12000.0f, 3000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  // 初始化拨弹电机速度环 PID 参数（M2006 双环内环，输出电流限幅 10000）
  PID_Init_Params_jie(
      &pid_speed[2], 8.0f, 0.0f, 0.0f, 0.0002f, 10000.0f, 3000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  // 初始化拨弹电机位置环 PID（仅 M2006 需要位置环，输出为目标速度）
  PID_Init_Params_jie(
      &pid_location, 8.0f, 0.0f, 0.0f, 0.0002f, 3000.0f, 1000.0f, 0.0f,
      PID_INC_INTEGRAL_LIMIT | PID_INC_DERIVATIVE_ON_MEASUREMENT |
          PID_INC_OUTPUT_FILTER | PID_INC_DERIVATIVE_FILTER);
  LOG_INFO("发射计算", "VT13 发射 PID 初始化完成");
}
void dj_motor_pid_control_location(uint8_t motor_id, float target_angle) {
  dj_motor_feedback_t feedback;
  if ((dj_motor_get_feedback(&shoot_motors[motor_id], &feedback) != OK) ||
      !feedback.valid) {
    return;
  }

  /* 直接使用驱动提供的逻辑方向转子单圈角度。 */
  const float current_angle_deg = feedback.position_deg;
  const float normalized_target =
      GetAngleBetween180(GetAngleBetween360(target_angle));

  const float angle_error =
      GetAngleBetween180(normalized_target - current_angle_deg);
  const float target_speed =
      PID_Calculate_jie(&pid_location, angle_error, 0.0f);
  float motor_current = PID_Calculate_jie(&pid_speed[motor_id], target_speed,
                                          (float)feedback.speed_rpm);

  const float current_limit =
      (float)dj_motor_drv_command_limit(shoot_motors[motor_id].type);
  motor_current = CONSTRAIN(motor_current, -current_limit, current_limit);
  (void)dj_motor_set_command(&shoot_motors[motor_id], (int16_t)motor_current);
}

/**
 * @brief 安全停止发射机构所有电机
 * @details 将所有发射电机的命令清零并立即发送
 */
static void shoot_stop(void) {
  /* 发射机构所有电机（ID 1-3）都在 0x200 控制组，只需清空该组 */
  dj_motor_zero_and_flush(&shoot_bus, DJ_MOTOR_GROUP_200);
}

void Shoot_Mode(void) {
  static bool offline_logged = false;
  if ((vt13 == NULL) || !vt13->online_) {
    if (!offline_logged) {
      LOG_WARN("发射计算", "VT13 遥控器离线，发射机构停止");
      offline_logged = true;
    }
    shoot_stop();
    return;
  }
  if (offline_logged) {
    LOG_INFO("发射计算", "VT13 遥控器恢复在线");
    offline_logged = false;
  }
  // TODO: 根据VT13遥控器按键状态实现发射模式控制逻辑
  static bool warning_logged = false;
  if (!warning_logged) {
    LOG_WARN("发射计算", "VT13 发射模式控制尚未实现");
    warning_logged = true;
  }
}
