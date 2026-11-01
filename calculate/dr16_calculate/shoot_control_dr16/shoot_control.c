/**
 * @file shoot_control.c
 * @brief 发射机构控制实现 (DR16 遥控器)
 */
#include "shoot_control.h"

#include <math.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "can.h"
#include "comp_utils.h"
#include "dj_motor_ctrl.h"
#include "dr16.h"
#include "pid_location.h"

/* DR16 task owns the decoded command and updates it periodically. */
extern DR16_t *dr16;

/* 发射机构电机总线与实例 */
static dj_motor_bus_t shoot_bus;
static dj_motor_t shoot_motors[3];
static err_t shoot_bus_init_result;
static err_t shoot_init_result[3];
/* 速度环 PID 实例：三台电机各一个 */
static PIDInstance pid_speed[3];
static err_t pid_init_result[3];
/* 位置环 PID 实例：仅拨弹电机 M2006 需要位置环 */
static PIDInstance pid_location;
static pid_device_t shoot_pid_devices[3];

/**
 * @brief 初始化发射 CAN 总线与三台发射电机
 * @return OK 或错误码
 */
err_t shoot_control_init(void) {
  /* 获取 CAN1 的 BSP 对象（假设 CAN1 用于发射机构，与云台同总线）
   * CAN2 上底盘已占用 0x200 控制组的 1-4 号电机，发射机构不能复用同组 ID */
  ASSERT(CAN1 != NULL);
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN1);
  ASSERT(can_id != BSP_CAN_ID_ERROR);
  if (can_id == BSP_CAN_ID_ERROR) {
    return NOT_FOUND;
  }

  STM32CAN_t *can1 = STM32CAN_GetInstance(can_id);
  ASSERT(can1 != NULL);
  if (can1 == NULL) {
    return PTR_NULL;
  }

  /* 初始化发射机构总线 */
  shoot_bus_init_result = dj_motor_bus_init(&shoot_bus, can1);
  VERIFY(shoot_bus_init_result == OK);
  if (shoot_bus_init_result != OK) {
    return shoot_bus_init_result;
  }

  /* 初始化发射机构电机（控制组 0x200）
   * 实车电机编号与机构的对应关系：
   *   左摩擦轮 = 1 号电机（M3508，单速度环）
   *   右摩擦轮 = 2 号电机（M3508，单速度环）
   *   拨弹电机 = 3 号电机（M2006，速度位置双环）
   * reversed 参数根据实际机械安装方向设置 */
  shoot_init_result[0] =
      dj_motor_init(&shoot_motors[0], &shoot_bus, DJ_MOTOR_M3508, 7,
                    false); /* 左摩擦轮 = 1 号 */
  VERIFY(shoot_init_result[0] == OK);
  if (shoot_init_result[0] != OK)
    return shoot_init_result[0];

  shoot_init_result[1] =
      dj_motor_init(&shoot_motors[1], &shoot_bus, DJ_MOTOR_M3508, 8,
                    false); /* 右摩擦轮 = 2 号 */
  VERIFY(shoot_init_result[1] == OK);
  if (shoot_init_result[1] != OK)
    return shoot_init_result[1];

  shoot_init_result[2] =
      dj_motor_init(&shoot_motors[2], &shoot_bus, DJ_MOTOR_M2006, 3,
                    false); /* 拨弹电机 = 3 号 */
  VERIFY(shoot_init_result[2] == OK);
  if (shoot_init_result[2] != OK)
    return shoot_init_result[2];

  return OK;
}

void shoot_speed_pid_init(void) {
  // 初始化左摩擦轮电机速度 PID 参数（M3508 单速度环）
  PIDInit(&pid_speed[0], 10.0f, 0.0f, 0.0f, 12000.0f, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.05f, 0.1f);
  // 初始化右摩擦轮电机速度 PID 参数（M3508 单速度环）
  PIDInit(&pid_speed[1], 10.0f, 0.0f, 0.0f, 12000.0f, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.05f, 0.1f);
  // 初始化拨弹电机速度环 PID 参数（M2006 双环内环，输出电流限幅 10000）
  PIDInit(&pid_speed[2], 8.0f, 0.0f, 0.0f, 10000.0f, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.05f, 0.1f);
  // 初始化拨弹电机位置环 PID（仅 M2006 需要位置环，输出为目标速度）
  PIDInit(&pid_location, 8.0f, 0.0f, 0.0f, 3000.0f, 1000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.05f, 0.1f);

  pid_init_result[0] =
      pid_device_init(&shoot_pid_devices[0], &shoot_motors[0],
                      PID_DEVICE_POSITIONAL, &pid_speed[0], NULL);
  VERIFY(pid_init_result[0] == OK);
  pid_init_result[1] =
      pid_device_init(&shoot_pid_devices[1], &shoot_motors[1],
                      PID_DEVICE_POSITIONAL, &pid_speed[1], NULL);
  VERIFY(pid_init_result[1] == OK);
  pid_init_result[2] =
      pid_device_init(&shoot_pid_devices[2], &shoot_motors[2],
                      PID_DEVICE_POSITIONAL, &pid_speed[2], &pid_location);
  VERIFY(pid_init_result[2] == OK);
}

/**
 * @brief 安全停止发射机构所有电机
 * @details 将所有发射电机的命令清零并立即发送
 */
static void shoot_stop(void) {
  /* 发射机构所有电机（ID 1-3）都在 0x200 控制组，只需清空该组 */
  /* CAN_RegisterAllDevices 当前尚未注册发射机构，未初始化时保持安全空操作。 */
  if (!shoot_bus.initialized)
    return;
  VERIFY(dj_motor_zero_and_flush(&shoot_bus, DJ_MOTOR_GROUP_200) == OK);
}

/* 单发目标只在进入 ch_res 正向模式时锁存一次。 */
static float shoot_single_target_deg;      /* 本次单发的目标角度，单位：度。 */
static bool shoot_single_target_valid;     /* 目标角度是否已经成功锁存。 */
static bool shoot_single_done;             /* 本次单发是否已经到位并停止。 */
static uint8_t shoot_single_confirm_count; /* 正向 ch_res 连续确认的周期数。 */

static void shoot_single_reset(void) {
  shoot_single_target_valid = false;
  shoot_single_done = false;
  shoot_single_confirm_count = 0U;
}

static void shoot_single_mode(void) {
  ASSERT(shoot_bus.initialized);
  ASSERT(shoot_pid_devices[0].calculate != NULL);
  ASSERT(shoot_pid_devices[1].calculate != NULL);
  ASSERT(shoot_pid_devices[2].calculate != NULL);
  /* 单发模式下摩擦轮持续运行，直到退出该模式。 */
  VERIFY(pid_device_control_speed(&shoot_pid_devices[0], 2000.0f, 0.0f) == OK);
  VERIFY(pid_device_control_speed(&shoot_pid_devices[1], 2000.0f, 0.0f) == OK);

  if (shoot_single_done) {
    /* 拨弹已完成，只给拨弹电机发送零速，同时保持摩擦轮运行。 */
    VERIFY(pid_device_control_speed(&shoot_pid_devices[2], 0.0f, 0.0f) == OK);
    return;
  }

  if (shoot_single_confirm_count < 5) {
    ++shoot_single_confirm_count;
    /* 等待正向输入稳定期间，拨弹电机保持停止。 */
    VERIFY(pid_device_control_speed(&shoot_pid_devices[2], 0.0f, 0.0f) == OK);
    return;
  }

  dj_motor_feedback_t feedback;
  if (!shoot_single_target_valid) {
    const err_t feedback_result =
        dj_motor_get_feedback(&shoot_motors[2], &feedback);
    VERIFY(feedback_result == OK);
    if (feedback_result != OK || !feedback.valid) {
      VERIFY(pid_device_control_speed(&shoot_pid_devices[2], 0.0f, 0.0f) == OK);
      return;
    }
    ASSERT(isfinite(feedback.position_deg));
    shoot_single_target_deg = feedback.position_deg;
    circle_add(&shoot_single_target_deg, 30.0f, 360.0f);
    shoot_single_target_valid = true;
  }

  /* 摩擦轮已经持续运行，拨弹电机执行一次固定角度动作。 */
  const err_t location_result = pid_device_control_location(
      &shoot_pid_devices[2], shoot_single_target_deg, 0.0f, true);
  VERIFY((location_result == OK) || (location_result == STATE_ERR));
  if (location_result != OK) {
    VERIFY(pid_device_control_speed(&shoot_pid_devices[2], 0.0f, 0.0f) == OK);
    return;
  }

  const err_t feedback_result =
      dj_motor_get_feedback(&shoot_motors[2], &feedback);
  VERIFY(feedback_result == OK);
  if (feedback_result != OK || !feedback.valid) {
    VERIFY(pid_device_control_speed(&shoot_pid_devices[2], 0.0f, 0.0f) == OK);
    return;
  }

  ASSERT(isfinite(feedback.position_deg));
  const float angle_error =
      circle_error(shoot_single_target_deg, feedback.position_deg, 360.0f);
  ASSERT(isfinite(angle_error));
  if (angle_error < 3.0f && angle_error > -3.0f) {
    shoot_single_done = true;
  }
}

static void shoot_continuous_mode(void) {
  ASSERT(shoot_bus.initialized);
  ASSERT(shoot_pid_devices[0].calculate != NULL);
  ASSERT(shoot_pid_devices[1].calculate != NULL);
  ASSERT(shoot_pid_devices[2].calculate != NULL);
  shoot_single_reset();
  VERIFY(pid_device_control_speed(&shoot_pid_devices[0], 2000.0f, 0.0f) == OK);
  VERIFY(pid_device_control_speed(&shoot_pid_devices[1], 2000.0f, 0.0f) == OK);
  VERIFY(pid_device_control_speed(&shoot_pid_devices[2], 3000.0f, 0.0f) == OK);
}

void Shoot_Mode(void) {
  if ((dr16 == NULL) || !dr16->online_) {
    shoot_single_reset();
    shoot_stop();
    return;
  }

  if ((dr16->dr16_cmd.sw_l == CMD_SW_MID) &&
      (dr16->dr16_cmd.sw_r == CMD_SW_MID)) {
    const float ch_res = dr16->dr16_cmd.ch_res;
    ASSERT(isfinite(ch_res));

    if (ch_res > 0.5) {
      /* ch_res 正向保持 10 ms 后单发一次，拨弹完成后摩擦轮继续运行。 */
      shoot_single_mode();
    } else if (ch_res < -0.5) {
      /* ch_res 负向时连续拨弹，直到离开本模式或回到死区。 */
      shoot_continuous_mode();
    } else {
      /* 死区内停止，并允许下一次正向请求重新单发。 */
      shoot_single_reset();
      shoot_stop();
    }
  } else {
    /* 离开右拨杆中位或左拨杆中位后统一停止发射机构。 */
    shoot_single_reset();
    shoot_stop();
  }
}
