/**
 * @file shoot_control.c
 * @brief 发射机构控制实现 (DR16 遥控器)
 */
#include "shoot_control.h"
#include "bsp_log.h"

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

/* 发射机构控制参数（控制周期 2 ms） */
#define SHOOT_FRICTION_SPEED_RPM (2000.0f) // 摩擦轮稳定转速
#define SHOOT_FRICTION_READY_RPM (1600.0f) // 摩擦轮就绪判定阈值
#define SHOOT_FRICTION_DROP_RPM (1000.0f)  // 摩擦轮掉速判定阈值
#define SHOOT_FRICTION_READY_CYCLES (10U) /* 20 ms 稳定达到目标转速后允许供弹 \
                                           */

#define SHOOT_CH_RES_THRESHOLD (0.5f) // 遥控器拨杆正向阈值，超过该值切换状态
#define SHOOT_SINGLE_CONFIRM_CYCLES (5U)     /* 10 ms 防抖确认 */
#define SHOOT_SINGLE_OUTPUT_STEP_DEG (45.0f) /* 8 孔拨弹盘单发输出轴角度（°） \
                                              */
#define SHOOT_SINGLE_OUTPUT_TOL_DEG                                            \
  (0.5f)                                /* 输出轴到位容差（°），对应转子 18° */
#define SHOOT_SINGLE_STOP_RPM (150.0f)  /* 判稳转速阈值（转子 RPM） */
#define SHOOT_SINGLE_SETTLE_CYCLES (5U) /* 到位判稳持续 10 ms */
#define SHOOT_SINGLE_TIMEOUT_CYCLES (500U) /* 1000 ms 单发超时堵转保护 */
#define SHOOT_CONTINUOUS_FEED_RPM                                              \
  (3000.0f) // 摩擦轮连续供弹转速阈值（RPM），低于该值停止供弹

/* 摩擦轮就绪状态 */
static bool shoot_friction_ready;
static uint8_t shoot_friction_ready_count;

/* 单发目标只在进入 ch_res 正向模式且摩擦轮就绪时锁存一次。 */
static float shoot_single_target_deg;  /* 本次单发的输出轴相对目标角度（°） */
static bool shoot_single_target_valid; /* 目标角度是否已经成功锁存 */
static bool shoot_single_done;         /* 本次单发是否已经到位或超时停止 */
static uint8_t shoot_single_confirm_count;   /* 正向 ch_res 连续确认的周期数 */
static uint8_t shoot_single_settle_count;    /* 到位判稳连续确认周期数 */
static uint16_t shoot_single_elapsed_cycles; /* 本次单发已运行的控制周期数 */
static bool shoot_stop_pid_reset_done; /* 停机后是否已复位运行期 PID 状态 */

/**
 * @brief 初始化发射 CAN 总线与三台发射电机
 * @return OK 或错误码
 */
err_t shoot_control_init(void) {
  LOG_DEBUG("发射计算", "开始初始化发射 CAN 总线和电机");
  /* 获取 CAN1 的 BSP 对象（假设 CAN1 用于发射机构，与云台同总线）
   * CAN2 上底盘已占用 0x200 控制组的 1-4 号电机，发射机构不能复用同组 ID */
  ASSERT(CAN1 != NULL);
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN1);
  ASSERT(can_id != BSP_CAN_ID_ERROR);
  if (can_id == BSP_CAN_ID_ERROR) {
    LOG_ERROR("发射计算", "未找到 CAN1 对象");
    return NOT_FOUND;
  }

  STM32CAN_t *can1 = STM32CAN_GetInstance(can_id);
  ASSERT(can1 != NULL);
  if (can1 == NULL) {
    LOG_ERROR("发射计算", "CAN1 控制块为空");
    return PTR_NULL;
  }

  /* 初始化发射机构总线 */
  shoot_bus_init_result = dj_motor_bus_init(&shoot_bus, can1);
  if (shoot_bus_init_result != OK) {
    LOG_ERROR("发射计算", "发射 CAN 总线初始化失败，错误码=%d",
              (int)shoot_bus_init_result);
    VERIFY(shoot_bus_init_result == OK);
    return shoot_bus_init_result;
  }

  /* 初始化发射机构电机：
   *   左摩擦轮 = 7 号电机（M3508，0x1FF 组，单速度环）
   *   右摩擦轮 = 8 号电机（M3508，0x1FF 组，单速度环）
   *   拨弹电机 = 3 号电机（M2006，0x200 组，速度位置双环）
   * reversed 参数根据实际机械安装方向设置 */
  shoot_init_result[0] =
      dj_motor_init(&shoot_motors[0], &shoot_bus, DJ_MOTOR_M3508, 7,
                    false); /* 左摩擦轮 = 7 号 */
  if (shoot_init_result[0] != OK) {
    LOG_ERROR("发射计算", "左摩擦轮初始化失败，错误码=%d",
              (int)shoot_init_result[0]);
    VERIFY(shoot_init_result[0] == OK);
    return shoot_init_result[0];
  }

  shoot_init_result[1] =
      dj_motor_init(&shoot_motors[1], &shoot_bus, DJ_MOTOR_M3508, 8,
                    false); /* 右摩擦轮 = 8 号 */
  if (shoot_init_result[1] != OK) {
    LOG_ERROR("发射计算", "右摩擦轮初始化失败，错误码=%d",
              (int)shoot_init_result[1]);
    VERIFY(shoot_init_result[1] == OK);
    return shoot_init_result[1];
  }

  shoot_init_result[2] =
      dj_motor_init(&shoot_motors[2], &shoot_bus, DJ_MOTOR_M2006, 3,
                    false); /* 拨弹电机 = 3 号 */
  if (shoot_init_result[2] != OK) {
    LOG_ERROR("发射计算", "拨弹电机初始化失败，错误码=%d",
              (int)shoot_init_result[2]);
    VERIFY(shoot_init_result[2] == OK);
    return shoot_init_result[2];
  }

  LOG_DEBUG("发射计算", "发射 CAN 和三台电机初始化完成");
  return OK;
}

void shoot_speed_pid_init(void) {
  // 初始化左摩擦轮电机速度 PID 参数（M3508 单速度环）
  PIDInit(&pid_speed[0], 10.0f, 0.0f, 0.0f, 12000.0f, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.0015f, 0.0015f);
  // 初始化右摩擦轮电机速度 PID 参数（M3508 单速度环）
  PIDInit(&pid_speed[1], 10.0f, 0.0f, 0.0f, 12000.0f, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.0015f, 0.0015f);
  // 初始化拨弹电机速度环 PID 参数（M2006 双环内环，输出电流限幅 10000）
  PIDInit(&pid_speed[2], 8.0f, 0.0f, 0.0f, 10000.0f, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.0015f, 0.0015f);
  // 初始化拨弹电机位置环 PID（仅 M2006 需要位置环，输出为目标速度）
  PIDInit(&pid_location, 8.0f, 0.0f, 0.0f, 3000.0f, 1000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.0015f, 0.0015f);

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
  LOG_DEBUG("发射计算", "发射机构速度环和位置环 PID 初始化完成");
}

/**
 * @brief 复位单发状态机，允许下一次正向请求重新触发单发
 */
static void shoot_single_reset(void) {
  shoot_single_target_valid = false;
  shoot_single_done = false;
  shoot_single_confirm_count = 0U;
  shoot_single_settle_count = 0U;
  shoot_single_elapsed_cycles = 0U;
}

/**
 * @brief 复位摩擦轮就绪状态
 */
static void shoot_runtime_reset(void) {
  shoot_friction_ready = false;
  shoot_friction_ready_count = 0U;
}

/**
 * @brief 拨弹电机速度闭环控制（发送失败时仅记录状态，不阻塞任务重试）
 */
static void shoot_feeder_speed(float target_rpm) {
  (void)pid_device_control_speed(&shoot_pid_devices[2], target_rpm, 0.0f);
}

/**
 * @brief 仅停止拨弹电机（保持摩擦轮继续运行）
 */
static void shoot_feeder_stop(void) { shoot_feeder_speed(0.0f); }

/**
 * @brief 安全停止发射机构所有电机（同时清零 0x1FF 摩擦轮组与 0x200 拨弹组）
 */
static void shoot_stop(void) {
  static err_t last_stop_err_1ff = OK;
  static err_t last_stop_err_200 = OK;

  shoot_single_reset();
  shoot_runtime_reset();

  if (!shoot_stop_pid_reset_done) {
    PIDReset(&pid_speed[0]);
    PIDReset(&pid_speed[1]);
    PIDReset(&pid_speed[2]);
    PIDReset(&pid_location);
    shoot_stop_pid_reset_done = true;
  }

  if (!shoot_bus.initialized) {
    return;
  }

  const err_t err_1ff = dj_motor_zero_and_flush(&shoot_bus, DJ_MOTOR_GROUP_1FF);
  if (err_1ff != last_stop_err_1ff) {
    if (err_1ff != OK) {
      LOG_ERROR("发射计算", "CAN1 摩擦轮停机帧发送失败 group=0x1FF err=%d",
                (int)err_1ff);
    }
    last_stop_err_1ff = err_1ff;
  }

  const err_t err_200 = dj_motor_zero_and_flush(&shoot_bus, DJ_MOTOR_GROUP_200);
  if (err_200 != last_stop_err_200) {
    if (err_200 != OK) {
      LOG_ERROR("发射计算", "CAN1 拨弹停机帧发送失败 group=0x200 err=%d",
                (int)err_200);
    }
    last_stop_err_200 = err_200;
  }
}

/**
 * @brief 驱动双摩擦轮旋转，并基于实际转速反馈判稳就绪状态
 */
static void shoot_friction_control(void) {
  ASSERT(shoot_bus.initialized);
  ASSERT(shoot_pid_devices[0].calculate != NULL);
  ASSERT(shoot_pid_devices[1].calculate != NULL);

  (void)pid_device_control_speed(&shoot_pid_devices[0],
                                 SHOOT_FRICTION_SPEED_RPM, 0.0f);
  (void)pid_device_control_speed(&shoot_pid_devices[1],
                                 SHOOT_FRICTION_SPEED_RPM, 0.0f);

  dj_motor_feedback_t fb0;
  dj_motor_feedback_t fb1;
  const err_t res0 = dj_motor_get_feedback(&shoot_motors[0], &fb0);
  const err_t res1 = dj_motor_get_feedback(&shoot_motors[1], &fb1);
  if ((res0 != OK) || (res1 != OK) || !fb0.valid || !fb1.valid) {
    shoot_friction_ready = false;
    shoot_friction_ready_count = 0U;
    return;
  }

  const float abs_rpm0 = fabsf((float)fb0.speed_rpm);
  const float abs_rpm1 = fabsf((float)fb1.speed_rpm);
  if (!shoot_friction_ready) {
    if ((abs_rpm0 >= SHOOT_FRICTION_READY_RPM) &&
        (abs_rpm1 >= SHOOT_FRICTION_READY_RPM)) {
      ++shoot_friction_ready_count;
      if (shoot_friction_ready_count >= SHOOT_FRICTION_READY_CYCLES) {
        shoot_friction_ready = true;
        LOG_INFO("发射计算", "摩擦轮已就绪，允许单发/连发供弹");
      }
    } else {
      shoot_friction_ready_count = 0U;
    }
  } else if ((abs_rpm0 < SHOOT_FRICTION_DROP_RPM) ||
             (abs_rpm1 < SHOOT_FRICTION_DROP_RPM)) {
    shoot_friction_ready = false;
    shoot_friction_ready_count = 0U;
    LOG_WARN("发射计算", "摩擦轮转速过低或堵转，暂停供弹等待恢复");
  }
}

/**
 * @brief 单发拨弹控制（前提：摩擦轮已在 shoot_friction_control 中持续旋转）
 */
static void shoot_single_mode(void) {
  ASSERT(shoot_bus.initialized);
  ASSERT(shoot_pid_devices[2].calculate != NULL);

  if (!shoot_friction_ready) {
    shoot_single_confirm_count = 0U;
    shoot_feeder_stop();
    return;
  }

  if (shoot_single_done) {
    /* 本次单发已完成，拨弹电机保持零速，等待指令回中后再允许下一次单发。 */
    shoot_feeder_stop();
    return;
  }

  if (shoot_single_confirm_count < SHOOT_SINGLE_CONFIRM_CYCLES) {
    ++shoot_single_confirm_count;
    /* 等待正向输入稳定期间，拨弹电机保持停止。 */
    shoot_feeder_stop();
    return;
  }

  if (!shoot_single_target_valid) {
    if (pid_device_reset_multi_turn(&shoot_pid_devices[2]) != OK) {
      shoot_feeder_stop();
      return;
    }
    shoot_single_target_deg = SHOOT_SINGLE_OUTPUT_STEP_DEG;
    shoot_single_settle_count = 0U;
    shoot_single_elapsed_cycles = 0U;
    PIDReset(&pid_location);
    shoot_single_target_valid = true;
#if BSP_LOG_ENABLED
    char target_text[32];
#endif
    LOG_DEBUG("发射计算", "单发输出轴目标角度已锁存：%s 度",
              Float2StrN(target_text, sizeof(target_text),
                         shoot_single_target_deg) >= 0
                  ? target_text
                  : "<invalid>");
  }

  /* 公共层基于 dj_motor 中断维护的多圈编码器刻度执行输出轴角度闭环。 */
  const err_t loc_result = pid_device_control_multi_location(
      &shoot_pid_devices[2], shoot_single_target_deg, 0.0f, true);
  if (loc_result == STATE_ERR) {
    shoot_feeder_stop();
    return;
  }

  float current_output_deg = 0.0f;
  dj_motor_feedback_t feedback;
  if ((pid_device_get_multi_turn_deg(&shoot_pid_devices[2], &current_output_deg,
                                     true) != OK) ||
      (dj_motor_get_feedback(&shoot_motors[2], &feedback) != OK) ||
      !feedback.valid) {
    return;
  }

  const float angle_error = shoot_single_target_deg - current_output_deg;
  ASSERT(isfinite(angle_error));
  if ((fabsf(angle_error) <= SHOOT_SINGLE_OUTPUT_TOL_DEG) &&
      (fabsf((float)feedback.speed_rpm) <= SHOOT_SINGLE_STOP_RPM)) {
    ++shoot_single_settle_count;
    if (shoot_single_settle_count >= SHOOT_SINGLE_SETTLE_CYCLES) {
      shoot_single_done = true;
#if BSP_LOG_ENABLED
      char error_text[32];
#endif
      LOG_DEBUG("发射计算", "单发拨弹到位并停稳，输出轴误差=%s 度",
                Float2StrN(error_text, sizeof(error_text), angle_error) >= 0
                    ? error_text
                    : "<invalid>");
    }
  } else {
    shoot_single_settle_count = 0U;
    ++shoot_single_elapsed_cycles;
    if (shoot_single_elapsed_cycles >= SHOOT_SINGLE_TIMEOUT_CYCLES) {
      shoot_single_done = true;
#if BSP_LOG_ENABLED
      char error_text[32];
#endif
      LOG_WARN("发射计算", "单发拨弹超时保护停止，输出轴剩余误差=%s 度",
               Float2StrN(error_text, sizeof(error_text), angle_error) >= 0
                   ? error_text
                   : "<invalid>");
    }
  }
}

/**
 * @brief 连发拨弹控制（前提：摩擦轮已在 shoot_friction_control 中持续旋转）
 */
static void shoot_continuous_mode(void) {
  ASSERT(shoot_bus.initialized);
  ASSERT(shoot_pid_devices[2].calculate != NULL);

  shoot_single_reset();
  if (!shoot_friction_ready) {
    shoot_feeder_stop();
    return;
  }
  shoot_feeder_speed(SHOOT_CONTINUOUS_FEED_RPM);
}

void Shoot_Mode(void) {
  static uint8_t last_mode = 0xFFU;
  uint8_t mode = 0U;
  if ((dr16 == NULL) || !dr16->online_) {
    mode = 0U;
    if (mode != last_mode) {
      LOG_WARN("发射计算", "遥控器离线，发射机构停止");
      last_mode = mode;
    }
    shoot_stop();
    return;
  }

  if (dr16->dr16_cmd.sw_r == CMD_SW_MID) {
    shoot_stop_pid_reset_done = false;
    /* 只要右拨杆在中位，先进入摩擦轮旋转待命状态，再由 ch_res 决定单发/连发。
     */
    shoot_friction_control();

    const float ch_res = dr16->dr16_cmd.ch_res;
    if (!isfinite(ch_res)) {
      mode = 5U;
      if (mode != last_mode) {
        LOG_ERROR("发射计算", "发射通道指令非法，拨弹电机保持停止");
        last_mode = mode;
      }
      shoot_single_reset();
      shoot_feeder_stop();
      return;
    }

    if (ch_res > SHOOT_CH_RES_THRESHOLD) {
      mode = 1U;
      if (mode != last_mode) {
        LOG_INFO("发射计算", "进入单发模式");
        last_mode = mode;
      }
      /* 摩擦轮就绪且 ch_res 正向保持 10 ms 后单发一次，到位后拨弹停止。 */
      shoot_single_mode();
    } else if (ch_res < -SHOOT_CH_RES_THRESHOLD) {
      mode = 2U;
      if (mode != last_mode) {
        LOG_INFO("发射计算", "进入连续发射模式");
        last_mode = mode;
      }
      /* 摩擦轮就绪后连续拨弹，直到离开本模式或回到死区。 */
      shoot_continuous_mode();
    } else {
      mode = 3U;
      if (mode != last_mode) {
        LOG_INFO("发射计算", "摩擦轮旋转待命，拨弹指令处于死区");
        last_mode = mode;
      }
      /* 死区内仅停止拨弹并允许下一次正向请求重新单发，摩擦轮继续保持旋转。 */
      shoot_single_reset();
      shoot_feeder_stop();
    }
  } else {
    mode = 4U;
    if (mode != last_mode) {
      LOG_INFO("发射计算", "右拨杆离开中位，发射机构停止");
      last_mode = mode;
    }
    /* 离开右拨杆中位后统一停止摩擦轮和拨弹电机。 */
    shoot_stop();
  }
}
