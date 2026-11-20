#include "pid_device.h"

#include <math.h>
#include <stddef.h>
#include <stdatomic.h>

#include "bsp_log.h"
#include "comp_utils.h"
#include "pid_incremental.h"
#include "pid_location.h"

/** 使用位置式 PID 计算一次控制输出。 */
static float pid_device_calculate_positional(void* pid, float target,
                                             float actual) {
  return PIDCalculate((PIDInstance*)pid, target, actual);
}

/** 使用增量式 PID 计算一次控制输出。 */
static float pid_device_calculate_incremental(void* pid, float target,
                                              float actual) {
  return PID_Calculate_jie((PIDInstance_jie*)pid, target, actual);
}

/* 每类 CAN 故障成功报告后才去重，发送失败时保留后续报告机会。 */
#if BSP_LOG_ENABLED
static _Atomic uint32_t pid_device_can_fault_reported;
#endif

/**
 * @brief 将电机控制失败转换为可直接定位硬件的中文日志。
 * @details 日志从 PID 控制层输出，因此所有底盘、云台和发射控制路径都能
 *          在 VERIFY 触发前留下 CAN 编号、电机 ID 和错误码。
 */
static void pid_device_log_failure(const pid_device_t* device,
                                   const char* operation, bool transmission,
                                   err_t result, float target_rpm,
                                   float feedforward) {
#if BSP_LOG_ENABLED
  if ((device == NULL) || (device->motor == NULL) ||
      (device->motor->bus == NULL) || (device->motor->bus->can == NULL)) {
    return;
  }

  const unsigned can_number = (unsigned)device->motor->bus->can->id_ + 1U;
  const unsigned motor_id = (unsigned)device->motor->device_id;
  const unsigned fault_bit = (can_number - 1U) * 2U + (transmission ? 1U : 0U);
  if ((fault_bit < 32U) &&
      ((atomic_load_explicit(&pid_device_can_fault_reported, memory_order_relaxed) &
        (1UL << fault_bit)) != 0UL)) {
    return;
  }

  char target_text[32];
  char feedforward_text[32];
  const err_t log_result = LOG_FATAL("电机控制",
            "CAN%u %s失败 motor=%u err=%d rpm=%s ff=%s",
            can_number, operation, motor_id, (int)result,
            Float2StrN(target_text, sizeof(target_text), target_rpm) >= 0
                ? target_text : "<invalid>",
            Float2StrN(feedforward_text, sizeof(feedforward_text), feedforward) >= 0
                ? feedforward_text : "<invalid>");
  if (log_result == OK && fault_bit < 32U) {
    atomic_fetch_or_explicit(&pid_device_can_fault_reported, 1UL << fault_bit,
                             memory_order_relaxed);
  }
#else
  RM_UNUSED(device);
  RM_UNUSED(operation);
  RM_UNUSED(result);
  RM_UNUSED(target_rpm);
  RM_UNUSED(feedforward);
#endif
}

/** 绑定电机和速度/位置 PID 实例，选择底层算法适配器。 */
err_t pid_device_init(pid_device_t* device, dj_motor_t* motor,
                      pid_device_algorithm_e algorithm, void* speed_pid,
                      void* location_pid) {
  if ((device == NULL) || (motor == NULL) || (speed_pid == NULL)) {
    return PTR_NULL;
  }

  float (*calculate)(void* pid, float target, float actual);
  switch (algorithm) {
    case PID_DEVICE_POSITIONAL:
      calculate = pid_device_calculate_positional;
      break;
    case PID_DEVICE_INCREMENTAL:
      calculate = pid_device_calculate_incremental;
      break;
    default:
      return ARG_ERR;
  }

  device->motor = motor;
  device->speed_pid = speed_pid;
  device->location_pid = location_pid;
  device->calculate = calculate;
  device->multi_turn_zero_valid = false;
  device->multi_turn_zero_ticks = 0;
  return OK;
}

/** 检查设备绑定状态并读取一份电机反馈快照。 */
static err_t pid_device_feedback(pid_device_t* device,
                                 dj_motor_feedback_t* feedback) {
  if (device == NULL) return PTR_NULL;
  if ((device->motor == NULL) || (device->speed_pid == NULL) ||
      (device->calculate == NULL)) {
    return STATE_ERR;
  }
  return dj_motor_get_feedback(device->motor, feedback);
}

/** 叠加前馈、按电机型号限幅，并写入电机命令缓存。 */
static err_t pid_device_write_command(pid_device_t* device, float output,
                                      float feedforward) {
  const float limit = (float)dj_motor_drv_command_limit(device->motor->type);
  float command = output + feedforward;
  if (!isfinite(command) || (limit <= 0.0f)) return ARG_ERR;

  /* 浮点限幅必须先于 int16_t 转换，避免前馈叠加后溢出。 */
  clampf(&command, -limit, limit);
  return dj_motor_set_command(device->motor, (int16_t)command);
}

/** 执行转子 RPM 速度环控制，并写入最终电机命令。 */
err_t pid_device_control_speed(pid_device_t* device, float target_rpm,
                               float feedforward) {
  if (!isfinite(target_rpm) || !isfinite(feedforward)) {
    LOG_ERROR("电机控制", "速度控制参数非法：目标转速或前馈不是有限值");
    return ARG_ERR;
  }

  dj_motor_feedback_t feedback;
  const err_t result = pid_device_feedback(device, &feedback);
  if (result != OK) {
    pid_device_log_failure(device, "读取反馈", false, result, target_rpm,
                           feedforward);
    return result;
  }

  const float output = device->calculate(device->speed_pid, target_rpm,
                                         (float)feedback.speed_rpm);
  const err_t write_result =
      pid_device_write_command(device, output, feedforward);
  if (write_result != OK) {
    pid_device_log_failure(device, "发送命令", true, write_result, target_rpm,
                           feedforward);
  }
  return write_result;
}
dj_motor_feedback_t feedback_jie = {};
/**
 * 执行位置串级控制：位置环生成目标 RPM，速度环生成电机命令。
 * is_optimize 为 true 时使用 ±180° 最短路径，否则直接使用角度反馈。
 */
err_t pid_device_control_location(pid_device_t* device, float target_deg,
                                  float feedforward, bool is_optimize) {
  if (!isfinite(target_deg) || !isfinite(feedforward)) return ARG_ERR;

  dj_motor_feedback_t feedback;
  const err_t result = pid_device_feedback(device, &feedback);
  if (result != OK) return result;

  /* 先由驱动填充完整快照，再复制给调试观察变量。此前这里复制的是
   * 未初始化的局部变量，导致温度、角度、valid 和 tick 都可能是随机值。 */
  feedback_jie = feedback;

  if ((device->location_pid == NULL) || !feedback.valid) return STATE_ERR;
  if (!isfinite(feedback.position_deg)) return ARG_ERR;

  float target_rpm;
  if (is_optimize) {
    const float error = circle_error(target_deg, feedback.position_deg, 360.0f);
    target_rpm = device->calculate(device->location_pid, error, 0.0f);
  } else {
    target_rpm = device->calculate(device->location_pid, target_deg,
                                   feedback.position_deg);
  }

  if (!isfinite(target_rpm)) return ARG_ERR;
  const float output = device->calculate(device->speed_pid, target_rpm,
                                         (float)feedback.speed_rpm);
  return pid_device_write_command(device, output, feedforward);
}

/** 计算相对多圈零点的转子累计角度（°）；首次遇到有效反馈时自动建立零点。 */
static float pid_device_relative_rotor_deg(pid_device_t* device,
                                           const dj_motor_feedback_t* feedback) {
  if (!device->multi_turn_zero_valid) {
    device->multi_turn_zero_ticks = feedback->total_encoder;
    device->multi_turn_zero_valid = true;
  }
  return (float)(feedback->total_encoder - device->multi_turn_zero_ticks) *
         (360.0f / (float)DJ_MOTOR_ENCODER_RESOLUTION);
}

err_t pid_device_reset_multi_turn(pid_device_t* device) {
  dj_motor_feedback_t feedback;
  const err_t result = pid_device_feedback(device, &feedback);
  if (result != OK) {
    return result;
  }
  if (!feedback.valid) {
    device->multi_turn_zero_valid = false;
    device->multi_turn_zero_ticks = 0;
    return STATE_ERR;
  }

  device->multi_turn_zero_ticks = feedback.total_encoder;
  device->multi_turn_zero_valid = true;
  return OK;
}

err_t pid_device_get_multi_turn_deg(pid_device_t* device, float* angle_deg,
                                    bool use_output_shaft) {
  if (angle_deg == NULL) {
    return PTR_NULL;
  }

  dj_motor_feedback_t feedback;
  const err_t result = pid_device_feedback(device, &feedback);
  if (result != OK) {
    return result;
  }
  if (!feedback.valid) {
    return STATE_ERR;
  }

  float rotor_deg = pid_device_relative_rotor_deg(device, &feedback);
  if (use_output_shaft) {
    const float reduction = dj_motor_drv_reduction_ratio(device->motor->type);
    if (reduction > 0.0f) {
      rotor_deg /= reduction;
    }
  }
  *angle_deg = rotor_deg;
  return OK;
}

err_t pid_device_control_multi_location(pid_device_t* device, float target_deg,
                                        float feedforward,
                                        bool use_output_shaft) {
  if (!isfinite(target_deg) || !isfinite(feedforward)) {
    return ARG_ERR;
  }

  dj_motor_feedback_t feedback;
  const err_t result = pid_device_feedback(device, &feedback);
  if (result != OK) {
    return result;
  }
  if ((device->location_pid == NULL) || !feedback.valid) {
    return STATE_ERR;
  }

  const float actual_rotor_deg = pid_device_relative_rotor_deg(device, &feedback);
  float target_rotor_deg = target_deg;
  if (use_output_shaft) {
    const float reduction = dj_motor_drv_reduction_ratio(device->motor->type);
    if (reduction > 0.0f) {
      target_rotor_deg *= reduction;
    }
  }
  if (!isfinite(target_rotor_deg)) {
    return ARG_ERR;
  }

  const float target_rpm =
      device->calculate(device->location_pid, target_rotor_deg, actual_rotor_deg);
  if (!isfinite(target_rpm)) {
    return ARG_ERR;
  }

  const float output =
      device->calculate(device->speed_pid, target_rpm, (float)feedback.speed_rpm);
  return pid_device_write_command(device, output, feedforward);
}
