#include "pid_device.h"

#include <math.h>
#include <stddef.h>

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
  if (!isfinite(target_rpm) || !isfinite(feedforward)) return ARG_ERR;

  dj_motor_feedback_t feedback;
  const err_t result = pid_device_feedback(device, &feedback);
  if (result != OK) return result;

  const float output = device->calculate(device->speed_pid, target_rpm,
                                         (float)feedback.speed_rpm);
  return pid_device_write_command(device, output, feedforward);
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
