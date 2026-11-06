/**
 * @file chassis_control.c
 * @brief 麦克纳姆轮底盘控制实现
 */
#include "chassis_control.h"
#include "bsp_log.h"

#include <math.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "can.h"
#include "chassis_dynamics.h"
#include "comp_utils.h"
#include "dj_motor_ctrl.h"
#include "dr16.h"
#include "pid_location.h"

/* DR16 task owns the decoded command and updates it periodically. */
extern DR16_t* dr16;

/* 底盘电机总线与实例 */
dj_motor_bus_t chassis_bus;
dj_motor_t chassis_motors[4];

/* 速度环 PID 实例 */
static PIDInstance pid_speed[4];
static pid_device_t chassis_pid_devices[4];

static err_t chassis_bus_init_result;
static err_t chassis_init_result[4];

/* 底盘状态与目标速度 */
static chassis_control_state_t chassis_control_state_;
static float motor_target_speed[4];
static float torque_ff_current[4];

/**
 * @brief 初始化底盘 CAN 总线与四个 M3508 电机
 * @return OK 或错误码
 */
err_t chassis_control_init(void) {
  LOG_INFO("底盘计算", "开始初始化底盘 CAN 总线和电机");
  /* 获取 CAN2 的 BSP 对象（假设 CAN2 用于底盘） */
  ASSERT(CAN2 != NULL);
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN2);
  ASSERT(can_id != BSP_CAN_ID_ERROR);
  if (can_id == BSP_CAN_ID_ERROR) {
    LOG_ERROR("底盘计算", "未找到 CAN2 对象");
    return NOT_FOUND;
  }

  STM32CAN_t* can2 = STM32CAN_GetInstance(can_id);
  ASSERT(can2 != NULL);
  if (can2 == NULL) {
    LOG_ERROR("底盘计算", "CAN2 控制块为空");
    return PTR_NULL;
  }

  /* 初始化底盘总线 */
  chassis_bus_init_result = dj_motor_bus_init(&chassis_bus, can2);
  VERIFY(chassis_bus_init_result == OK);
  if (chassis_bus_init_result != OK) {
    LOG_ERROR("底盘计算", "底盘 CAN 总线初始化失败，错误码=%d",
              (int)chassis_bus_init_result);
    return chassis_bus_init_result;
  }

  /* 初始化四个 M3508 底盘电机（控制组 0x200）
   * 实车电机编号与轮位的对应关系：
   *   左前轮 FL = 4 号电机
   *   右前轮 FR = 3 号电机
   *   左后轮 RL = 2 号电机
   *   右后轮 RR = 1 号电机
   * reversed 参数根据实际机械安装方向设置 */
  chassis_init_result[0] =
      dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FL], &chassis_bus,
                    DJ_MOTOR_M3508, 4, false); /* 左前轮 = 4 号 */
  VERIFY(chassis_init_result[0] == OK);
  if (chassis_init_result[0] != OK) {
    LOG_ERROR("底盘计算", "左前电机初始化失败，错误码=%d",
              (int)chassis_init_result[0]);
    return chassis_init_result[0];
  }

  chassis_init_result[1] =
      dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FR], &chassis_bus,
                    DJ_MOTOR_M3508, 3, false); /* 右前轮 = 3 号 */
  VERIFY(chassis_init_result[1] == OK);
  if (chassis_init_result[1] != OK) {
    LOG_ERROR("底盘计算", "右前电机初始化失败，错误码=%d",
              (int)chassis_init_result[1]);
    return chassis_init_result[1];
  }

  chassis_init_result[2] =
      dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RL], &chassis_bus,
                    DJ_MOTOR_M3508, 2, false); /* 左后轮 = 2 号 */
  VERIFY(chassis_init_result[2] == OK);
  if (chassis_init_result[2] != OK) {
    LOG_ERROR("底盘计算", "左后电机初始化失败，错误码=%d",
              (int)chassis_init_result[2]);
    return chassis_init_result[2];
  }

  chassis_init_result[3] =
      dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RR], &chassis_bus,
                    DJ_MOTOR_M3508, 1, false); /* 右后轮 = 1 号 */
  VERIFY(chassis_init_result[3] == OK);
  if (chassis_init_result[3] != OK) {
    LOG_ERROR("底盘计算", "右后电机初始化失败，错误码=%d",
              (int)chassis_init_result[3]);
    return chassis_init_result[3];
  }

  LOG_INFO("底盘计算", "底盘 CAN 和四个电机初始化完成");
  return OK;
}

static void chassis_speed_pid_init_single(PIDInstance* pid, float kp, float ki,
                                          float kd, float max_out) {
  ASSERT(pid != NULL);
  ASSERT(isfinite(kp) && isfinite(ki) && isfinite(kd));
  ASSERT(max_out > 0.0f);
  PIDInit(pid, kp, ki, kd, max_out, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.0002f, 0.0002f);
}

void chassis_speed_pid_init(void) {
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FL], 12.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FR], 8.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RL], 8.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RR], 14.0f, 2.0f, 0.0f,
                                12000.0f);

  for (uint8_t i = 0; i < 4; ++i) {
    VERIFY(pid_device_init(&chassis_pid_devices[i], &chassis_motors[i],
                           PID_DEVICE_POSITIONAL, &pid_speed[i], NULL) == OK);
  }
  LOG_INFO("底盘计算", "四个底盘速度环 PID 初始化完成");
}

pid_device_t* chassis_get_pid_device(uint8_t motor_index) {
  return motor_index < 4 ? &chassis_pid_devices[motor_index] : NULL;
}

static void chassis_stop(void) {
  chassis_control_state_.command.vx = 0.0f;
  chassis_control_state_.command.vy = 0.0f;
  chassis_control_state_.command.wz = 0.0f;

  /* 使用 zero_and_flush 安全停机（推荐的停机方式） */
  VERIFY(dj_motor_zero_and_flush(&chassis_bus, DJ_MOTOR_GROUP_200) == OK);
}

static void chassis_control(void) {
  ASSERT(dr16 != NULL);
  const cmd_rc_t* command = &dr16->dr16_cmd;
  ASSERT(isfinite(command->ch.l.x) && isfinite(command->ch.l.y) &&
         isfinite(command->ch.r.x));

  chassis_dynamics_feedforward(torque_ff_current);
  /*
   * 统一底盘坐标系：
   *   +X：前
   *   +Y：左
   *   +Z：上
   *   +wz：顺时针
   *
   * DR16 左摇杆：l.x / l.y
   *   注意：这里保留当前遥控器通道的实际方向反号，
   *   只把它统一映射到机器人底盘坐标系。
   */
  chassis_control_state_.command.vx = -command->ch.l.y * 3000.0f;  // +X：前
  chassis_control_state_.command.vy = -command->ch.l.x * 3000.0f;  // +Y：左
  chassis_control_state_.command.wz = command->ch.r.x * 3000.0f;  // +wz：顺时针

  /*
   * 麦克纳姆轮逆运动学：
   *   底盘坐标系：
   *       +X：前
   *       +Y：左
   *       +Z：上
   *       +wz：顺时针
   *
   *             前（+X）
   *                ↑
   *        4号 FL     3号 FR
   *        左前       右前
   *
   *        2号 RL     1号 RR
   *        左后       右后
   *
   *   输出数组顺序固定为 [FL, FR, RL, RR] = [4, 3, 2, 1]。
   */
  /* 麦克纳姆轮运动学解算 */
  chassis_dynamics_inverse(
      chassis_control_state_.command.vx, chassis_control_state_.command.vy,
      chassis_control_state_.command.wz, motor_target_speed);

  /* 执行速度环 PID 控制（写齐后自动发送 CAN 帧） */
  for (uint8_t i = 0; i < 4; i++) {
    ASSERT(isfinite(motor_target_speed[i]) && isfinite(torque_ff_current[i]));
    VERIFY(pid_device_control_speed(&chassis_pid_devices[i],
                                    motor_target_speed[i],
                                    torque_ff_current[i]) == OK);
  }
}

/**
 * @brief 小陀螺模式
 *
 * 左拨杆 DOWN 时：
 *   - 左摇杆仍然有效：控制前后/左右平移
 *   - 右摇杆完全不参与底盘控制
 *   - 即使左摇杆回中，底盘仍保持固定 wz 原地旋转
 *
 * 因此：
 *   vx = 左摇杆前后
 *   vy = 左摇杆左右
 *   wz = 固定小陀螺速度
 */
static void chassis_small_gyro_control(void) {
  ASSERT(dr16 != NULL);
  const cmd_rc_t* command = &dr16->dr16_cmd;
  ASSERT(isfinite(command->ch.l.x) && isfinite(command->ch.l.y));

  chassis_dynamics_feedforward(torque_ff_current);

  /*
   * 左摇杆：
   *   l.y -> 前后移动 vx
   *   l.x -> 左右移动 vy
   *
   * 右摇杆：
   *   r.x 不读取，因此不会影响小陀螺旋转速度/方向。
   */
  chassis_control_state_.command.vx = -command->ch.l.y * 5000.0f;
  chassis_control_state_.command.vy = -command->ch.l.x * 5000.0f;

  /* 固定原地旋转速度：不动左摇杆时也会持续自转。 */
  chassis_control_state_.command.wz = 3000;

  chassis_dynamics_inverse(
      chassis_control_state_.command.vx, chassis_control_state_.command.vy,
      chassis_control_state_.command.wz, motor_target_speed);

  for (uint8_t i = 0; i < 4; i++) {
    ASSERT(isfinite(motor_target_speed[i]) && isfinite(torque_ff_current[i]));
    VERIFY(pid_device_control_speed(&chassis_pid_devices[i],
                                    motor_target_speed[i],
                                    torque_ff_current[i]) == OK);
  }
}

void Chassis_Mode(void) {
  static uint8_t last_mode = 0xFFU;
  uint8_t mode = 0U;
  /* 接收机掉线：底盘必须停止 */
  if ((dr16 == NULL) || !dr16->online_) {
    mode = 0U;
    if (mode != last_mode) {
      LOG_WARN("底盘计算", "遥控器离线，底盘进入停止模式");
      last_mode = mode;
    }
    chassis_stop();
    return;
  }

  if (dr16->dr16_cmd.sw_l == CMD_SW_MID) {
    mode = 1U;
    if (mode != last_mode) {
      LOG_INFO("底盘计算", "进入正常底盘控制模式");
      last_mode = mode;
    }
    chassis_control();

  } else if (dr16->dr16_cmd.sw_l == CMD_SW_DOWN) {
    mode = 2U;
    if (mode != last_mode) {
      LOG_INFO("底盘计算", "进入小陀螺模式");
      last_mode = mode;
    }
    chassis_small_gyro_control();
  } else {
    mode = 3U;
    if (mode != last_mode) {
      LOG_WARN("底盘计算", "档位异常或处于 UP，底盘停止");
      last_mode = mode;
    }
    /* UP 或异常状态：安全停止 */
    chassis_stop();
  }
}
