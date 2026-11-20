#pragma once

#include "dj_motor_ctrl.h"

/** PID 算法与传入实例类型必须匹配。 */
typedef enum {
  PID_DEVICE_POSITIONAL = 0, /**< PIDInstance / PIDInit */
  PID_DEVICE_INCREMENTAL     /**< PIDInstance_jie / PID_Init_Params_jie */
} pid_device_algorithm_e;

/** 调用方持有电机及 PID 实例，本对象只绑定它们，不分配内存。
 *  两种算法的适配和控制实现均位于 pid_device.c。 */
typedef struct {
  dj_motor_t* motor;
  void* speed_pid;
  void* location_pid; /**< NULL 表示仅支持速度环 */
  float (*calculate)(void* pid, float target, float actual);
  bool multi_turn_zero_valid;    /**< true 表示已设置多圈相对零点 */
  int64_t multi_turn_zero_ticks; /**< 相对零点对应的逻辑累计编码器刻度 */
} pid_device_t;

/**
 * @brief 绑定已初始化的 PID 实例，保留其参数和运行状态。
 * @param algorithm 两个环共用的算法；指针类型见 pid_device_algorithm_e。
 * @param location_pid 可为 NULL；速度环实例不可为空。
 * @note 绑定不要求电机已注册，但控制前必须完成 dj_motor_init。
 *       实例生命周期须覆盖本对象；同一 PID 实例不要绑定多台电机。
 * @return OK / PTR_NULL / ARG_ERR；失败时不修改 device。
 */
err_t pid_device_init(pid_device_t* device, dj_motor_t* motor,
                      pid_device_algorithm_e algorithm, void* speed_pid,
                      void* location_pid);

/**
 * @brief 转子 RPM 速度环，前馈单位为电调原始指令（不参与 PID 内部限幅）。
 * @details PID 输出叠加 feedforward 后按电机型号限幅，交给驱动写齐再发。
 *          沿用底盘速度环语义：get_feedback 成功即可计算，valid 不作为门限。
 * @return OK 或驱动错误；空指针 PTR_NULL，未绑定 STATE_ERR，非有限输入
 * ARG_ERR。
 */
err_t pid_device_control_speed(pid_device_t* device, float target_rpm,
                               float feedforward);

/**
 * @brief 单圈位置串级环：目标为转子编码器角度（度）。
 * @param is_optimize true：归一化目标并以 ±180 度内的最短路径误差作为位置环
 *        目标、反馈传 0；false：直接把 target_deg 和 position_deg 传入位置环，
 *        不归一化目标或误差。
 * @details 位置环输出为目标转子 RPM，速度环输出为电调原始指令。
 *          不累计圈数、不换算减速器输出轴角度；要求 feedback.valid。
 * @return 同速度环；未绑定位置环或无有效位置反馈返回 STATE_ERR，不写命令。
 */
err_t pid_device_control_location(pid_device_t* device, float target_deg,
                                  float feedforward, bool is_optimize);

/**
 * @brief 将当前多圈累计位置设为相对零点（0°）。
 * @param device 已初始化的设备对象。
 * @return OK 成功；空指针返回 PTR_NULL；未绑定或尚无有效反馈返回 STATE_ERR。
 */
err_t pid_device_reset_multi_turn(pid_device_t* device);

/**
 * @brief 读取相对多圈零点的累计角度（单位：度）。
 * @param device 已初始化的设备对象。
 * @param angle_deg 输出相对角度，不可为 NULL。
 * @param use_output_shaft true 返回输出轴角度（已除电机减速比）；false 返回转子角度。
 * @return OK 成功；空指针 PTR_NULL；未绑定或无有效反馈 STATE_ERR。
 * @note 若尚未设置零点且反馈有效，首次调用会自动把当前位置设为零点（返回 0°）。
 */
err_t pid_device_get_multi_turn_deg(pid_device_t* device, float* angle_deg,
                                    bool use_output_shaft);

/**
 * @brief 多圈位置串级环：基于底层连续多圈编码器刻度做相对位置闭环。
 * @param device 已初始化且绑定了位置环的设备对象。
 * @param target_deg 相对多圈零点的目标角度（度）。
 * @param feedforward 速度环输出叠加的电调原始前馈指令。
 * @param use_output_shaft true 表示 target_deg 为输出轴角度（内部乘减速比后传入位置环，
 *        保持位置环输入单位为转子度、输出单位为转子 RPM）；false 表示 target_deg 为转子角度。
 * @return 同 pid_device_control_location。
 */
err_t pid_device_control_multi_location(pid_device_t* device, float target_deg,
                                        float feedforward,
                                        bool use_output_shaft);
