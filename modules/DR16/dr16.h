#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_uart.h"
#include "comp_cmd.h"
#include "comp_def.h"
#include "task.h"

// ==================== DR16 协议常量 ====================

#define DR16_FRAME_SIZE (18u)          //! DR16 SBUS 一帧固定 18 字节
#define DR16_OFFLINE_TIMEOUT_MS (20)   //! 超过该时间未收到新帧则认为遥控器离线

#define DR16_CH_VALUE_MIN (364u)       //! 摇杆原始值最小值
#define DR16_CH_VALUE_MID (1024u)      //! 摇杆原始值中值
#define DR16_CH_VALUE_MAX (1684u)      //! 摇杆原始值最大值

#ifdef __cplusplus
extern "C" {
#endif

//! DR16 原始帧位域映射。
//! 必须 packed，字段顺序与遥控器 18 字节协议严格对应。
typedef struct __attribute__((packed)) {
  uint16_t ch_r_x : 11;  //! 右摇杆 X 原始通道
  uint16_t ch_r_y : 11;  //! 右摇杆 Y 原始通道
  uint16_t ch_l_x : 11;  //! 左摇杆 X 原始通道
  uint16_t ch_l_y : 11;  //! 左摇杆 Y 原始通道
  uint8_t sw_r : 2;      //! 右拨杆位置
  uint8_t sw_l : 2;      //! 左拨杆位置
  int16_t x;             //! 鼠标 X 增量
  int16_t y;             //! 鼠标 Y 增量
  int16_t z;             //! 鼠标滚轮增量
  uint8_t press_l;       //! 鼠标左键状态
  uint8_t press_r;       //! 鼠标右键状态
  uint16_t key;          //! 键盘按键位图
  uint16_t res;          //! 保留通道
} dr16_data_t;

#if defined(__cplusplus)
static_assert(sizeof(dr16_data_t) == DR16_FRAME_SIZE,
              "DR16 raw layout must remain 18 bytes");
#else
_Static_assert(sizeof(dr16_data_t) == DR16_FRAME_SIZE,
               "DR16 raw layout must remain 18 bytes");
#endif

//! DR16 遥控器对象。
//! RX 使用 UART DMA 接收完整 18 字节帧，任务上下文解析为归一化 cmd_rc_t。
typedef struct {
  TaskHandle_t thread_alert;     //! 接收事件通知的 FreeRTOS 任务句柄
  BaseType_t switch_required;    //! 预留状态切换标志
  cmd_rc_t dr16_cmd;             //! 解析后的遥控器命令
  bool online_;                  //! 最近周期是否收到并解析到合法帧
  STM32UART_t uart_;             //! UART DMA 接收通道
  err_t init_error_;             //! BSP 初始化结果，Start 前保留
  dr16_data_t dr16_data_;        //! 最近一次原始帧快照
} DR16_t;

//! 初始化 DR16 对象，绑定 UART 句柄和 RX DMA 缓冲。
err_t DR16_Init(DR16_t *self, UART_HandleTypeDef *uart_handle);
//! 启动 DR16 RX DMA。
err_t DR16_Start(DR16_t *self);
//! 任务周期入口：等待完整帧通知，解析成功则置 online_。
void DR16_Update(DR16_t *self, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
