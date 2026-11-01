#include "dr16.h"

#include <string.h>

#include "comp_utils.h"

// ==================== DMA 缓冲与对象状态 ====================

static uint8_t dma_buffer_[DR16_FRAME_SIZE * 2];
static uint8_t dr16_rx_buffer_[DR16_FRAME_SIZE];
static uint8_t dr16_frame_accumulator_[DR16_FRAME_SIZE];
static size_t dr16_frame_accumulator_size_ = 0U;
static volatile bool frame_ready_ = false;
static DR16_t *instance_ = NULL;

// ==================== 原始帧校验与解析 ====================

//! 检查 DR16 原始通道值和拨杆值是否落在协议合法范围内。
static bool DR16_DataCorrupted(const dr16_data_t *dr16) {
  ASSERT(dr16);

  if ((dr16->ch_r_x < DR16_CH_VALUE_MIN) || (dr16->ch_r_x > DR16_CH_VALUE_MAX))
    return true;

  if ((dr16->ch_r_y < DR16_CH_VALUE_MIN) || (dr16->ch_r_y > DR16_CH_VALUE_MAX))
    return true;

  if ((dr16->ch_l_x < DR16_CH_VALUE_MIN) || (dr16->ch_l_x > DR16_CH_VALUE_MAX))
    return true;

  if ((dr16->ch_l_y < DR16_CH_VALUE_MIN) || (dr16->ch_l_y > DR16_CH_VALUE_MAX))
    return true;

  if (dr16->sw_l == CMD_SW_ERR)
    return true;

  if (dr16->sw_r == CMD_SW_ERR)
    return true;

  return false;
}

//! 将 18 字节原始帧解析为通用 cmd_rc_t，并把摇杆通道归一化到约 -1~1。
static err_t DR16_ParseRc(const dr16_data_t *dr16, cmd_rc_t *rc) {
  ASSERT(dr16);
  ASSERT(rc);

  if ((dr16 == NULL) || (rc == NULL)) {
    return PTR_NULL;
  }

  if (DR16_DataCorrupted(dr16)) {
    return FAILED;
  }

  memset(rc, 0, sizeof(*rc));

  const float full_range =
      (float)(DR16_CH_VALUE_MAX - DR16_CH_VALUE_MIN); // 归一化分母。

  rc->ch.r.x = 2.0f * ((float)dr16->ch_r_x - DR16_CH_VALUE_MID) / full_range;
  rc->ch.r.y = 2.0f * ((float)dr16->ch_r_y - DR16_CH_VALUE_MID) / full_range;
  rc->ch.l.x = 2.0f * ((float)dr16->ch_l_x - DR16_CH_VALUE_MID) / full_range;
  rc->ch.l.y = 2.0f * ((float)dr16->ch_l_y - DR16_CH_VALUE_MID) / full_range;

  rc->sw_l = (cmd_switch_pos_t)dr16->sw_l;
  rc->sw_r = (cmd_switch_pos_t)dr16->sw_r;

  rc->mouse.x = dr16->x;
  rc->mouse.y = dr16->y;
  rc->mouse.z = dr16->z;
  rc->mouse.click.l = dr16->press_l;
  rc->mouse.click.r = dr16->press_r;

  rc->key = dr16->key;
  rc->ch_res = 2.0f * ((float)dr16->res - DR16_CH_VALUE_MID) / full_range;

  return OK;
}

// ==================== ISR 接收入口 ====================

//! 判断一段候选帧是否已经对齐到 DR16 协议边界。
//! 这里只做整数范围检查，避免在中断中执行浮点解析。
static bool DR16_FrameLooksValid(const uint8_t *data) {
  if (data == NULL) {
    return false;
  }

  dr16_data_t candidate;
  memcpy(&candidate, data, sizeof(candidate));
  return !DR16_DataCorrupted(&candidate);
}

//! RX DMA 接收回调（中断上下文）：接收任意长度片段并重同步完整帧。
//! IDLE、半传输和全传输事件的片段长度都可能不同，不能要求 size 恰好为 18。
static void DR16_RxCallback(uint8_t *data, size_t size) {
  if ((data == NULL) || (size == 0U)) {
    return;
  }

  for (size_t index = 0U; index < size; ++index) {
    dr16_frame_accumulator_[dr16_frame_accumulator_size_++] = data[index];

    if (dr16_frame_accumulator_size_ < DR16_FRAME_SIZE) {
      continue;
    }

    if (DR16_FrameLooksValid(dr16_frame_accumulator_)) {
      memcpy(dr16_rx_buffer_, dr16_frame_accumulator_, DR16_FRAME_SIZE);
      frame_ready_ = true;

      if ((instance_ != NULL) && (instance_->thread_alert != NULL)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(instance_->thread_alert, SIGNAL_DR16_RAW_REDY,
                           eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
      dr16_frame_accumulator_size_ = 0U;
    } else {
      /* 丢弃一个字节后继续尝试，支持从任意 DMA/上电偏移恢复。 */
      memmove(dr16_frame_accumulator_, dr16_frame_accumulator_ + 1U,
              DR16_FRAME_SIZE - 1U);
      dr16_frame_accumulator_size_ = DR16_FRAME_SIZE - 1U;
    }
  }
}

// ==================== 初始化与任务周期处理 ====================

//! 初始化 DR16 对象，绑定 UART RX DMA 和接收回调。
err_t DR16_Init(DR16_t *self, UART_HandleTypeDef *uart_handle) {
  if (self == NULL) {
    return PTR_NULL;
  }

  memset(self, 0, sizeof(*self));
  dr16_frame_accumulator_size_ = 0U;
  frame_ready_ = false;
  self->switch_required = pdFALSE;
  self->online_ = false;
  self->init_error_ = STM32UART_Init(
      &self->uart_, uart_handle,
      (BSP_UART_RawData_t){dma_buffer_, sizeof(dma_buffer_)}, DR16_RxCallback);
  instance_ = self;
  return self->init_error_;
}

//! 启动 DR16 UART DMA 接收。
err_t DR16_Start(DR16_t *self) {
  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->init_error_ != OK) {
    return self->init_error_;
  }
  return STM32UART_SetRxDMA(&self->uart_);
}

//! 等待一帧 DR16 数据；超时或解析失败时清零命令并标记离线。
void DR16_Update(DR16_t *self, uint32_t timeout_ms) {
  if (self == NULL) {
    return;
  }

  uint32_t notify_value = 0;
  (void)xTaskNotifyWait(SIGNAL_DR16_RAW_REDY, UINT32_MAX,
                       &notify_value, pdMS_TO_TICKS(timeout_ms));

  /* 标志检查、快照复制和标志清除必须一起完成，防止接收 ISR
   * 在复制过程中改写快照，或其新帧标志被任务误清除。 */
  taskENTER_CRITICAL();
  bool ready = frame_ready_;
  if (ready) {
    memcpy(&self->dr16_data_, dr16_rx_buffer_, DR16_FRAME_SIZE);
    frame_ready_ = false;
  }
  taskEXIT_CRITICAL();

  if (ready) {
    if (DR16_ParseRc(&self->dr16_data_, &self->dr16_cmd) == OK) {
      self->online_ = true;
    } else {
      /* 收到完整但非法的候选帧时也必须立即进入安全离线状态，
       * 不能继续沿用上一帧的摇杆命令。 */
      self->online_ = false;
      memset(&self->dr16_cmd, 0, sizeof(self->dr16_cmd));
    }
  } else {
    self->online_ = false;
    memset(&self->dr16_cmd, 0, sizeof(self->dr16_cmd));
  }
}
