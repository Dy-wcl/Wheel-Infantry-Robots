#include "i6x.h"

#include <string.h>

#include "comp_utils.h"

// ==================== DMA 缓冲与对象状态 ====================

static uint8_t dma_buffer_[I6X_FRAME_SIZE * 2u];
static uint8_t i6x_rx_buffer_[I6X_FRAME_SIZE];
static volatile bool frame_ready_ = false;
static I6X_t *instance_ = NULL;
static i6x_frame_sync_t frame_sync_;
static uint32_t i6x_last_rx_error_count_ = 0U;

// ==================== 原始帧校验与解析 ====================

//! 将 iBus 通道值归一化到 [-1, 1]。
static float I6X_NormalizeChannel(uint16_t raw)
{
  float out = ((float)raw - (float)I6X_CH_VALUE_MID) /
              (float)(I6X_CH_VALUE_MAX - I6X_CH_VALUE_MID);
  clampf(&out, -1.0f, 1.0f);
  return out;
}

//! 判断 14 路通道是否均落在常见 iBus 有效范围内。
static bool I6X_ChannelOutOfRange(const uint16_t channels[I6X_CHANNEL_COUNT])
{
  ASSERT(channels);

  for (size_t i = 0; i < I6X_CHANNEL_COUNT; i++) {
    if ((channels[i] < I6X_CH_VALUE_MIN) || (channels[i] > I6X_CH_VALUE_MAX)) {
      return true;
    }
  }

  return false;
}

//! 计算 iBus 校验：0xFFFF 减去前 30 个字节。
static uint16_t I6X_CalcChecksum(const uint8_t frame[I6X_FRAME_SIZE])
{
  uint16_t checksum = 0xFFFFu;

  ASSERT(frame);
  if (frame == NULL) {
    return 0u;
  }

  for (size_t i = 0; i < I6X_FRAME_SIZE - 2u; i++) {
    checksum = (uint16_t)(checksum - frame[i]);
  }

  return checksum;
}

void I6X_FrameSyncReset(i6x_frame_sync_t *sync)
{
  if (sync == NULL) {
    return;
  }

  memset(sync, 0, sizeof(*sync));
}

bool I6X_FrameSyncPush(i6x_frame_sync_t *sync, uint8_t byte,
                       uint8_t out_frame[I6X_FRAME_SIZE])
{
  if ((sync == NULL) || (out_frame == NULL)) {
    return false;
  }

  if (sync->index == 0u) {
    if (byte != I6X_FRAME_LENGTH) {
      return false;
    }
    sync->buffer[sync->index++] = byte;
    return false;
  }

  if (sync->index == 1u) {
    if (byte != I6X_FRAME_COMMAND) {
      sync->index = (byte == I6X_FRAME_LENGTH) ? 1u : 0u;
      sync->buffer[0] = (byte == I6X_FRAME_LENGTH) ? byte : 0u;
      return false;
    }
    sync->buffer[sync->index++] = byte;
    return false;
  }

  sync->buffer[sync->index++] = byte;
  if (sync->index >= I6X_FRAME_SIZE) {
    memcpy(out_frame, sync->buffer, I6X_FRAME_SIZE);
    sync->index = 0u;
    return true;
  }

  return false;
}

err_t I6X_DecodeFrame(const uint8_t frame[I6X_FRAME_SIZE], i6x_cmd_rc_t *cmd)
{
  uint16_t channels[I6X_CHANNEL_COUNT] = {0};
  i6x_cmd_rc_t decoded;

  ASSERT(frame);
  ASSERT(cmd);
  if ((frame == NULL) || (cmd == NULL)) {
    return PTR_NULL;
  }

  if ((frame[0] != I6X_FRAME_LENGTH) || (frame[1] != I6X_FRAME_COMMAND)) {
    return FAILED;
  }

  const uint16_t checksum_cal = I6X_CalcChecksum(frame);
  const uint16_t checksum_rx = (uint16_t)(((uint16_t)frame[31] << 8) | frame[30]);
  if (checksum_cal != checksum_rx) {
    return CHECK_ERR;
  }

  for (size_t i = 0; i < I6X_CHANNEL_COUNT; i++) {
    channels[i] = (uint16_t)(((uint16_t)frame[3u + i * 2u] << 8) |
                             frame[2u + i * 2u]);
  }

  if (I6X_ChannelOutOfRange(channels)) {
    return OUT_OF_RANGE;
  }

  memset(&decoded, 0, sizeof(decoded));
  memcpy(decoded.channel, channels, sizeof(decoded.channel));

  decoded.ch.r.x = I6X_NormalizeChannel(channels[0]);
  decoded.ch.r.y = I6X_NormalizeChannel(channels[1]);
  decoded.ch.l.y = I6X_NormalizeChannel(channels[2]);
  decoded.ch.l.x = I6X_NormalizeChannel(channels[3]);

  for (size_t i = 0; i < I6X_AUX_CHANNEL_COUNT; i++) {
    decoded.aux[i] = I6X_NormalizeChannel(channels[i + 4u]);
  }

  decoded.checksum_cal = checksum_cal;
  decoded.checksum_rx = checksum_rx;
  decoded.frame.length = frame[0];
  decoded.frame.command = frame[1];
  decoded.frame.valid = true;

  *cmd = decoded;
  return OK;
}

// ==================== ISR 接收入口 ====================

//! RX DMA 接收回调（中断上下文）：拼出候选完整帧并通知任务解析。
static void I6X_RxCallback(uint8_t *data, size_t size)
{
  if (data == NULL) {
    return;
  }

  if (instance_ != NULL) {
    const uint32_t rx_err_count = STM32UART_GetRxErrorCount(&instance_->uart_);
    if (rx_err_count != i6x_last_rx_error_count_) {
      i6x_last_rx_error_count_ = rx_err_count;
      I6X_FrameSyncReset(&frame_sync_);
    }
  }

  for (size_t i = 0; i < size; i++) {
    if (I6X_FrameSyncPush(&frame_sync_, data[i], i6x_rx_buffer_)) {
      frame_ready_ = true;

      if ((instance_ != NULL) && (instance_->thread_alert != NULL)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(instance_->thread_alert, SIGNAL_I6X_RAW_READY,
                           eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
}

// ==================== 初始化与任务周期处理 ====================

err_t I6X_Init(I6X_t *self, UART_HandleTypeDef *uart_handle)
{
  if (self == NULL) {
    return PTR_NULL;
  }

  memset(self, 0, sizeof(*self));
  self->switch_required = pdFALSE;
  self->online_ = false;
  I6X_FrameSyncReset(&frame_sync_);
  i6x_last_rx_error_count_ = 0U;
  frame_ready_ = false;
  self->init_error_ = STM32UART_Init(
      &self->uart_, uart_handle,
      (BSP_UART_RawData_t){dma_buffer_, sizeof(dma_buffer_)}, I6X_RxCallback);
  instance_ = self;
  return self->init_error_;
}

err_t I6X_Start(I6X_t *self)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->init_error_ != OK) {
    return self->init_error_;
  }
  return STM32UART_SetRxDMA(&self->uart_);
}

void I6X_Update(I6X_t *self, uint32_t timeout_ms)
{
  if (self == NULL) {
    return;
  }

  uint32_t notify_value = 0;
  (void)xTaskNotifyWait(0U, SIGNAL_I6X_RAW_READY, &notify_value,
                        pdMS_TO_TICKS(timeout_ms));

  taskENTER_CRITICAL();
  const bool ready = frame_ready_;
  if (ready) {
    memcpy(self->raw_frame, i6x_rx_buffer_, I6X_FRAME_SIZE);
    frame_ready_ = false;
  }
  taskEXIT_CRITICAL();

  if (ready) {
    if (I6X_DecodeFrame(self->raw_frame, &self->cmd) == OK) {
      self->online_ = true;
    } else {
      self->online_ = false;
      memset(&self->cmd, 0, sizeof(self->cmd));
    }
  } else {
    self->online_ = false;
    memset(&self->cmd, 0, sizeof(self->cmd));
  }

  if (STM32UART_GetLastError(&self->uart_) != OK) {
    taskENTER_CRITICAL();
    I6X_FrameSyncReset(&frame_sync_);
    i6x_last_rx_error_count_ = STM32UART_GetRxErrorCount(&self->uart_);
    taskEXIT_CRITICAL();
    (void)STM32UART_RestartRxDMA(&self->uart_);
  }
}

