#include "bsp_usb.h"

#include <string.h>

#include "bsp_log.h"
#include "comp_utils.h"

// ==================== USB CDC 对象表 ====================
//! CubeMX CDC 回调只提供全局 hUsbDeviceFS，BSP 通过句柄反查对象式控制块。
//! 回调只负责分发数据和推进发送状态，协议解析和复杂业务应放在任务上下文。

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

static STM32USB_t* stm32_usb_map[BSP_USB_NUMBER] = {0};

//! 将 USB Device 句柄转换为 BSP USB 逻辑 ID。
//! 当前工程只有 FS CDC 设备，后续新增 HS 或复合设备时优先在这里补映射。
BSP_USB_t BSP_USB_get_id(USBD_HandleTypeDef* usb_handle) {
  if (usb_handle == NULL) {
    return BSP_USB_ID_ERROR;
  }

  if (usb_handle == &hUsbDeviceFS) {
    return BSP_USB_FS;
  }

  return BSP_USB_ID_ERROR;
}

//! 检查 BSP USB 逻辑 ID 是否可用。
//! BSP_USB_ID_ERROR 和越界值都视为非法，避免访问对象表越界。
static bool BSP_USB_is_valid_id(BSP_USB_t id) {
  return (id != BSP_USB_ID_ERROR) && (id < BSP_USB_NUMBER);
}

//! 根据 USB Device 句柄查找已注册的 USB 控制块。
static STM32USB_t* BSP_USB_get_object(USBD_HandleTypeDef* usb_handle) {
  const BSP_USB_t id = BSP_USB_get_id(usb_handle);
  if (!BSP_USB_is_valid_id(id)) {
    return NULL;
  }

  return stm32_usb_map[id];
}

//! 将 USBD CDC 状态转换为 BSP 错误码。
static err_t BSP_USB_status_to_error(uint8_t status) {
  if (status == USBD_OK) {
    return OK;
  }

  if (status == USBD_BUSY) {
    return BUSY;
  }

  if (status == USBD_EMEM) {
    return NO_MEM;
  }

  return FAILED;
}

// ==================== 对象式 USB CDC 接口 ====================

//! 初始化 USB CDC BSP 控制块。
err_t STM32USB_Init(STM32USB_t* self, USBD_HandleTypeDef* usb_handle,
                    STM32USB_RxCallback_t rx_callback,
                    STM32USB_TxCompleteCallback_t tx_callback) {
  if (self == NULL) {
    LOG_ERROR("USB", "Init failed: object is NULL");
    return PTR_NULL;
  }

  self->id_ =
      (usb_handle != NULL) ? BSP_USB_get_id(usb_handle) : BSP_USB_ID_ERROR;
  self->usb_handle_ = usb_handle;
  self->rx_callback_ = rx_callback;
  self->tx_callback_ = tx_callback;
  self->last_error_ = PENDING;

  ASSERT(self->usb_handle_ != NULL);
  if (self->usb_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    LOG_ERROR("USB", "Init failed: device handle is NULL");
    return self->last_error_;
  }

  ASSERT(BSP_USB_is_valid_id(self->id_));
  if (!BSP_USB_is_valid_id(self->id_)) {
    self->last_error_ = NOT_FOUND;
    LOG_ERROR("USB", "Init failed: unsupported USB device");
    return self->last_error_;
  }

  ASSERT((stm32_usb_map[self->id_] == NULL) ||
         (stm32_usb_map[self->id_] == self));
  if ((stm32_usb_map[self->id_] != NULL) &&
      (stm32_usb_map[self->id_] != self)) {
    self->last_error_ = BUSY;
    LOG_ERROR("USB", "Init failed: USB device already registered");
    return self->last_error_;
  }

  stm32_usb_map[self->id_] = self;
  self->last_error_ = OK;
  LOG_INFO("USB", "CDC device initialized");
  return self->last_error_;
}

//! 检查 USB 设备是否已经枚举并进入 CONFIGURED 状态。
bool STM32USB_IsConfigured(const STM32USB_t* self) {
  if (self == NULL) {
    return false;
  }

  if (self->usb_handle_ == NULL) {
    return false;
  }

  return self->usb_handle_->dev_state == USBD_STATE_CONFIGURED;
}

//! 通过 USB CDC 发送一段数据。
//! 发送前复制到 UserTxBufferFS，避免调用方栈上临时数据在 USB IN 完成前失效。
err_t STM32USB_Write(STM32USB_t* self, const uint8_t* data, size_t size) {
  if (self == NULL) {
    LOG_ERROR("USB", "Write failed: object is NULL");
    return PTR_NULL;
  }

  ASSERT(data != NULL);
  if (data == NULL) {
    self->last_error_ = PTR_NULL;
    LOG_ERROR("USB", "Write failed: data is NULL");
    return self->last_error_;
  }

  ASSERT(size > 0U);
  if (size == 0U) {
    self->last_error_ = SIZE_ERR;
    LOG_ERROR("USB", "Write failed: empty payload");
    return self->last_error_;
  }

  ASSERT(size <= APP_TX_DATA_SIZE);
  if (size > APP_TX_DATA_SIZE) {
    self->last_error_ = OUT_OF_RANGE;
    LOG_ERROR("USB", "Write failed: size=%u exceeds buffer=%u", (unsigned)size,
              (unsigned)APP_TX_DATA_SIZE);
    return self->last_error_;
  }

  ASSERT(self->usb_handle_ != NULL);
  if (self->usb_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    LOG_ERROR("USB", "Write failed: device handle is NULL");
    return self->last_error_;
  }

  if (!STM32USB_IsConfigured(self)) {
    self->last_error_ = STATE_ERR;
    LOG_WARN("USB", "Write rejected: device is not configured");
    return self->last_error_;
  }

  USBD_CDC_HandleTypeDef* cdc =
      (USBD_CDC_HandleTypeDef*)self->usb_handle_->pClassData;
  ASSERT(cdc != NULL);
  if (cdc == NULL) {
    self->last_error_ = STATE_ERR;
    LOG_ERROR("USB", "Write failed: CDC class data is NULL");
    return self->last_error_;
  }

  if (cdc->TxState != 0U) {
    self->last_error_ = BUSY;
    LOG_DEBUG("USB", "Write deferred: TX busy");
    return self->last_error_;
  }

  memcpy(UserTxBufferFS, data, size);

  uint8_t status =
      USBD_CDC_SetTxBuffer(self->usb_handle_, UserTxBufferFS, (uint32_t)size);
  VERIFY(status == USBD_OK);
  if (status != USBD_OK) {
    self->last_error_ = BSP_USB_status_to_error(status);
    LOG_ERROR("USB", "Set TX buffer failed: status=%u", (unsigned)status);
    return self->last_error_;
  }

  status = USBD_CDC_TransmitPacket(self->usb_handle_);
  VERIFY(status == USBD_OK);
  self->last_error_ = BSP_USB_status_to_error(status);
  if (self->last_error_ == OK) {
    LOG_DEBUG("USB", "TX started: size=%u", (unsigned)size);
  } else {
    LOG_ERROR("USB", "TX start failed: status=%u", (unsigned)status);
  }
  return self->last_error_;
}

//! 更新 USB CDC RX 用户回调。
void STM32USB_SetRxCallback(STM32USB_t* self, STM32USB_RxCallback_t callback) {
  if (self == NULL) {
    return;
  }

  self->rx_callback_ = callback;
  LOG_DEBUG("USB", "RX callback %s", callback == NULL ? "cleared" : "set");
}

//! 更新 USB CDC TX 完成回调。
void STM32USB_SetTxCompleteCallback(STM32USB_t* self,
                                    STM32USB_TxCompleteCallback_t callback) {
  if (self == NULL) {
    return;
  }

  self->tx_callback_ = callback;
  LOG_DEBUG("USB", "TX callback %s", callback == NULL ? "cleared" : "set");
}

//! 读取最近一次错误码。
err_t STM32USB_GetLastError(const STM32USB_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }

  return self->last_error_;
}

//! 处理 CDC OUT 收到的数据。
void STM32USB_HandleRxData(USBD_HandleTypeDef* usb_handle, uint8_t* data,
                           size_t size) {
  STM32USB_t* usb = BSP_USB_get_object(usb_handle);
  if (usb == NULL) {
    return;
  }

  ASSERT(data != NULL);
  if (data == NULL) {
    usb->last_error_ = PTR_NULL;
    return;
  }

  ASSERT(size > 0U);
  if (size == 0U) {
    usb->last_error_ = SIZE_ERR;
    return;
  }

  ASSERT(usb->rx_callback_ != NULL);
  if (usb->rx_callback_ == NULL) {
    usb->last_error_ = PTR_NULL;
    return;
  }

  usb->last_error_ = OK;
  usb->rx_callback_(data, size);
}

//! 处理 CDC IN 发送完成事件。
void STM32USB_HandleTxComplete(USBD_HandleTypeDef* usb_handle) {
  STM32USB_t* usb = BSP_USB_get_object(usb_handle);
  if (usb == NULL) {
    return;
  }

  usb->last_error_ = OK;

  if (usb->tx_callback_ != NULL) {
    usb->tx_callback_();
  }
}
