#include "bsp_gpio.h"

#include <stddef.h>

#include "bsp_log.h"
#include "comp_utils.h"

enum {
  GPIO_LINE_COUNT = 16,    /* STM32 GPIO/EXTI 线数量 */
  GPIO_INVALID_LINE = 0xFF /* 非法 GPIO 引脚对应的 line 标记 */
};

/* EXTI 线由所有 GPIO 端口共享，因此每条线只映射一个 GPIO 对象。 */
static GPIO* gpio_map[GPIO_LINE_COUNT] = {0};

/**
 * @brief 将单 bit GPIO 引脚掩码转换为 EXTI 线号。
 * @param pin GPIO 引脚掩码，例如 GPIO_PIN_3。
 * @return 0~15 表示有效线号，GPIO_INVALID_LINE 表示参数非法。
 */
static uint8_t GPIO_PinToLine(uint16_t pin) {
  uint8_t line = 0U;

  /* 合法引脚必须且只能包含一个 bit。 */
  if ((pin == 0U) || ((pin & (uint16_t)(pin - 1U)) != 0U)) {
    return GPIO_INVALID_LINE;
  }

  /* 通过右移统计最低有效 bit 的位置。 */
  while ((pin >>= 1U) != 0U) {
    ++line;
  }
  return line;
}

/**
 * @brief 保存 GPIO 对象最近一次操作的错误码。
 * @param gpio GPIO 对象。
 * @param error 要保存的错误码。
 */
static void GPIO_SetError(GPIO* gpio, err_t error) {
  if (gpio != NULL) {
    gpio->last_error = error;
  }
}

/**
 * @brief 初始化 GPIO 对象并注册 EXTI 映射。
 * @param gpio 由调用方提供的 GPIO 对象存储区。
 * @param port GPIO 端口，例如 GPIOA、GPIOB。
 * @param pin GPIO 引脚掩码，只支持单个引脚。
 * @param irq 对应的 NVIC 中断号；NonMaskableInt_IRQn 表示不使用中断。
 * @return 初始化后的 GPIO 对象副本。
 */
GPIO GPIO_Init(GPIO* gpio, GPIO_TypeDef* port, uint16_t pin, IRQn_Type irq) {
  GPIO empty = {0};
  const uint8_t line = GPIO_PinToLine(pin);
  const uint8_t old_line =
      (gpio != NULL) ? GPIO_PinToLine(gpio->pin) : GPIO_INVALID_LINE;

  if (gpio == NULL) {
    LOG_ERROR("GPIO", "Init failed: object is NULL");
    return empty;
  }

  /* 对象重复使用时，先清除旧的 EXTI 映射。 */
  if ((old_line != GPIO_INVALID_LINE) && (gpio_map[old_line] == gpio)) {
    gpio_map[old_line] = NULL;
  }

  gpio->port = port;
  gpio->pin = pin;
  gpio->irq = irq;
  gpio->callback = NULL;
  gpio->callback_context = NULL;
  gpio->last_error = OK;

  if ((port == NULL) || (line == GPIO_INVALID_LINE)) {
    gpio->last_error = ARG_ERR;
    LOG_ERROR("GPIO", "Init failed: port or pin is invalid (pin=0x%04X)",
              (unsigned)pin);
    return *gpio;
  }

  /* 只有配置了有效 IRQ 的对象才参与中断分发。 */
  if (irq != NonMaskableInt_IRQn) {
    gpio_map[line] = gpio;
  }

  LOG_INFO("GPIO", "Initialized pin=0x%04X line=%u irq=%d", (unsigned)pin,
           (unsigned)line, (int)irq);
  return *gpio;
}

/**
 * @brief 读取 GPIO 输入电平。
 * @param gpio GPIO 对象。
 * @return true 表示高电平，false 表示低电平。
 */
bool GPIO_Read(const GPIO* gpio) {
  ASSERT(gpio != NULL);
  ASSERT(gpio->port != NULL);
  const bool value = (gpio->port->IDR & gpio->pin) != 0U;
  LOG_TRACE("GPIO", "Read pin=0x%04X value=%u", (unsigned)gpio->pin,
            value ? 1U : 0U);
  return value;
}

/**
 * @brief 写入 GPIO 输出电平。
 * @param gpio GPIO 对象。
 * @param value true 输出高电平，false 输出低电平。
 */
void GPIO_Write(GPIO* gpio, bool value) {
  ASSERT(gpio != NULL);
  ASSERT(gpio->port != NULL);

  if (value) {
    gpio->port->BSRR = (uint32_t)gpio->pin;
  } else {
    gpio->port->BSRR = (uint32_t)gpio->pin << 16U;
  }
  LOG_TRACE("GPIO", "Write pin=0x%04X value=%u", (unsigned)gpio->pin,
            value ? 1U : 0U);
}

/**
 * @brief 使能 GPIO 对应的 NVIC 中断。
 * @param gpio GPIO 对象。
 * @return OK 表示成功，PTR_NULL 或 STATE_ERR 表示参数/状态错误。
 */
err_t GPIO_EnableInterrupt(GPIO* gpio) {
  if (gpio == NULL) {
    LOG_ERROR("GPIO", "Enable interrupt failed: object is NULL");
    return PTR_NULL;
  }
  if (gpio->irq == NonMaskableInt_IRQn) {
    GPIO_SetError(gpio, STATE_ERR);
    LOG_WARN("GPIO", "Enable interrupt rejected: no IRQ configured");
    return gpio->last_error;
  }

  HAL_NVIC_EnableIRQ(gpio->irq);
  GPIO_SetError(gpio, OK);
  LOG_INFO("GPIO", "Interrupt enabled for pin=0x%04X", (unsigned)gpio->pin);
  return gpio->last_error;
}

/**
 * @brief 禁用 GPIO 对应的 NVIC 中断。
 * @param gpio GPIO 对象。
 * @return OK 表示成功，PTR_NULL 或 STATE_ERR 表示参数/状态错误。
 */
err_t GPIO_DisableInterrupt(GPIO* gpio) {
  if (gpio == NULL) {
    LOG_ERROR("GPIO", "Disable interrupt failed: object is NULL");
    return PTR_NULL;
  }
  if (gpio->irq == NonMaskableInt_IRQn) {
    GPIO_SetError(gpio, STATE_ERR);
    LOG_WARN("GPIO", "Disable interrupt rejected: no IRQ configured");
    return gpio->last_error;
  }

  HAL_NVIC_DisableIRQ(gpio->irq);
  GPIO_SetError(gpio, OK);
  LOG_INFO("GPIO", "Interrupt disabled for pin=0x%04X", (unsigned)gpio->pin);
  return gpio->last_error;
}

/**
 * @brief 使用 HAL 配置 GPIO 方向、上下拉和中断触发方式。
 * @param gpio GPIO 对象。
 * @param config GPIO 方向及上下拉配置。
 * @return OK 表示配置成功，其他值表示参数错误。
 */
err_t GPIO_SetConfig(GPIO* gpio, GPIO_Configuration config) {
  GPIO_InitTypeDef init = {0};

  if ((gpio == NULL) || (gpio->port == NULL) ||
      (GPIO_PinToLine(gpio->pin) == GPIO_INVALID_LINE)) {
    GPIO_SetError(gpio, (gpio == NULL) ? PTR_NULL : ARG_ERR);
    LOG_ERROR("GPIO", "SetConfig failed: invalid object or pin");
    return (gpio == NULL) ? PTR_NULL : gpio->last_error;
  }

  init.Pin = gpio->pin;
  /* 将自定义方向枚举转换为 STM32 HAL 工作模式。 */
  switch (config.direction) {
    case GPIO_DIRECTION_INPUT:
      init.Mode = GPIO_MODE_INPUT;
      break;
    case GPIO_DIRECTION_OUTPUT_PUSH_PULL:
      init.Mode = GPIO_MODE_OUTPUT_PP;
      break;
    case GPIO_DIRECTION_OUTPUT_OPEN_DRAIN:
      init.Mode = GPIO_MODE_OUTPUT_OD;
      break;
    case GPIO_DIRECTION_FALL_INTERRUPT:
      init.Mode = GPIO_MODE_IT_FALLING;
      break;
    case GPIO_DIRECTION_RISING_INTERRUPT:
      init.Mode = GPIO_MODE_IT_RISING;
      break;
    case GPIO_DIRECTION_FALL_RISING_INTERRUPT:
      init.Mode = GPIO_MODE_IT_RISING_FALLING;
      break;
    default:
      GPIO_SetError(gpio, ARG_ERR);
      LOG_ERROR("GPIO", "SetConfig failed: invalid direction=%d",
                (int)config.direction);
      return gpio->last_error;
  }

  /* 将自定义上下拉枚举转换为 HAL 上下拉配置。 */
  switch (config.pull) {
    case GPIO_PULL_NONE:
      init.Pull = GPIO_NOPULL;
      break;
    case GPIO_PULL_UP:
      init.Pull = GPIO_PULLUP;
      break;
    case GPIO_PULL_DOWN:
      init.Pull = GPIO_PULLDOWN;
      break;
    default:
      GPIO_SetError(gpio, ARG_ERR);
      LOG_ERROR("GPIO", "SetConfig failed: invalid pull=%d", (int)config.pull);
      return gpio->last_error;
  }

  init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(gpio->port, &init);
  GPIO_SetError(gpio, OK);
  LOG_INFO("GPIO", "Configured pin=0x%04X direction=%d pull=%d",
           (unsigned)gpio->pin, (int)config.direction, (int)config.pull);
  return gpio->last_error;
}

/**
 * @brief 注册 GPIO EXTI 事件回调。
 * @param gpio GPIO 对象。
 * @param callback 中断触发时调用的回调函数，可传 NULL 清除回调。
 * @param context 传递给回调的用户上下文。
 * @return OK 表示注册成功，PTR_NULL 表示 GPIO 对象为空。
 */
err_t GPIO_RegisterCallback(GPIO* gpio, GPIO_Callback callback, void* context) {
  if (gpio == NULL) {
    LOG_ERROR("GPIO", "Register callback failed: object is NULL");
    return PTR_NULL;
  }

  gpio->callback = callback;
  gpio->callback_context = context;
  GPIO_SetError(gpio, OK);
  LOG_DEBUG("GPIO", "Callback %s for pin=0x%04X",
            callback == NULL ? "cleared" : "registered", (unsigned)gpio->pin);
  return gpio->last_error;
}

/**
 * @brief 获取 GPIO 对象最近一次操作的错误码。
 * @param gpio GPIO 对象。
 * @return 最近一次错误码；传入 NULL 时返回 PTR_NULL。
 */
err_t GPIO_GetLastError(const GPIO* gpio) {
  return (gpio != NULL) ? gpio->last_error : PTR_NULL;
}

/**
 * @brief 根据 EXTI 线号查找对象并执行用户回调。
 * @param pin HAL 传入的 GPIO 引脚掩码。
 */
void GPIO_DispatchInterrupt(uint16_t pin) {
  const uint8_t line = GPIO_PinToLine(pin);
  if (line == GPIO_INVALID_LINE) {
    return;
  }

  GPIO* gpio = gpio_map[line];
  /* 无映射或未注册回调时，忽略本次中断。 */
  if ((gpio != NULL) && (gpio->callback != NULL)) {
    gpio->callback(gpio, true, gpio->callback_context);
  }
}

/** @brief HAL 通用 EXTI 回调入口。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  GPIO_DispatchInterrupt(GPIO_Pin);
}

/** @brief HAL 上升沿 EXTI 回调入口。 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  GPIO_DispatchInterrupt(GPIO_Pin);
}

/** @brief HAL 下降沿 EXTI 回调入口。 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
  GPIO_DispatchInterrupt(GPIO_Pin);
}
