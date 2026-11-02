#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

#if BSP_LOG_ENABLED
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "task.h"

static char data_buffer[1024];
static char reserved_buffer[256];
/* Keep the complete formatted line out of the caller's task stack. */
static char line_buffer[512];
static StaticSemaphore_t line_mutex_storage;
static SemaphoreHandle_t line_mutex;
#endif

static int last_error;

int BSPLogInit(void) {
#if BSP_LOG_ENABLED

  if (line_mutex == NULL) {
    line_mutex = xSemaphoreCreateMutexStatic(&line_mutex_storage);
  }
  if (line_mutex == NULL) {
    last_error = -1;
    return last_error;
  }

  SEGGER_RTT_Init();
  last_error = SEGGER_RTT_ConfigUpBuffer(BSP_LOG_CHANNEL_DATA, "DataOut",
                                         data_buffer, sizeof(data_buffer),
                                         SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  if (last_error == 0) {
    last_error = SEGGER_RTT_ConfigUpBuffer(
        BSP_LOG_CHANNEL_RESERVED, "Reserved", reserved_buffer,
        sizeof(reserved_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  }

  if (last_error == 0) {
    last_error = SEGGER_RTT_SetTerminal(BSP_LOG_TERMINAL_DEFAULT);
  }
  return last_error;
#else
  last_error = -1;
  return last_error;
#endif
}

int BSPLogGetLastError(void) { return last_error; }

int BSPLogConfigUpBuffer(unsigned index, const char *name, void *buffer,
                         unsigned size, unsigned flags) {
#if BSP_LOG_ENABLED
  last_error = SEGGER_RTT_ConfigUpBuffer(index, name, buffer, size, flags);
  return last_error;
#else
  (void)index;
  (void)name;
  (void)buffer;
  (void)size;
  (void)flags;
  last_error = -1;
  return -1;
#endif
}

int BSPLogConfigDownBuffer(unsigned index, const char *name, void *buffer,
                           unsigned size, unsigned flags) {
#if BSP_LOG_ENABLED
  last_error = SEGGER_RTT_ConfigDownBuffer(index, name, buffer, size, flags);
  return last_error;
#else
  (void)index;
  (void)name;
  (void)buffer;
  (void)size;
  (void)flags;
  last_error = -1;
  return -1;
#endif
}

int BSPLogSelectTerminal(unsigned char terminal_id) {
#if BSP_LOG_ENABLED
  last_error = SEGGER_RTT_SetTerminal(terminal_id);
  return last_error;
#else
  (void)terminal_id;
  last_error = -1;
  return -1;
#endif
}

int BSPLogWriteChannel(unsigned channel, const void *data, unsigned size) {
#if BSP_LOG_ENABLED
  return (int)SEGGER_RTT_Write(channel, data, size);
#else
  (void)channel;
  (void)data;
  (void)size;
  return -1;
#endif
}

int BSPLogPrintfTerminal(unsigned char terminal_id, const char *type,
                         const char *color, const char *format, ...) {
#if BSP_LOG_ENABLED
  static const char suffix[] = "\r\n" RTT_CTRL_RESET;
  const size_t suffix_size = sizeof(suffix) - 1u;
  size_t prefix_size;
  size_t body_capacity;
  size_t body_size;
  int written;
  int result = -1;
  BaseType_t lock_required;
  va_list args;

  if ((line_mutex == NULL) || (__get_IPSR() != 0U) || (suffix_size >= 512)) {
    return -1;
  }

  /* Before the scheduler starts there is only one caller. Avoid taking a
   * mutex without a current task; ISR callers were rejected above. */
  lock_required = (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
  if (lock_required) {
    /* A task must be able to block while another task owns the buffer. */
    if ((xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) ||
        (__get_PRIMASK() != 0U) || (__get_BASEPRI() != 0U)) {
      return -1;
    }
    if (xSemaphoreTake(line_mutex, portMAX_DELAY) != pdTRUE) {
      return -1;
    }
  }

  written = snprintf(line_buffer, sizeof(line_buffer), "  %s%s", color, type);
  if (written < 0 || (size_t)written >= sizeof(line_buffer) - suffix_size) {
    goto out;
  }
  prefix_size = (size_t)written;
  body_capacity = sizeof(line_buffer) - prefix_size - suffix_size - 1u;

  va_start(args, format);
  written =
      vsnprintf(line_buffer + prefix_size, body_capacity + 1u, format, args);
  va_end(args);
  if (written < 0) {
    goto out;
  }
  body_size = (size_t)written;
  if (body_size > body_capacity) {
    body_size = body_capacity;
  }

  memcpy(line_buffer + prefix_size + body_size, suffix, suffix_size + 1u);
  result = SEGGER_RTT_TerminalOut(terminal_id, line_buffer);

out:
  if (lock_required) {
    (void)xSemaphoreGive(line_mutex);
  }
  return result;
#else
  (void)terminal_id;
  (void)type;
  (void)color;
  (void)format;
  return -1;
#endif
}

int PrintLog(const char *fmt, ...) {
#if BSP_LOG_ENABLED
  va_list args;
  va_start(args, fmt);
  int n = SEGGER_RTT_vprintf(BSP_LOG_CHANNEL_TERMINAL, fmt, &args);
  va_end(args);
  return n;
#else
  (void)fmt;
  return 0;
#endif
}

void Float2Str(char *str, float va) {
  int flag = va < 0;
  int head = (int)va;
  int point = (int)((va - head) * 1000);
  head = abs(head);
  point = abs(point);
  if (flag) {
    sprintf(str, "-%d.%d", head, point);
  } else {
    sprintf(str, "%d.%d", head, point);
  }
}
