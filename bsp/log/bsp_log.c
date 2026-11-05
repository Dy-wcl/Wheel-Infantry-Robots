#include "bsp_log.h"

#include <inttypes.h>
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "stm32f4xx.h"
#include "task.h"

#define BSP_LOG_TLS_ERROR_INDEX 0

#if configNUM_THREAD_LOCAL_STORAGE_POINTERS <= BSP_LOG_TLS_ERROR_INDEX
#error "bsp_log requires a reserved FreeRTOS TLS slot"
#endif

#if BSP_LOG_ENABLED
/* RTT 在整个程序运行期间保留通道缓冲区地址，因此必须使用静态存储。
 * 这些缓冲区与每次调用的格式化缓冲区相互独立。 */
static char data_buffer[BSP_LOG_DATA_BUFFER_SIZE];
static char reserved_buffer[BSP_LOG_RESERVED_BUFFER_SIZE];
#endif

static err_t startup_error = OK;
static _Atomic uint8_t current_level = BSP_LOG_DEFAULT_LEVEL;

#if BSP_LOG_ENABLED
static char level_command_buffer[BSP_LOG_COMMAND_BUFFER_SIZE];
static size_t level_command_size;
static bool level_command_overflowed;

static bool log_initialized;

static const char *BSPLogLevelName(bsp_log_level_t level) {
  static const char *const names[] = {"TRACE", "DEBUG", "INFO",
                                      "WARN",  "ERROR", "FATAL"};
  return ((unsigned)level < ARRAY_LEN(names)) ? names[level] : "UNKNOWN";
}
#endif

static bool BSPLogInThreadMode(void) {
  return __get_IPSR() == BSP_LOG_IPSR_THREAD_MODE;
}

#if BSP_LOG_ENABLED
static bool BSPLogContextAllowsWrite(void) {
  BaseType_t scheduler_state;

  if (!BSPLogInThreadMode()) {
    return false;
  }
  /* 只查询一次调度器状态，后续判断使用同一次结果。NOT_STARTED
   * 使用启动阶段状态， RUNNING 允许任务输出；SUSPENDED 拒绝可能触发 libc
   * 内存分配的格式化， 避免在禁止任务切换期间执行该操作。 */
  scheduler_state = xTaskGetSchedulerState();

  /* 调度器状态和两组中断屏蔽条件必须同时满足。 */
  return (scheduler_state == taskSCHEDULER_NOT_STARTED ||
          scheduler_state == taskSCHEDULER_RUNNING) &&
         /* 要求 PRIMASK 为 0，避免在可配置优先级异常被全局屏蔽期间格式化。
          * NMI 和 HardFault 不受 PRIMASK 屏蔽。 */
         (__get_PRIMASK() == BSP_LOG_PRIMASK_UNMASKED) &&
         /* 还要求 BASEPRI 为 0；即使 PRIMASK 为 0，FreeRTOS 临界区仍可能
          * 通过 BASEPRI 屏蔽较低优先级异常。 */
         (__get_BASEPRI() == BSP_LOG_BASEPRI_UNMASKED);
}
#endif

static err_t BSPLogRecordResult(err_t result) {
  if (result == PENDING || !BSPLogInThreadMode()) {
    return result;
  }
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    startup_error = result;
  } else {
    /* NULL 表示当前任务。将状态经 intptr_t 编码为指针值保存，不解引用。
     * 新任务的 TLS 初值为 NULL，解码后恰好为 OK（0）。 */
    vTaskSetThreadLocalStoragePointer(NULL, BSP_LOG_TLS_ERROR_INDEX,
                                      (void *)(intptr_t)result);
  }
  return result;
}

static err_t BSPLogValidateLevel(bsp_log_level_t level) {
  return ((unsigned)level < BSP_LOG_LEVEL_COUNT) ? OK : ARG_ERR;
}

#if BSP_LOG_ENABLED
static bool BSPLogEqualIgnoreCase(const char *left, const char *right) {
  while (*left != '\0' && *right != '\0') {
    char left_char = *left;
    char right_char = *right;
    if (left_char >= 'a' && left_char <= 'z') {
      left_char = (char)(left_char - ('a' - 'A'));
    }
    if (right_char >= 'a' && right_char <= 'z') {
      right_char = (char)(right_char - ('a' - 'A'));
    }
    if (left_char != right_char) {
      return false;
    }
    ++left;
    ++right;
  }
  return *left == '\0' && *right == '\0';
}

static bool BSPLogStartsWithIgnoreCase(const char *value,
                                       const char *prefix) {
  while (*prefix != '\0') {
    char value_char = *value++;
    char prefix_char = *prefix++;
    if (value_char >= 'a' && value_char <= 'z') {
      value_char = (char)(value_char - ('a' - 'A'));
    }
    if (prefix_char >= 'a' && prefix_char <= 'z') {
      prefix_char = (char)(prefix_char - ('a' - 'A'));
    }
    if (value_char != prefix_char) {
      return false;
    }
  }
  return true;
}

static char *BSPLogTrimCommand(char *command) {
  char *first = command;
  char *last;
  while (*first == ' ' || *first == '\t') {
    ++first;
  }
  last = first + strlen(first);
  while (last > first && (last[-1] == ' ' || last[-1] == '\t')) {
    --last;
    *last = '\0';
  }
  return first;
}

static err_t BSPLogParseLevelCommand(const char *command,
                                     bsp_log_level_t *level) {
  static const char *const names[] = {"TRACE", "DEBUG", "INFO",
                                      "WARN",  "ERROR", "FATAL"};
  const char *value = command;
  size_t index;

  if (BSPLogStartsWithIgnoreCase(value, "log_level") &&
      (value[9] == '\0' || value[9] == ' ' || value[9] == '\t' ||
       value[9] == '=')) {
    value += 9;
    while (*value == ' ' || *value == '\t' || *value == '=') {
      ++value;
    }
  } else if (BSPLogStartsWithIgnoreCase(value, "log level") &&
             (value[9] == '\0' || value[9] == ' ' || value[9] == '\t' ||
              value[9] == '=')) {
    value += 9;
    while (*value == ' ' || *value == '\t' || *value == '=') {
      ++value;
    }
  } else if (BSPLogStartsWithIgnoreCase(value, "level") &&
             (value[5] == '\0' || value[5] == ' ' || value[5] == '\t' ||
              value[5] == '=')) {
    value += 5;
    while (*value == ' ' || *value == '\t') {
      ++value;
    }
    if (*value == '=') {
      ++value;
      while (*value == ' ' || *value == '\t') {
        ++value;
      }
    }
  }

  if (value[0] >= '0' && value[0] <= '5' && value[1] == '\0') {
    *level = (bsp_log_level_t)(value[0] - '0');
    return OK;
  }
  for (index = 0u; index < ARRAY_LEN(names); ++index) {
    if (BSPLogEqualIgnoreCase(value, names[index])) {
      *level = (bsp_log_level_t)index;
      return OK;
    }
  }

  return ARG_ERR;
}

static err_t BSPLogApplyLevelCommand(void) {
  bsp_log_level_t level;
  char *command;
  err_t result;

  level_command_buffer[level_command_size] = '\0';
  command = BSPLogTrimCommand(level_command_buffer);
  result = BSPLogParseLevelCommand(command, &level);
  level_command_size = 0u;
  if (result != OK) {
    return BSPLogRecordResult(ARG_ERR);
  }
  return BSPLogSetLevel(level);
}
#endif

#if BSP_LOG_ENABLED
static err_t BSPLogWriteRaw(const char *data, size_t size) {
  const unsigned written =
      SEGGER_RTT_Write(BSP_LOG_CHANNEL_TERMINAL, data, (unsigned)size);
  return (written == size) ? OK : NO_BUFF;
}
#endif

NO_THREAD_SAFE_PROMISE err_t BSPLogInit(void) {
#if BSP_LOG_ENABLED
  err_t result = OK;

  if (!BSPLogInThreadMode() ||
      xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    return BSPLogRecordResult(STATE_ERR);
  }
  if (log_initialized) {
    return BSPLogRecordResult(OK);
  }

  /* 在重复初始化检查之后才重置 RTT 控制块。 */
  SEGGER_RTT_Init();
  /* 显式选择非阻塞整条跳过模式，避免依赖 RTT 默认配置。 */
  if (SEGGER_RTT_SetFlagsUpBuffer(BSP_LOG_CHANNEL_TERMINAL,
                                  SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0) {
    return BSPLogRecordResult(INIT_ERR);
  }
  /* 保留数据通道 1 原有的 JScope 格式描述。 */
  result = (SEGGER_RTT_ConfigUpBuffer(BSP_LOG_CHANNEL_DATA, "JScope_t4f4f4f4",
                                      data_buffer, sizeof(data_buffer),
                                      SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0)
               ? INIT_ERR
               : OK;
  if (result == OK) {
    /* 预留通道 2 与终端文本通道 0 使用不同缓冲区。 */
    result = (SEGGER_RTT_ConfigUpBuffer(
                  BSP_LOG_CHANNEL_RESERVED, "Reserved", reserved_buffer,
                  sizeof(reserved_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0)
                 ? INIT_ERR
                 : OK;
  }
  if (result == OK) {
    /* 所有通道配置成功后，才发布默认等级和可用状态。 */
    atomic_store_explicit(&current_level, BSP_LOG_DEFAULT_LEVEL,
                          memory_order_release);
    log_initialized = true;
  }
  return BSPLogRecordResult(result);
#else
  return BSPLogRecordResult(NOT_SUPPORT);
#endif
}

err_t BSPLogSetLevel(bsp_log_level_t level) {
  const err_t valid = BSPLogValidateLevel(level);
  if (valid != OK) {
    return BSPLogRecordResult(valid);
  }
  atomic_store_explicit(&current_level, (uint8_t)level, memory_order_release);
  return BSPLogRecordResult(OK);
}

err_t BSPLogGetLevel(bsp_log_level_t *level) {
  if (level == NULL) {
    return BSPLogRecordResult(PTR_NULL);
  }
  *level = (bsp_log_level_t)atomic_load_explicit(&current_level,
                                                 memory_order_acquire);
  return OK;
}

err_t BSPLogPollLevelCommand(void) {
#if BSP_LOG_ENABLED
  char input[8];
  unsigned read_size;
  unsigned index;
  err_t result = PENDING;

  if (!BSPLogInThreadMode() ||
      xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ||
      __get_PRIMASK() != BSP_LOG_PRIMASK_UNMASKED ||
      __get_BASEPRI() != BSP_LOG_BASEPRI_UNMASKED) {
    return BSPLogRecordResult(STATE_ERR);
  }
  if (!log_initialized) {
    return BSPLogRecordResult(INIT_ERR);
  }

  do {
    read_size = SEGGER_RTT_Read(BSP_LOG_CHANNEL_COMMAND, input,
                                (unsigned)sizeof(input));
    for (index = 0u; index < read_size; ++index) {
      const char byte = input[index];
      /* 专用下行通道也接受直接发送的等级枚举值，便于上位机脚本控制。 */
      if (!level_command_overflowed && level_command_size == 0u &&
          (unsigned char)byte < BSP_LOG_LEVEL_COUNT) {
        result = BSPLogSetLevel((bsp_log_level_t)(unsigned char)byte);
        continue;
      }
      if (byte == '\r' || byte == '\n') {
        if (level_command_overflowed) {
          level_command_overflowed = false;
          level_command_size = 0u;
          result = BSPLogRecordResult(ARG_ERR);
        } else if (level_command_size != 0u) {
          const err_t command_result = BSPLogApplyLevelCommand();
          result = command_result;
        }
        continue;
      }
      if (level_command_overflowed) {
        continue;
      }
      if (level_command_size + 1u >= sizeof(level_command_buffer)) {
        level_command_size = 0u;
        level_command_overflowed = true;
        continue;
      }
      level_command_buffer[level_command_size++] = byte;
    }
  } while (read_size == sizeof(input));

  return result;
#else
  return BSPLogRecordResult(NOT_SUPPORT);
#endif
}

err_t BSPLogGetLastError(void) {
  if (!BSPLogInThreadMode()) {
    return STATE_ERR;
  }
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return startup_error;
  }
  return (err_t)(intptr_t)pvTaskGetThreadLocalStoragePointer(
      NULL, BSP_LOG_TLS_ERROR_INDEX);
}

err_t BSPLogVWrite(bsp_log_level_t level, const char *tag, const char *format,
                   va_list args) {
#if BSP_LOG_ENABLED
  static const char line_end[] = "\r\n";
  /* 每次调用独占栈缓冲区。格式化在 RTT 锁外完成，锁只保护最后的传输复制。 */
  char line_buffer[BSP_LOG_LINE_BUFFER_SIZE];
  int prefix_size;
  int body_size;
  size_t body_capacity;
  size_t total_size;

  if (BSPLogValidateLevel(level) != OK) {
    return BSPLogRecordResult(ARG_ERR);
  }
  if (tag == NULL || format == NULL) {
    return BSPLogRecordResult(PTR_NULL);
  }
  if (!log_initialized) {
    return BSPLogRecordResult(INIT_ERR);
  }
  /* 等级过滤不执行格式化或 RTT 输出，也不修改已保存的错误状态。
   * PENDING 表示本条记录被丢弃，不存在排队或稍后补发。 */
  if (level < (bsp_log_level_t)atomic_load_explicit(&current_level,
                                                    memory_order_acquire)) {
    return PENDING;
  }
  if (!BSPLogContextAllowsWrite()) {
    return BSPLogRecordResult(STATE_ERR);
  }

  /* 检查 snprintf 返回的完整所需长度，而非实际写入长度。
   * 即使正文为空，也至少为 CR、LF 和 NUL 预留三个字节。 */
  prefix_size = snprintf(line_buffer, sizeof(line_buffer), "[%s][%s] ",
                         BSPLogLevelName(level), tag);
  if (prefix_size < 0) {
    return BSPLogRecordResult(FAILED);
  }
  if ((size_t)prefix_size > sizeof(line_buffer) - sizeof(line_end)) {
    return BSPLogRecordResult(SIZE_ERR);
  }
  /* 先预留 CRLF，再将包含 NUL 空间的剩余容量交给 vsnprintf。 */
  body_capacity =
      sizeof(line_buffer) - (size_t)prefix_size - (sizeof(line_end) - 1u);
  body_size = vsnprintf(line_buffer + prefix_size, body_capacity, format, args);
  if (body_size < 0) {
    return BSPLogRecordResult(FAILED);
  }
  if ((size_t)body_size >= body_capacity) {
    /* 拒绝发送被截断的半条记录，避免误读诊断信息。 */
    return BSPLogRecordResult(SIZE_ERR);
  }
  /* 用 CRLF 覆盖正文结束符，再附加 NUL；RTT 发送长度不包含 NUL。 */
  total_size = (size_t)prefix_size + (size_t)body_size;
  memcpy(line_buffer + total_size, line_end, sizeof(line_end));
  total_size += sizeof(line_end) - 1u;

  return BSPLogRecordResult(BSPLogWriteRaw(line_buffer, total_size));
#else
  /* 日志关闭时，直接调用接口仍返回明确状态；LOG_* 宏则完全不求值参数。 */
  RM_UNUSED(level);
  RM_UNUSED(tag);
  RM_UNUSED(format);
  RM_UNUSED(args);
  return BSPLogRecordResult(NOT_SUPPORT);
#endif
}

err_t BSPLogWrite(bsp_log_level_t level, const char *tag, const char *format,
                  ...) {
  err_t result;
  va_list args;
  va_start(args, format);
  result = BSPLogVWrite(level, tag, format, args);
  va_end(args);
  return result;
}

err_t BSPLogClear(void) {
#if BSP_LOG_ENABLED
  static const char clear[] = "  " RTT_CTRL_CLEAR;

  if (!log_initialized) {
    return BSPLogRecordResult(INIT_ERR);
  }
  if (!BSPLogContextAllowsWrite()) {
    return BSPLogRecordResult(STATE_ERR);
  }
  return BSPLogRecordResult(BSPLogWriteRaw(clear, sizeof(clear) - 1u));
#else
  return BSPLogRecordResult(NOT_SUPPORT);
#endif
}

int Float2StrN(char *str, size_t size, float va) {
  double rounded;
  uint64_t magnitude;
  uint64_t integer;
  uint32_t fraction;
  int result;

  if ((str == NULL) || (size == 0u) || !isfinite(va)) {
    if ((str != NULL) && (size != 0u)) {
      str[0] = '\0';
    }
    return -1;
  }

  /* 保留三位小数，中点向远离零的方向取整。避免直接使用 %.3f，
   * 以保留现有的边界舍入和零值符号规则。 */
  rounded = round((double)va * 1000.0);

  /* INT64_MAX 转为 double 后会成为 2^63，因此使用精确的二进制边界检查。
   * 先验证有符号范围，避免有风险的 double 到 int64_t 转换；
   * 通过检查后，绝对值可安全存入 uint64_t。 */
  if (rounded < -0x1p63 || rounded >= 0x1p63) {
    str[0] = '\0';
    return -1;
  }

  /* 仅对严格小于零的取整结果输出负号，取整后为零时输出 0.000。
   * 将整数按十亿拆为 uint32_t 分组，兼容不支持 long long 格式的 newlib-nano。
   * 存在高位分组时，低位补齐九位，小数补齐三位；由 snprintf 完成
   * 十进制转换、截断、NUL 终止和完整所需长度的计算。 */
  magnitude = (uint64_t)fabs(rounded);
  integer = magnitude / 1000u;
  fraction = (uint32_t)(magnitude % 1000u);
  if (integer < 1000000000u) {
    result = snprintf(str, size, "%s%" PRIu32 ".%03" PRIu32,
                      rounded < 0.0 ? "-" : "", (uint32_t)integer, fraction);
  } else {
    result =
        snprintf(str, size, "%s%" PRIu32 "%09" PRIu32 ".%03" PRIu32,
                 rounded < 0.0 ? "-" : "", (uint32_t)(integer / 1000000000u),
                 (uint32_t)(integer % 1000000000u), fraction);
  }
  if (result < 0) {
    str[0] = '\0';
    return -1;
  }
  return result;
}
