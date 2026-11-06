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

/**
 * @brief 验证日志等级是否在有效范围内。
 *
 * @param[in] level 要验证的日志等级。
 * @return 验证结果，具体取值如下。
 * @retval OK 等级有效。
 * @retval ARG_ERR 等级无效。
 * @note 该函数不修改已保存的错误状态。
 */
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

/**
 * @brief 检查 value 是否以 prefix 开头，忽略大小写。
 *
 * @param[in] value 要判断的字符串
 * @param[in] prefix 需要匹配的字符串
 * @return 如果 value 以 prefix 开头，返回 true；否则返回 false。
 * @note 仅在调用期间借用输入字符串；不修改输入内容。
 */
static bool BSPLogStartsWithIgnoreCase(const char *value, const char *prefix) {
  /*在前缀和输入字符串没有结果前逐字符比较*/
  while (*prefix != '\0' && *value != '\0') {
    char value_char = *value++;
    char prefix_char = *prefix++;
    /* 将小写字母转换为大写字母进行比较 */
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
  /*需要与prefix完全匹配*/
  return *prefix == '\0';
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

/** @brief 解析接收到的日志等级命令，根据解析结果设置日志等级
 *
 * @param[in] command 接收到的日志等级命令，
 * @param[out] level The parsed log level.
 * @return  日志解析结结果，具体取值如下。
 * @retval OK 解析成功，成功设置日志等级。
 * @retval ARG_ERR 解析失败，日志等级无效。
 * @throw None.
 * @note 日志等级命令格式为 `log level <name>`，其中 `<name>`
 * 为日志等级名称（TRACE、DEBUG、INFO、WARN、ERROR、FATAL）。
 */
static err_t BSPLogParseLevelCommand(const char *command,
                                     bsp_log_level_t *level) {
  static const char *const names[] = {"TRACE", "DEBUG", "INFO",
                                      "WARN",  "ERROR", "FATAL"};
  const char *value;
  size_t index;

  /* 判断命令格式是否符合log level */
  if (!BSPLogStartsWithIgnoreCase(command, "log level") ||
      (command[9] != ' ' && command[9] != '\t')) {
    return ARG_ERR;
  }
  value = command + 9;
  while (*value == ' ' || *value == '\t') {
    ++value;
  }

  for (index = 0u; index < ARRAY_LEN(names); ++index) {
    if (BSPLogEqualIgnoreCase(value, names[index])) {
      *level = (bsp_log_level_t)index;
      return OK;
    }
  }

  return ARG_ERR;
}

/**
 * @brief Applies the complete textual log-level command in the shared buffer.
 *
 * Terminates the buffered command, trims surrounding whitespace, parses the
 * `log level <name>` form, and applies the resulting log level. The buffered
 * command length is cleared before the function returns.
 *
 * @return The command result. Returns `OK` when the parsed level is applied or
 *         `ARG_ERR` when the command is invalid.
 * @throws None.
 * @note The function consumes the command currently stored in
 *       `level_command_buffer` and resets `level_command_size` even after a
 *       parse failure.
 * @warning The caller serializes access to the shared command buffer state.
 * @since 1.0.1.
 * @ingroup bsp_log.
 */
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

err_t BSPLogInit(void) {
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
  /*
   * 这是一次 SEGGER_RTT_Read() 使用的临时块，而不是命令缓冲区。
   * 8 字节在栈占用、单次轮询工作量和 RTT 吞吐之间取折中；如果本次
   * 读取恰好填满该块，下面的循环会继续读取下一块，因此不会限制命令
   * 长度。命令本身的上限由 BSP_LOG_COMMAND_BUFFER_SIZE 控制。
   */
  char input[BSP_LOG_COMMAND_READ_BUFFER_SIZE];
  unsigned read_size;
  unsigned index;
  err_t result = PENDING;

  /*
   * 轮询会访问 RTT 和 FreeRTOS 状态，只允许在线程模式、调度器未挂起、
   * 且 PRIMASK/BASEPRI 都未屏蔽中断时执行。这样可以避免在临界区或 ISR
   * 中调用可能依赖锁的 RTT 接口。
   */
  if (!BSPLogInThreadMode() ||
      xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ||
      __get_PRIMASK() != BSP_LOG_PRIMASK_UNMASKED ||
      __get_BASEPRI() != BSP_LOG_BASEPRI_UNMASKED) {
    return BSPLogRecordResult(STATE_ERR);
  }
  if (!log_initialized) {
    return BSPLogRecordResult(INIT_ERR);
  }

  /* RTT 可能一次提供多于临时块大小的数据；读满一块时继续排空通道。 */
  do {
    read_size = SEGGER_RTT_Read(BSP_LOG_CHANNEL_COMMAND, input,
                                (unsigned)sizeof(input));
    for (index = 0u; index < read_size; ++index) {
      const char byte = input[index];
      /* CR/LF 提交当前文本命令；超长命令在结束符处统一报告参数错误。 */
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

      /* 已经溢出的命令必须丢弃到下一个 CR/LF，避免拼接出错误命令。 */
      if (level_command_overflowed) {
        continue;
      }

      /* 为结尾的 NUL 预留一个字节；超出容量后丢弃当前命令剩余内容。 */
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
