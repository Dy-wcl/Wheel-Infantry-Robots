/**
 * @file bsp_log.c
 * @brief 基于 SEGGER RTT 的分级日志及浮点字符串转换工具。
 *
 * 日志流程：检查参数和等级 -> 在本次调用的栈上格式化 -> 一次提交 RTT。
 * 格式化缓冲区不在任务间共享；环形缓冲区由 RTT 内置锁保护，日志层不使用
 * mutex。通道 0 采用非阻塞整条跳过模式，主机未连接时也不会等待主机读取。
 *
 * 日志状态统一使用 comp_cmd.h 的 err_t；每个任务的最近状态保存在 TLS 槽 0。
 * 浮点格式化依赖 FreeRTOS 的独立 newlib 状态及 sysmem.c 的分配器保护钩子。
 */
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

/* 此槽位只编码 err_t 数值，不分配对象，也不能由其他模块复用。 */
#define BSP_LOG_TLS_ERROR_INDEX 0

#if configNUM_THREAD_LOCAL_STORAGE_POINTERS <= BSP_LOG_TLS_ERROR_INDEX
#error "bsp_log requires a reserved FreeRTOS TLS slot"
#endif

#if BSP_LOG_ENABLED
/* RTT 持有缓冲区地址直至程序结束，因此数据通道必须使用静态存储。
 * 它们不是格式化行缓冲区，不会造成多个日志调用之间的内容串写。 */
static char data_buffer[BSP_LOG_DATA_BUFFER_SIZE];
static char reserved_buffer[BSP_LOG_RESERVED_BUFFER_SIZE];
#endif

/* 调度器启动前还没有当前任务，使用独立的启动状态，避免污染首个任务。 */
static err_t startup_error = OK;
/* 等级是运行时共享配置，通过原子存取保证任务间访问一致。 */
static _Atomic uint8_t current_level = BSP_LOG_DEFAULT_LEVEL;

#if BSP_LOG_ENABLED
/* 只在调度器启动前的单线程初始化阶段写入，运行期间只读取。 */
static bool log_initialized;

/** @brief 将已经校验的日志等级转换成输出前缀中的名称。 */
static const char *BSPLogLevelName(bsp_log_level_t level) {
  static const char *const names[] = {"TRACE", "DEBUG", "INFO",
                                      "WARN",  "ERROR", "FATAL"};
  _Static_assert(ARRAY_LEN(names) == BSP_LOG_LEVEL_COUNT,
                 "Each log level must have a name");
  return ((unsigned)level < ARRAY_LEN(names)) ? names[level] : "UNKNOWN";
}
#endif

/** @brief IPSR 为 0 表示线程模式；非 0 表示正在执行异常或中断处理。 */
static bool BSPLogInThreadMode(void) {
  return __get_IPSR() == BSP_LOG_IPSR_THREAD_MODE;
}

#if BSP_LOG_ENABLED
/**
 * @brief 判断当前上下文是否允许执行格式化和日志提交。
 *
 * 允许调度器启动前及正常任务上下文；拒绝 ISR、调度器挂起以及中断被屏蔽的
 * 临界区。即使 RTT 提交不等待，libc 格式化仍可能分配内存，不能放入上述上下文。
 * 先判断 IPSR，避免从 ISR 调用调度器查询接口。
 */
static bool BSPLogContextAllowsWrite(void) {
  BaseType_t scheduler_state;

  if (!BSPLogInThreadMode()) {
    return false;
  }
  /* 读取一次调度器状态，后面的判断统一使用这次查询结果：
   * - NOT_STARTED：调度器尚未启动，允许启动阶段输出日志；此时错误状态
   *   保存在 startup_error 中，而不是当前任务的 TLS。
   * - RUNNING：调度器正常运行，允许普通任务输出日志。
   * - SUSPENDED：调度器已挂起，任务暂时不能切换，但中断仍可能响应。
   *   本接口拒绝在此期间格式化，避免延长不可调度区间；格式化可能触发
   *   libc 内存分配，所以不能只根据 RTT 写入不阻塞就允许调用。 */
  scheduler_state = xTaskGetSchedulerState();

  /* 三组条件必须同时成立，任意一组不满足就返回 false。 */
  return (scheduler_state == taskSCHEDULER_NOT_STARTED ||
          scheduler_state == taskSCHEDULER_RUNNING) &&
         /* PRIMASK 为 0：没有通过 PRIMASK 全局屏蔽可配置优先级异常。
          * PRIMASK 为 1 通常表示代码主动关闭了这类中断；NMI 和 HardFault
          * 不受该位屏蔽。此处要求为 0，避免在关中断期间执行日志格式化。 */
         (__get_PRIMASK() == BSP_LOG_PRIMASK_UNMASKED) &&
         /* BASEPRI 为 0：没有设置中断优先级屏蔽阈值。
          * 非 0 时，优先级编码数值大于或等于该阈值的异常会被屏蔽；
          * 本项目 FreeRTOS 的临界区通过 BASEPRI 实现。因此即使 PRIMASK
          * 为 0，也可能仍处于临界区，必须继续检查 BASEPRI 是否为 0。 */
         (__get_BASEPRI() == BSP_LOG_BASEPRI_UNMASKED);
}
#endif

/**
 * @brief 保存调用结果，并原样返回，供各接口统一收尾。
 * @param result 本次操作状态；OK 清除旧错误，负值记录失败原因。
 *
 * PENDING 在本模块表示日志被等级过滤，不是错误，也不覆盖先前的错误。
 * ISR 只返回结果，不读写被中断任务的 TLS。启动阶段写 startup_error；
 * 调度器启动后写当前任务槽位，因此其他任务的日志不会覆盖本任务的状态。
 */
static err_t BSPLogRecordResult(err_t result) {
  if (result == PENDING || !BSPLogInThreadMode()) {
    return result;
  }
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    startup_error = result;
  } else {
    /* NULL 表示当前任务；经 intptr_t 编码的状态只保存为指针值，不解引用。
     * 新任务的 TLS 初值为 NULL，解码后正好是 OK（0）。 */
    vTaskSetThreadLocalStoragePointer(NULL, BSP_LOG_TLS_ERROR_INDEX,
                                      (void *)(intptr_t)result);
  }
  return result;
}

/** @brief 校验等级；先转为无符号值，使负值也会被判为非法参数。 */
static err_t BSPLogValidateLevel(bsp_log_level_t level) {
  return ((unsigned)level < BSP_LOG_LEVEL_COUNT) ? OK : ARG_ERR;
}

#if BSP_LOG_ENABLED
/**
 * @brief 将完整记录一次提交给 RTT；data 和 size 由内部调用者保证有效。
 *
 * RTT 在复制环形缓冲区时自行加锁。通道 0 的 NO_BLOCK_SKIP 模式保证空间
 * 不足时整条放弃，返回 NO_BUFF；本函数不重试、不分段，也不额外加锁。
 * size 是实际字节数，因此正文即使包含通过 %c 写入的 NUL 也能完整提交。
 */
static err_t BSPLogWriteRaw(const char *data, size_t size) {
  const unsigned written =
      SEGGER_RTT_Write(BSP_LOG_CHANNEL_TERMINAL, data, (unsigned)size);
  return (written == size) ? OK : NO_BUFF;
}
#endif

/**
 * @brief 在调度器启动前初始化日志传输通道。
 * @return OK：完成或此前已成功；STATE_ERR：调用上下文非法；
 *         INIT_ERR：RTT 通道配置失败；NOT_SUPPORT：编译时关闭日志。
 *
 * 该接口不承诺并发初始化。只有全部通道配置成功才设置 log_initialized；
 * 失败时可在启动阶段重试。成功后的重复调用不重置等级，也不丢弃待读取日志。
 */
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

  /* RTT_Init 会重置 RTT 控制块，必须放在重复初始化检查之后。 */
  SEGGER_RTT_Init();
  /* 显式保证终端为非阻塞整条跳过模式，避免依赖 RTT 库的默认配置。 */
  if (SEGGER_RTT_SetFlagsUpBuffer(BSP_LOG_CHANNEL_TERMINAL,
                                  SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0) {
    return BSPLogRecordResult(INIT_ERR);
  }
  /* 通道 1 供 JScope 数据使用；名称保留现有的数据格式描述。 */
  result = (SEGGER_RTT_ConfigUpBuffer(BSP_LOG_CHANNEL_DATA, "JScope_t4f4f4f4",
                                      data_buffer, sizeof(data_buffer),
                                      SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0)
               ? INIT_ERR
               : OK;
  if (result == OK) {
    /* 通道 2 预留，不与通道 0 的文本日志共用缓冲区。 */
    result = (SEGGER_RTT_ConfigUpBuffer(
                  BSP_LOG_CHANNEL_RESERVED, "Reserved", reserved_buffer,
                  sizeof(reserved_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0)
                 ? INIT_ERR
                 : OK;
  }
  if (result == OK) {
    /* 初始化完成后才发布默认等级和可用状态，避免使用半初始化的通道。 */
    atomic_store_explicit(&current_level, BSP_LOG_DEFAULT_LEVEL,
                          memory_order_release);
    log_initialized = true;
  }
  return BSPLogRecordResult(result);
#else
  return BSPLogRecordResult(NOT_SUPPORT);
#endif
}

/** @brief 设置全局最低输出等级；非法等级返回 ARG_ERR，不改变旧配置。 */
err_t BSPLogSetLevel(bsp_log_level_t level) {
  const err_t valid = BSPLogValidateLevel(level);
  if (valid != OK) {
    return BSPLogRecordResult(valid);
  }
  atomic_store_explicit(&current_level, (uint8_t)level, memory_order_release);
  return BSPLogRecordResult(OK);
}

/**
 * @brief 读取全局最低输出等级。
 * @param level 接收等级的地址；NULL 返回 PTR_NULL 并记录到本任务状态。
 * @note 查询成功不会清除先前错误，便于调用者先查配置再查错误。
 */
err_t BSPLogGetLevel(bsp_log_level_t *level) {
  if (level == NULL) {
    return BSPLogRecordResult(PTR_NULL);
  }
  *level = (bsp_log_level_t)atomic_load_explicit(&current_level,
                                                 memory_order_acquire);
  return OK;
}

/**
 * @brief 非破坏性读取当前任务最近一次记录的日志状态。
 *
 * 启动阶段返回独立的 startup_error；ISR 返回 STATE_ERR，不访问任务 TLS。
 * 查询本身不修改状态，其他任务产生的成功或失败不会影响本任务的查询结果。
 */
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

/**
 * @brief 输出带等级和标签的格式化日志，供需要转发 va_list 的封装层调用。
 * @param level 日志等级，范围为 TRACE 至 FATAL。
 * @param tag 标签字符串，允许空串，不允许 NULL；只在本次调用期间读取。
 * @param format printf 风格格式串，不允许 NULL。
 * @param args 与格式串匹配的参数列表；调用后如需再次使用，由调用者先 va_copy。
 * @return OK 已提交；PENDING 已过滤；ARG_ERR 非法等级；PTR_NULL 空指针；
 *         INIT_ERR 未初始化；STATE_ERR 非法上下文；FAILED 格式化失败；
 *         SIZE_ERR 整条记录过长；NO_BUFF RTT 空间不足；NOT_SUPPORT 日志关闭。
 */
err_t BSPLogVWrite(bsp_log_level_t level, const char *tag, const char *format,
                   va_list args) {
#if BSP_LOG_ENABLED
  static const char line_end[] = "\r\n";
  /* 每次调用独占缓冲区：任务切换不会互相覆盖，格式化无需 mutex。
   * 此数组占用调用任务栈；RTT 内置锁只保护最后的传输复制。 */
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
  /* 等级不足时直接结束，不格式化、不写 RTT，也不覆盖本任务的旧错误。
   * PENDING 仅表示本次跳过，本模块没有待发送队列或后续补发机制。 */
  if (level < (bsp_log_level_t)atomic_load_explicit(&current_level,
                                                    memory_order_acquire)) {
    return PENDING;
  }
  if (!BSPLogContextAllowsWrite()) {
    return BSPLogRecordResult(STATE_ERR);
  }

  /* snprintf 返回本来需要的长度，不是实际写入长度；超长前缀必须拒绝。
   * 即使正文为空，也至少要留下 CR、LF、NUL 三个字节。 */
  prefix_size = snprintf(line_buffer, sizeof(line_buffer), "[%s][%s] ",
                         BSPLogLevelName(level), tag);
  if (prefix_size < 0) {
    return BSPLogRecordResult(FAILED);
  }
  if ((size_t)prefix_size > sizeof(line_buffer) - sizeof(line_end)) {
    return BSPLogRecordResult(SIZE_ERR);
  }
  /* 先预留 CRLF，剩余容量交给 vsnprintf，其中还包含它写入的 NUL。 */
  body_capacity =
      sizeof(line_buffer) - (size_t)prefix_size - (sizeof(line_end) - 1u);
  body_size = vsnprintf(line_buffer + prefix_size, body_capacity, format, args);
  if (body_size < 0) {
    return BSPLogRecordResult(FAILED);
  }
  if ((size_t)body_size >= body_capacity) {
    /* 不发送已被 vsnprintf 截断的半条内容，避免误读诊断信息。 */
    return BSPLogRecordResult(SIZE_ERR);
  }
  /* 正文成功后用 CRLF 覆盖其结束符，再补 NUL；NUL 不计入 RTT 发送长度。 */
  total_size = (size_t)prefix_size + (size_t)body_size;
  memcpy(line_buffer + total_size, line_end, sizeof(line_end));
  total_size += sizeof(line_end) - 1u;

  return BSPLogRecordResult(BSPLogWriteRaw(line_buffer, total_size));
#else
  /* 直接调用禁用的接口仍给出明确状态；LOG_* 宏则完全不求值参数。 */
  RM_UNUSED(level);
  RM_UNUSED(tag);
  RM_UNUSED(format);
  RM_UNUSED(args);
  return BSPLogRecordResult(NOT_SUPPORT);
#endif
}

/** @brief 普通可变参数入口；正确配对 va_start/va_end，状态由 VWrite 记录。 */
err_t BSPLogWrite(bsp_log_level_t level, const char *tag, const char *format,
                  ...) {
  err_t result;
  va_list args;
  va_start(args, format);
  result = BSPLogVWrite(level, tag, format, args);
  va_end(args);
  return result;
}

/**
 * @brief 向终端提交 RTT 清屏控制序列，使用与普通日志相同的上下文限制。
 * @note 清屏是交给主机终端解释的控制字符，不会清空 RTT 的待读取缓冲区。
 *       不包含末尾 NUL；空间不足返回 NO_BUFF，不阻塞等待。
 */
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

/**
 * @brief 将有限浮点数转换为固定三位小数字符串，独立于日志状态系统。
 * @param str 调用者提供的输出缓冲区。
 * @param size 缓冲区字节容量，必须包含结束符的位置。
 * @param va 待转换数值，不接受 NaN 和无穷大。
 * @return 完整结果所需的字符数，不含 NUL；参数非法或数值不支持时返回 -1。
 *         返回值大于等于 size 表示输出被截断，缓冲区仍以 NUL 结束。
 * @note 此函数返回长度，因此保留 int 类型，不改为 err_t；不读写日志 TLS。
 *       内部复用标准库格式化，调用上下文应满足 libc 的可重入要求。
 */
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

  /* round 对半整数向远离零的方向取整，保留原有的三位小数取整规则。
   * 不直接使用 %.3f，避免受其临界舍入规则和负零输出影响。 */
  rounded = round((double)va * 1000.0);

  /* double 无法精确表示 INT64_MAX，转成 double 后会变为 2^63。
   * 用精确的二进制常量明确检查原先的有符号整数范围，且不进行有风险的
   * double -> int64_t 转换；通过检查后，绝对值可安全存入 uint64_t。 */
  if (rounded < -0x1p63 || rounded >= 0x1p63) {
    str[0] = '\0';
    return -1;
  }

  /* 只对严格负值输出负号，使 -0.0 和取整后为零的小负数仍输出 0.000。
   * 项目链接的 newlib-nano 不支持 long long 格式，因此把整数按十亿拆成
   * 两个 uint32_t 段：高段正常输出，存在高段时低段补齐九位，小数补齐三位。
   * snprintf 统一负责十进制转换、截断、NUL 终止和所需长度的计算。 */
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
