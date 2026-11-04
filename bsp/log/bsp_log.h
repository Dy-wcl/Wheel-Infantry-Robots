#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "comp_cmd.h"

#ifndef BSP_LOG_ENABLED
#define BSP_LOG_ENABLED 0
#endif

#define BSP_LOG_CHANNEL_TERMINAL 0u
#define BSP_LOG_CHANNEL_DATA 1u
#define BSP_LOG_CHANNEL_RESERVED 2u
#define BSP_LOG_LINE_BUFFER_SIZE 512u
#define BSP_LOG_DATA_BUFFER_SIZE 1024u
#define BSP_LOG_RESERVED_BUFFER_SIZE 256u
#define BSP_LOG_DEFAULT_LEVEL BSP_LOG_LEVEL_INFO
#define BSP_LOG_IPSR_THREAD_MODE 0U
#define BSP_LOG_PRIMASK_UNMASKED 0U
#define BSP_LOG_BASEPRI_UNMASKED 0U

typedef enum {
  BSP_LOG_LEVEL_TRACE = 0,
  BSP_LOG_LEVEL_DEBUG,
  BSP_LOG_LEVEL_INFO,
  BSP_LOG_LEVEL_WARN,
  BSP_LOG_LEVEL_ERROR,
  BSP_LOG_LEVEL_FATAL,
  BSP_LOG_LEVEL_COUNT
} bsp_log_level_t;

#ifndef NO_THREAD_SAFE_PROMISE
#define NO_THREAD_SAFE_PROMISE
#endif

/* 仅在调度器启动前由单线程初始化；成功后重复调用不会清空 RTT。 */
NO_THREAD_SAFE_PROMISE err_t BSPLogInit(void);

/* 独立栈缓冲区格式化，RTT 内置锁提交；等级过滤返回 PENDING，不排队。 */
err_t BSPLogWrite(bsp_log_level_t level, const char* tag,
                  const char* format, ...);
err_t BSPLogVWrite(bsp_log_level_t level, const char* tag,
                   const char* format, va_list args);

err_t BSPLogSetLevel(bsp_log_level_t level);
err_t BSPLogGetLevel(bsp_log_level_t* level);
err_t BSPLogGetLastError(void);
err_t BSPLogClear(void);

#if BSP_LOG_ENABLED
#define LOG_TRACE(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_TRACE, (tag), (format), ##__VA_ARGS__)
#define LOG_DEBUG(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_DEBUG, (tag), (format), ##__VA_ARGS__)
#define LOG_INFO(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_INFO, (tag), (format), ##__VA_ARGS__)
#define LOG_WARN(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_WARN, (tag), (format), ##__VA_ARGS__)
#define LOG_ERROR(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_ERROR, (tag), (format), ##__VA_ARGS__)
#define LOG_FATAL(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_FATAL, (tag), (format), ##__VA_ARGS__)
#else
#define LOG_TRACE(tag, format, ...) ((void)0)
#define LOG_DEBUG(tag, format, ...) ((void)0)
#define LOG_INFO(tag, format, ...) ((void)0)
#define LOG_WARN(tag, format, ...) ((void)0)
#define LOG_ERROR(tag, format, ...) ((void)0)
#define LOG_FATAL(tag, format, ...) ((void)0)
#endif

int Float2StrN(char* str, size_t size, float va);

#endif
