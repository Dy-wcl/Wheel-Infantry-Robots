#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include <stdarg.h>
#include <stdio.h>

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

#define BSP_LOG_CHANNEL_TERMINAL 0u
#define BSP_LOG_CHANNEL_DATA 1u
#define BSP_LOG_CHANNEL_RESERVED 2u

/* Virtual terminals are multiplexed inside RTT channel 0. */
#define BSP_LOG_TERMINAL_DEFAULT 0u
#define BSP_LOG_TERMINAL_INFO 1u
#define BSP_LOG_TERMINAL_WARNING 2u
#define BSP_LOG_TERMINAL_ERROR 3u

/* Debug builds enable logs by default. Release can be overridden explicitly. */
#if defined(MCU_DEBUG_BUILD)
#define BSP_LOG_ENABLED 1
#else
#define BSP_LOG_ENABLED 0
#endif

/**
 * @brief 初始化 RTT 日志系统和默认上行通道。
 * @note 启动阶段、调度器启动前调用；内部使用静态分配的互斥锁。
 * @return 0 表示成功，负值表示某个通道或终端配置失败。
 */
int BSPLogInit(void);

/**
 * @brief 获取最近一次初始化或终端选择操作的错误码。
 */
int BSPLogGetLastError(void);

/**
 * @brief 配置 RTT 上行通道。
 */
int BSPLogConfigUpBuffer(unsigned index, const char* name, void* buffer,
                         unsigned size, unsigned flags);

/**
 * @brief 配置 RTT 下行通道。
 */
int BSPLogConfigDownBuffer(unsigned index, const char* name, void* buffer,
                           unsigned size, unsigned flags);

/**
 * @brief 选择通道 0 内的 RTT 虚拟终端，终端编号范围由 SEGGER RTT 实现定义。
 */
int BSPLogSelectTerminal(unsigned char terminal_id);

/**
 * @brief 向指定 RTT 上行通道写入原始数据。
 * @return 实际写入的字节数，失败返回负值。
 */
int BSPLogWriteChannel(unsigned channel, const void* data, unsigned size);

/**
 * @brief 将一条格式化日志原子地写入指定 RTT 虚拟终端。
 * @note BSPLogInit() 后可在启动阶段或任务中调用。共享静态行缓冲区由互斥锁
 *       保护；任务竞争时会等待。不支持 ISR、关中断/临界区或调度器挂起期间调用。
 *       vsnprintf()（尤其浮点格式化）的调用栈仍需由任务栈承担。
 * @return SEGGER_RTT_TerminalOut() 的返回值，失败返回负值。
 */
int BSPLogPrintfTerminal(unsigned char terminal_id, const char* type,
                         const char* color, const char* format, ...);

/**
 * @brief 日志功能原型，供 LOG、LOGINFO、LOGWARNING、LOGERROR 使用。
 */
#if BSP_LOG_ENABLED
#define LOG_PROTO(terminal, type, color, format, ...) \
  BSPLogPrintfTerminal((terminal), (type), (color), (format), ##__VA_ARGS__)
#else
#define LOG_PROTO(terminal, type, color, format, ...) \
  do {                                                \
    (void)sizeof(terminal);                           \
    (void)sizeof(type);                               \
    (void)sizeof(color);                              \
    (void)sizeof(format);                             \
  } while (0)
#endif

/*----------------------------------------日志输出接口-------------------------------------------------*/

#if BSP_LOG_ENABLED
#define LOG_CLEAR() \
  SEGGER_RTT_TerminalOut(BSP_LOG_TERMINAL_DEFAULT, "  " RTT_CTRL_CLEAR)
#define LOG(format, ...) \
  LOG_PROTO(BSP_LOG_TERMINAL_DEFAULT, "", "", format, ##__VA_ARGS__)
#define LOGINFO(format, ...)                                         \
  LOG_PROTO(BSP_LOG_TERMINAL_INFO, "I:", RTT_CTRL_TEXT_BRIGHT_GREEN, \
            format, ##__VA_ARGS__)
#define LOGWARNING(format, ...)                                          \
  LOG_PROTO(BSP_LOG_TERMINAL_WARNING, "W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, \
            format, ##__VA_ARGS__)
#define LOGERROR(format, ...)                                       \
  LOG_PROTO(BSP_LOG_TERMINAL_ERROR, "E:", RTT_CTRL_TEXT_BRIGHT_RED, \
            format, ##__VA_ARGS__)
#else
#define LOG_CLEAR() \
  do {              \
  } while (0)
#define LOG(format, ...) \
  do {                   \
  } while (0)
#define LOGINFO(format, ...) \
  do {                       \
  } while (0)
#define LOGWARNING(format, ...) \
  do {                          \
  } while (0)
#define LOGERROR(format, ...) \
  do {                        \
  } while (0)
#endif

#define LOG_CHANNEL_WRITE(channel, data, size) \
  BSPLogWriteChannel((channel), (data), (size))

/**
 * @brief 通过 SEGGER RTT 打印格式化日志。
 */
int PrintLog(const char* fmt, ...);

/**
 * @brief 利用 sprintf() 将 float 转换为字符串。
 */
void Float2Str(char* str, float va);

#endif
