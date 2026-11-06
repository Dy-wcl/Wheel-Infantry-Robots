/**
 * @file bsp/log/bsp_log.h
 * @brief 声明 RTT 分级日志接口和有限浮点数转换接口。
 *
 * @author xiaoshuaijie
 * @date 2026-11-04
 * @version 1.0.1
 * @since 1.0.1
 *@copyright Copyright(c) 2026, xiaoshuaijie
 */

/**
 * @defgroup bsp_log RTT 日志
 * @since 1.0.1
 * @brief 提供带标签的 RTT 记录和任务本地错误查询。
 * @details 通道 0 用于文本，通道 1 用于 JScope，通道 2 用于预留数据。
 *
 * @par 状态保存约定
 * 除查询接口另有说明外，操作结果保存在启动状态或当前任务状态中；OK 清除旧错误，
 * PENDING 保留旧状态。ISR 调用不更新状态，FreeRTOS TLS 槽位 0 由本模块独占。
 * @{
 */

#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "comp_cmd.h"

/* 编译开关与 RTT 通道配置 */

#ifndef BSP_LOG_ENABLED
#define BSP_LOG_ENABLED   \
  0 /**< 编译开关：0 \
       关闭，非零启用；由$<$<CONFIG:Debug>:BSP_LOG_ENABLED>定义。 */
#endif

#define BSP_LOG_CHANNEL_TERMINAL 0u /**< 上行通道：终端日志。 */
#define BSP_LOG_CHANNEL_DATA 1u     /**< 上行通道：JScope 数据。 */
#define BSP_LOG_CHANNEL_RESERVED 2u /**< 上行通道：预留输出。 */
#define BSP_LOG_CHANNEL_COMMAND 0u  /**< 下行通道：日志等级命令。 */

/* 缓冲区容量与默认等级 */

#define BSP_LOG_COMMAND_READ_BUFFER_SIZE \
  8u /**< 单次轮询从 RTT 下行通道读取的临时块大小（字节）。 */
#define BSP_LOG_COMMAND_BUFFER_SIZE \
  32u /**< 命令缓冲区容量（字节，包含 NUL）。 */
#define BSP_LOG_LINE_BUFFER_SIZE \
  512u /**< 单次行缓冲区容量（字节，包含 NUL）。 */
#define BSP_LOG_DATA_BUFFER_SIZE \
  1024u /**< 静态 JScope 上行缓冲区容量（字节）。 */
#define BSP_LOG_RESERVED_BUFFER_SIZE \
  256u /**< 静态预留上行缓冲区容量（字节）。 */
#define BSP_LOG_DEFAULT_LEVEL BSP_LOG_LEVEL_INFO /**< 初始最低等级：INFO。 */

/* Cortex-M 上下文判定值 */

#define BSP_LOG_IPSR_THREAD_MODE 0U /**< IPSR：线程模式。 */
#define BSP_LOG_PRIMASK_UNMASKED 0U /**< PRIMASK：未屏蔽中断。 */
#define BSP_LOG_BASEPRI_UNMASKED 0U /**< BASEPRI：无优先级屏蔽。 */

/* 日志等级与接口标记 */

/**
 * @brief 按 TRACE（0）至 FATAL（5）排列日志记录等级。
 * @details 为记录和共享最低等级过滤器提供等级定义；COUNT 是哨兵值。
 */
typedef enum {
  BSP_LOG_LEVEL_TRACE = 0, /**< 跟踪（0）。 */
  BSP_LOG_LEVEL_DEBUG,     /**< 调试（1）。 */
  BSP_LOG_LEVEL_INFO,      /**< 信息（2）。 */
  BSP_LOG_LEVEL_WARN,      /**< 警告（3）。 */
  BSP_LOG_LEVEL_ERROR,     /**< 错误（4）。 */
  BSP_LOG_LEVEL_FATAL,     /**< 致命（5），不会停止运行。 */
  BSP_LOG_LEVEL_COUNT      /**< 等级数量，不是有效日志等级。 */
} bsp_log_level_t;

/* 初始化与日志输出 */

/**
 * @brief 在调度器启动前初始化 RTT 终端、数据及预留通道。
 *
 * @return 初始化状态，具体取值如下。
 * @retval OK 初始化成功或重复成功初始化。
 * @retval STATE_ERR 不在启动阶段线程上下文。
 * @retval INIT_ERR RTT 配置失败。
 * @retval NOT_SUPPORT 日志已关闭。
 * @note 将终端设置为非阻塞整条跳过模式，并将等级设置为 INFO。
 *       重复成功调用会保留缓冲数据和已配置等级。
 *       初始化失败时可在调度器启动前重试。
 *       结果写入启动状态或任务状态；ISR 结果不会更新状态。
 * @warning 要求在调度器启动前调用此函数完成初始化。
 */
err_t BSPLogInit(void);

/**
 * @brief 按指定等级格式化并提交带标签的日志记录。
 *
 * @param[in] level 日志等级，范围为 TRACE（0）至 FATAL（5）。
 * @param[in] tag 借用的非空、NUL 结尾标签；允许为空串，在调用期间有效。
 * @param[in] format 借用的非空 printf 格式串，在调用期间有效。
 * @param[in] ... 按 format 要求的顺序和类型提供格式化参数。
 * @return 日志提交状态，具体取值如下。
 * @retval OK 记录提交成功。
 * @retval PENDING 记录被等级过滤。
 * @retval ARG_ERR 日志等级非法。
 * @retval PTR_NULL tag 或 format 为空。
 * @retval INIT_ERR 日志尚未初始化。
 * @retval STATE_ERR 上下文被拒绝。
 * @retval FAILED 格式化失败。
 * @retval SIZE_ERR 记录过长。
 * @retval NO_BUFF RTT 空间不足。
 * @retval NOT_SUPPORT 日志已关闭。
 * @note 使用私有栈缓冲区，并通过 RTT 内部锁执行一次写入。
 *       记录格式为“[LEVEL][tag] 正文”，末尾为 CRLF，最多提交 511 字节。
 *       过滤在参数和初始化检查之后、上下文检查之前执行。
 *       PENDING 直接跳过记录，不入队且不改变已保存错误状态。
 *       其他结果更新启动状态或当前任务状态；ISR 结果不会更新状态。
 * @warning 未被过滤的写入拒绝 ISR、中断屏蔽和调度器挂起上下文。
 *          512 字节行缓冲区及 libc 格式化需要调用方提供足够栈空间。
 *          格式化参数必须与 format 匹配；libc 格式化可能分配内存。
 *          初始化后通道 0 必须保持 SEGGER_RTT_MODE_NO_BLOCK_SKIP 模式。
 */
err_t BSPLogWrite(bsp_log_level_t level, const char* tag, const char* format,
                  ...);

/**
 * @brief 使用已有参数列表格式化并提交带标签的日志记录。
 *
 * @param[in] level 日志等级，范围为 TRACE（0）至 FATAL（5）。
 * @param[in] tag 借用的非空、NUL 结尾标签；允许为空串，在调用期间有效。
 * @param[in] format 借用的非空 printf 格式串，在调用期间有效。
 * @param[in,out] args 格式化时消耗的参数列表；如需复用请使用 va_copy。
 * @return 与 BSPLogWrite() 相同，详见其返回值说明。
 * @note 输出格式、过滤顺序、栈需求和状态保存规则与 BSPLogWrite() 相同。
 *       函数返回后不保留 tag、format 或 args。
 * @warning 未被过滤的写入拒绝 ISR、中断屏蔽和调度器挂起上下文；通道 0 必须保持
 *          SEGGER_RTT_MODE_NO_BLOCK_SKIP 模式。
 */
err_t BSPLogVWrite(bsp_log_level_t level, const char* tag, const char* format,
                   va_list args);

/* 等级控制与状态查询 */

/**
 * @brief 设置后续日志记录使用的共享最低等级。
 *
 * @param[in] level 最低等级，范围为 TRACE（0）至 FATAL（5）。
 * @return 等级设置状态。
 * @retval OK 等级更新成功。
 * @retval ARG_ERR 等级超出范围。
 * @note 使用原子写入，即使关闭日志也可配置；非法输入保留原等级。
 */
err_t BSPLogSetLevel(bsp_log_level_t level);

/**
 * @brief 将共享最低等级读取到调用方提供的存储位置。
 *
 * @param[out] level 借用的非空输出指针，接收 TRACE 至 FATAL 范围内的等级。
 * @return 等级读取状态。
 * @retval OK 读取成功。
 * @retval PTR_NULL 输出指针为空。
 * @note 使用原子读取，即使关闭日志也可查询。成功读取保留已保存错误；
 *       除 ISR 外，空指针错误会更新已保存状态。
 */
err_t BSPLogGetLevel(bsp_log_level_t* level);

/**
 * @brief 轮询 RTT 下行通道 0 并应用完整的日志等级命令。
 *
 * @return 命令处理状态，具体取值如下。
 * @retval OK 至少应用一条命令。
 * @retval PENDING 没有完整命令可用。
 * @retval ARG_ERR 完整命令非法。
 * @retval STATE_ERR 不在任务上下文。
 * @retval INIT_ERR 尚未调用 BSPLogInit()。
 * @retval NOT_SUPPORT 日志已关闭。
 * @note 支持 0 至 5 的原始等级字节；文本命令不区分大小写，以 CR 或 LF 结束。
 *       文本格式：`0` 至 `5`、`TRACE` 至 `FATAL`、`level=<name>`、
 *       `level <name>` 或 `log level <name>`。
 *       不完整输入保留到下次调用；过长行丢弃至下一个结束符。
 *       每次 RTT 读取块大小由 BSP_LOG_COMMAND_READ_BUFFER_SIZE 配置，默认
 *       为 8 字节；该值只影响单次读取的栈占用和吞吐折中，不限制命令长度。
 * @warning 应在 BSPLogInit() 后由单个线程周期调用。
 */
err_t BSPLogPollLevelCommand(void);

/**
 * @brief 读取已保存的启动阶段或当前任务日志状态，不修改该状态。
 *
 * @return 线程模式下返回已保存的 err_t 状态；ISR 中返回 STATE_ERR。
 * @note 调度器启动前读取启动状态，启动后读取任务本地存储槽 0。
 *       新任务初始状态为 OK；被过滤记录保留已保存状态。ISR 查询不访问 TLS。
 * @warning FreeRTOS TLS 槽 0 专供本模块使用，不得复用。
 */
err_t BSPLogGetLastError(void);

/* 终端控制 */

/**
 * @brief 通过 RTT 通道 0 提交终端清屏控制序列。
 *
 * @return 控制序列提交状态，具体取值如下。
 * @retval OK 清屏序列提交成功。
 * @retval INIT_ERR 日志尚未初始化。
 * @retval STATE_ERR 上下文被拒绝。
 * @retval NO_BUFF RTT 空间不足。
 * @retval NOT_SUPPORT 日志已关闭。
 * @note 写入两个空格和 RTT_CTRL_CLEAR，不发送末尾 NUL。
 *       控制序列由主机终端解释，不丢弃 RTT 缓冲数据。
 *       结果写入启动状态或任务状态；ISR 结果不会更新状态。
 * @warning 拒绝 ISR、中断屏蔽和调度器挂起上下文；通道 0 必须保持
 *          SEGGER_RTT_MODE_NO_BLOCK_SKIP 模式。
 */
err_t BSPLogClear(void);

/**
 * @name 日志快捷宏
 * @brief 使用固定等级调用 BSPLogWrite()。
 *
 * @param[in] tag 为非 NULL、NUL 结尾的标签，允许空串；format 为非 NULL 的
 * printf 格式串。 两者仅在调用期间借用，可变参数须与 format 匹配。 启用时返回
 * BSPLogWrite() 的结果；关闭时展开为 ((void)0)。
 * @note 启用时每个参数求值一次，即使记录随后被等级过滤。
 *       关闭时参数不求值，不输出日志，也不更新已保存状态。
 *       需要始终获取返回状态时，请直接调用 BSPLogWrite()。
 * @warning 启用时遵守 BSPLogWrite() 的上下文、栈空间及格式要求。
 * @{
 */
#if BSP_LOG_ENABLED
/** @brief 提交 TRACE 记录；详细信息。 */
#define LOG_TRACE(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_TRACE, (tag), (format), ##__VA_ARGS__)

/** @brief 提交 DEBUG 记录；调试信息。 */
#define LOG_DEBUG(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_DEBUG, (tag), (format), ##__VA_ARGS__)

/** @brief 提交 INFO 记录；正常运行信息。 */
#define LOG_INFO(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_INFO, (tag), (format), ##__VA_ARGS__)

/** @brief 提交 WARN 记录；警告信息。 */
#define LOG_WARN(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_WARN, (tag), (format), ##__VA_ARGS__)

/** @brief 提交 ERROR 记录；错误信息。 */
#define LOG_ERROR(tag, format, ...) \
  BSPLogWrite(BSP_LOG_LEVEL_ERROR, (tag), (format), ##__VA_ARGS__)

/** @brief 提交 FATAL 记录；严重错误。 */
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

/** @} */

/* 数值转换（独立于日志开关及错误状态） */

/**
 * @brief 将有限浮点数转换为固定三位小数的字符串。
 *
 * @param[out] str 借用的非空调用方缓冲区，最多接收 size 字节（包括 NUL）。
 * @param[in] size 缓冲区容量，单位为字节，至少为 1，包括 NUL 结束符空间。
 * @param[in] va 有限输入值，取整后的 va * 1000 位于 [-2^63, 2^63) 内。
 * @return 成功时返回完整结果字符数（不含 NUL）；返回值大于等于 size 表示截断。
 *         str 为 NULL、size 为 0、输入非有限或越界、snprintf 失败时返回 -1。
 * @note 以 0.001 精度取整，中点向远离零方向舍入，取整后的零输出为 0.000。
 *       size 大于 0 且 str 非 NULL 时，即使失败，缓冲区也以 NUL 结尾。
 *       整数部分按 32 位十进制分组以支持
 * newlib-nano；不读取或修改日志错误状态。
 * @warning 要求 str 指向至少 size 字节的可写缓冲区，调用上下文满足 libc
 *          格式化要求。
 */
int Float2StrN(char* str, size_t size, float va);

/** @} */

#endif /* _BSP_LOG_H */
