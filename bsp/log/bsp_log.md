# bsp_log

`bsp_log` 是基于 SEGGER RTT 的并发安全日志库。日志使用通道 0 输出，调用者传入字符串 `tag` 作为显示标签；tag 不注册、不复制，也不参与等级配置。

## 初始化

在调度器启动前调用一次：

```c
BSPLogInit();
osKernelInitialize();
MX_FREERTOS_Init();
osKernelStart();
```

`BSPLogInit()` 初始化 RTT，将终端通道 0 显式设为 `SEGGER_RTT_MODE_NO_BLOCK_SKIP`，配置数据通道和 Reserved 通道，并把全局等级设为 `BSP_LOG_DEFAULT_LEVEL`（默认 `INFO`）。日志层不创建或使用 mutex。初始化接口标记为 `NO_THREAD_SAFE_PROMISE`，只能由调度器启动前的单个线程调用；ISR 或调度器启动后调用返回 `STATE_ERR`。成功后重复初始化直接返回成功，不重置 RTT 中已有的数据或运行时等级；初始化失败可以在启动阶段重试。

## 等级和过滤

等级顺序为：

```text
TRACE < DEBUG < INFO < WARN < ERROR < FATAL
```

默认全局等级为 `INFO`，可以运行时修改：

```c
BSPLogSetLevel(BSP_LOG_LEVEL_DEBUG);
```

低于全局等级的日志在格式化前直接返回 `PENDING`，不会执行格式化或写入 RTT。这是正常的过滤结果，不会更新当前任务的最后错误。这里的 `PENDING` 表示本次日志跳过，不会加入队列或稍后补发。

```c
err_t result =
    BSPLogWrite(BSP_LOG_LEVEL_DEBUG, "motor", "speed=%d", speed);
if (result == PENDING) {
  // 日志被当前等级正常过滤
}
```

## 输出接口

```c
err_t BSPLogWrite(bsp_log_level_t level,
                  const char *tag,
                  const char *format,
                  ...);

err_t BSPLogVWrite(bsp_log_level_t level,
                   const char *tag,
                   const char *format,
                   va_list args);
```

示例：

```c
LOG_INFO("motor", "speed=%d", speed);
LOG_ERROR("can", "send failed: %d", status);
```

输出格式为：

```text
[INFO][motor] speed=1200\r\n
[ERROR][can] send failed: -15\r\n
```

`tag` 和 `format` 不能为 `NULL`，否则返回 `PTR_NULL`。允许使用空字符串 tag。

## 返回状态

日志接口直接复用 `component/comp_cmd.h` 中的公共 `err_t`，不再定义日志专用错误枚举或旧名称别名。实际使用的状态如下：

| 状态 | 数值 | 日志接口中的含义 |
| --- | --- | --- |
| `PENDING` | 1 | 等级过滤，正常跳过，不更新最后错误 |
| `OK` | 0 | 操作成功；写日志时表示整条记录已写入 RTT |
| `FAILED` | -1 | `snprintf/vsnprintf` 返回负值，格式化失败 |
| `INIT_ERR` | -2 | 尚未初始化，或 RTT 通道配置失败 |
| `ARG_ERR` | -3 | 日志等级非法 |
| `STATE_ERR` | -4 | 初始化或输出的调用上下文非法 |
| `SIZE_ERR` | -5 | 标签或正文过长，整条日志不提交 |
| `NOT_SUPPORT` | -7 | 编译时关闭日志后，直接调用初始化、输出或清屏接口 |
| `NO_BUFF` | -11 | RTT 空间不足，整条记录未写入 |
| `PTR_NULL` | -16 | 标签、格式串或等级输出指针为空 |

RTT 写入不等待，因此不会返回 `TIMEOUT`。日志关闭时 `LOG_*` 宏仍为 `((void)0)`，不求值参数；等级设置、等级查询和最后错误查询接口仍可使用。

## 并发规则

每次调用使用独立的 512 字节栈缓冲区，先完成标签、正文和 `\r\n` 的格式化，再调用一次 `SEGGER_RTT_Write()` 提交整条记录。RTT 内置的 `SEGGER_RTT_LOCK/UNLOCK` 负责保护环形缓冲区；日志层不加锁，也不在关闭中断的临界区内执行 `snprintf/vsnprintf`。

通道 0 使用 `NO_BLOCK_SKIP`：空间足够时整条写入，不足时整条跳过并返回 `NO_BUFF`，不等待主机读取。请勿在初始化后将该通道改成阻塞或截断模式。清屏也通过一次 RTT 写入提交。

512 字节容量包括前缀、正文、两字节换行及字符串结束符，最多提交 511 字节。超长标签或正文返回 `SIZE_ERR`，不产生半条日志。`vsnprintf` 保留标准格式化及浮点支持。

独立行缓冲区会占用调用任务的栈；栈预算还必须包括 libc 浮点格式化及其调用链。DR16 任务保留 2048 字节栈；其他只有 512 字节栈的任务如需使用格式化日志，应先增加栈并测量运行时余量。日志层不分配堆内存，但 newlib 浮点格式化本身可能分配内存。

为支持并发 `vsnprintf`（尤其是 `%f`），`FreeRTOSConfig.h` 启用 `configUSE_NEWLIB_REENTRANT=1`，由内核为每个任务维护独立的 newlib 状态，增加每个任务 TCB 的内存开销。`sysmem.c` 提供 `__malloc_lock/unlock`，只在 newlib 分配/释放内存期间嵌套挂起调度器，不使用 mutex，不关闭中断，也不包住整个格式化过程。newlib 堆接口仅用于线程上下文。

普通任务和调度器启动前可以调用格式化日志。ISR、关闭中断、使用 `BASEPRI` 的临界区以及调度器挂起期间调用会返回 `STATE_ERR`。

`NO_THREAD_SAFE_PROMISE` 是接口注解，只标记必须由调用者保证单线程调用的初始化接口。日志写入和等级读写支持并发。RTT 锁范围内没有格式化操作；本项目 Cortex-M4 的 RTT 默认锁使用 `BASEPRI=0x20` 屏蔽相关中断及任务切换。

## 每任务错误状态

FreeRTOS 配置中的 `configNUM_THREAD_LOCAL_STORAGE_POINTERS` 设为 1，槽位 0 专供日志库保存当前任务的最后错误。任务 A 和任务 B 的错误状态互不覆盖：

```c
err_t result = LOG_ERROR("motor", "failed");
if (result < 0 && BSPLogGetLastError() != OK) {
  // 只读取当前任务的错误状态
}
```

规则如下：

- `PENDING` 不更新错误状态。
- 初始化、写入、清屏和设置等级返回 `OK` 时清除当前任务错误；成功查询等级或最后错误不改变错误状态。
- 其他失败状态写入当前任务 TLS。
- 调度器启动前使用独立的启动阶段错误状态。
- ISR 不读写任务 TLS。

## 清屏和工具接口

```c
err_t BSPLogClear(void);
err_t BSPLogSetLevel(bsp_log_level_t level);
err_t BSPLogGetLevel(bsp_log_level_t *level);
err_t BSPLogGetLastError(void);
```

日志库不包含 `SEGGER_RTT_Read()` 命令轮询、动态命令解析、模块枚举或 Flash 持久化配置。等级配置只通过 `BSPLogSetLevel()` 在运行时设置，重启后恢复默认值。

`Float2StrN()` 仍然是独立的浮点字符串工具，不参与日志等级、并发锁或错误状态。其返回值表示字符长度，因此保留 `int` 类型：失败返回 -1，成功返回完整结果所需字符数，不含结束符；返回值大于等于容量表示发生截断。

内部使用 `round()` 将数值量化到 0.001，再由整数格式的 `snprintf()` 输出，复用标准库的数字转换、长度计算和截断处理。半整数向远离零的方向取整，负零和取整后为零的小负数输出 `0.000`；NaN、无穷大以及量化后超出 `[-2^63, 2^63)` 的数值返回 -1。调用上下文需满足 libc 的可重入要求。

项目使用的 newlib-nano 未启用 `long long` 格式支持，因此整数部分按十亿拆分为两个 32 位段，使用 `PRIu32` 输出，兼容大数值且无需手工逐位转换。
