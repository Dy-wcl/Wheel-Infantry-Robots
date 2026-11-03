# bsp_log

`bsp_log` 是基于 SEGGER RTT 的日志模块。日志写入 RTT 上行通道 0，使用 RTT 的虚拟 Terminal 区分普通、信息、警告和错误日志。

## 启用条件

编译时定义 `BSP_LOG_ENABLED` 后，日志实现和日志宏才会启用。未定义该宏时：

- `LOG`、`LOGINFO`、`LOGWARNING`、`LOGERROR` 和 `LOG_CLEAR` 都是空操作；
- `BSPLogInit()`、`BSPLogSelectTerminal()` 和 `BSPLogPrintfTerminal()` 返回负值；
- `Float2StrN()` 不受该宏影响，仍然可用。

当前工程的 Debug 构建定义了 `BSP_LOG_ENABLED`。如果 Release 构建也需要日志，应在对应的构建配置中定义该宏。

## 初始化

启用日志时，应在启动调度器前调用一次 `BSPLogInit()`：

```c
int main(void) {
  // HAL、外设和时钟初始化...
  BSPLogInit() 
  

  // 创建并启动 RTOS 调度器...
}
```

初始化函数会完成以下操作：

1. 创建静态分配的 FreeRTOS 互斥锁；
2. 初始化 SEGGER RTT；
3. 将 RTT 通道 0 的默认虚拟 Terminal 设置为 0。

如果没有成功调用 `BSPLogInit()`，`BSPLogPrintfTerminal()` 会因为内部互斥锁未创建而返回 `-1`。`BSPLogGetLastError()` 返回最近一次 `BSPLogInit()` 或 `BSPLogSelectTerminal()` 更新的错误码，不是每条日志的独立状态。

## 日志宏

```c
LOG("system ready\r\n");
LOGINFO("motor=%d rpm=%d\r\n", motor_id, rpm);
LOGWARNING("temperature=%.1f\r\n", temperature);
LOGERROR("can error=%d\r\n", error_code);
LOG_CLEAR();
```

| 宏           | 虚拟 Terminal | 前缀 | 颜色   |
| ------------ | ------------: | ---- | ------ |
| `LOG`        |             0 | 无   | 无     |
| `LOGINFO`    |             1 | `I:` | 亮绿色 |
| `LOGWARNING` |             2 | `W:` | 亮黄色 |
| `LOGERROR`   |             3 | `E:` | 亮红色 |

`LOGINFO`、`LOGWARNING` 和 `LOGERROR` 会自动在日志前添加两个空格、颜色和级别前缀，并在末尾添加 `\r\n` 和 RTT 颜色复位控制码。调用者不需要再手动添加换行；如果格式字符串本身包含换行，输出中会保留它。

`LOG_CLEAR()` 直接向 Terminal 0 写入 RTT 清屏控制码。它不经过格式化，也不使用 `bsp_log.c` 的共享行缓冲区。

## 直接调用格式化接口

需要动态选择 Terminal 或自定义前缀、颜色时，可以直接调用：

```c
BSPLogPrintfTerminal(BSP_LOG_TERMINAL_ERROR,
                      "E:", RTT_CTRL_TEXT_BRIGHT_RED,
                      "motor=%u failed", motor_id);
```

`type`、`color` 和 `format` 必须是有效的非空字符串指针。任一参数为 `NULL` 时，函数直接返回 `-1`。

函数使用一个固定的 512 字节静态行缓冲区。完整输出由以下部分组成：

```text
两个空格 + color + type + 格式化正文 + "\r\n" + RTT_CTRL_RESET
```

正文过长时会被截断，但函数仍保证输出以字符串结束符结尾。正文可用长度取决于 `color` 和 `type` 的长度；512 字节目前是代码中的固定值，不由宏配置。前缀本身无法放入缓冲区时，函数返回 `-1`。

返回值是 `SEGGER_RTT_TerminalOut()` 的返回值：非负值表示 RTT 接口处理了该次输出，负值表示参数、Terminal 或 RTT 状态错误。RTT 使用跳过模式且空间不足时，可能返回 0，表示本次日志没有写入。

## 调度器和调用上下文

初始化后，普通任务可以并发调用日志接口。任务之间通过静态 FreeRTOS 互斥锁串行使用共享行缓冲区，竞争时会等待锁释放。

`BSPLogPrintfTerminal()` 不支持以下调用上下文，遇到这些情况会返回 `-1`：

- ISR 中调用；
- 调度器已启动但当前不处于运行状态；
- 关闭中断或处于使用 `BASEPRI` 的临界区；
- 未先成功初始化 `BSPLogInit()`。

调度器尚未启动时，函数允许单线程启动代码调用，并跳过互斥锁。`vsnprintf()` 的格式化栈仍由调用者任务承担；静态行缓冲区只移除了 512 字节的本地数组，不能据此推断任务栈只需要 512 字节。

## RTT Terminal

```c
#define BSP_LOG_CHANNEL_TERMINAL 0u
#define BSP_LOG_TERMINAL_DEFAULT 0u
#define BSP_LOG_TERMINAL_INFO    1u
#define BSP_LOG_TERMINAL_WARNING 2u
#define BSP_LOG_TERMINAL_ERROR   3u
```

Terminal 0 到 3 都属于 RTT 通道 0 内的虚拟终端，不是四个独立的 RTT 通道。`BSPLogSelectTerminal()` 可改变 RTT 当前活动 Terminal：

```c
if (BSPLogSelectTerminal(BSP_LOG_TERMINAL_INFO) < 0) {
  // Terminal 编号无效或 RTT 缓冲区空间不足
}
```

日志宏和 `BSPLogPrintfTerminal()` 会明确指定目标 Terminal。`BSPLogPrintfTerminal()` 完成写入后，SEGGER RTT 会恢复调用前的活动 Terminal，因此多个任务使用不同日志级别时不会因为活动 Terminal 被永久改变而串到其他终端。

在 RTT Viewer 或其他 RTT 客户端中，应连接通道 0，并按需要查看虚拟 Terminal 0、1、2、3。实际能否看到日志还取决于调试器已连接目标芯片并运行了支持 RTT 的调试会话。

## 浮点数转字符串

`Float2StrN()` 将 `float` 转换为固定三位小数的十进制字符串，并进行四舍五入：

```c
char text[32];
int length = Float2StrN(text, sizeof(text), -12.3456f);
// text: "-12.346"
// length: 7
```

`size` 包含结尾的 `\0`。即使目标缓冲区太小，函数也会尽量写入截断后的、以 `\0` 结尾的字符串，并返回完整结果长度；因此返回值大于等于 `size` 表示发生了截断。

以下情况返回 `-1`，并在目标缓冲区可用时写入空字符串：

- `str == NULL`；
- `size == 0`；
- 输入是 NaN 或无穷大；
- 缩放到千分位后超出内部 `int64_t` 表示范围。

该函数不依赖 `BSP_LOG_ENABLED`，可用于 RTT 日志之外的字符串格式化场景。

## 相关接口

| 接口                     | 作用                                   |
| ------------------------ | -------------------------------------- |
| `BSPLogInit()`           | 初始化 RTT 和日志互斥锁                |
| `BSPLogGetLastError()`   | 获取最近一次初始化或 Terminal 选择错误 |
| `BSPLogSelectTerminal()` | 设置 RTT 通道 0 的活动虚拟 Terminal    |
| `BSPLogPrintfTerminal()` | 格式化并写入指定虚拟 Terminal          |
| `Float2StrN()`           | 将 `float` 转换为固定三位小数字符串    |

以上接口和宏均声明在 `bsp/log/bsp_log.h` 中。
