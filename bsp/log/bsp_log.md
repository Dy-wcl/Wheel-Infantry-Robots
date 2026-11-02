# bsp_log

<p align='right'>neozng1@hnu.edu.cn</p>

## 使用说明

bsp_log是基于segger RTT实现的日志打印模块。

推荐使用`bsp_log.h`中提供的三级日志：

```c
#define LOGINFO(format,...)
#define LOGWARNING(format,...)
#define LOGERROR(format,...)
```

分别用于输出不同等级的日志。这些日志宏使用 `vsnprintf()` 格式化，支持
`%f`、`%.2f` 等浮点格式（工程已链接 `_printf_float`）。底层轻量版
`SEGGER_RTT_printf()` / `PrintLog()` 不支持 `%f`，需要先转换成字符串。

### 栈使用与并发

`BSPLogPrintfTerminal()` 将完整行缓冲区放在静态 RAM 中，默认大小仍为
512 字节，由 `BSP_LOG_LINE_BUFFER_SIZE` 配置。颜色、终端、换行、颜色复位
和超长日志的截断行为不变。各任务通过静态分配的 FreeRTOS 互斥锁串行使用
该缓冲区；等待时支持优先级继承，不在整个浮点格式化过程中关闭中断。

在调度器启动前调用 `BSPLogInit()`；初始化后可在启动阶段或普通任务中打印。
任务竞争缓冲区时会等待锁释放；不支持 ISR、关中断/临界区、调度器挂起期间
调用，以上调用返回负值。RTT 缓冲区满时的默认行为仍为跳过本条输出。
格式化函数自身的栈帧减少，但 `vsnprintf()` 及浮点转换仍使用任务栈，不能
仅根据行缓冲区移出栈就把任务栈减为 512 字节。

Debug 配置默认定义 `MCU_DEBUG_BUILD` 并启用日志；Release 配置定义
`DISABLE_LOG_SYSTEM`，日志宏为空操作。需要在现场诊断时保留日志，可以额外定义
`BSP_LOG_FORCE_ENABLE`。

## RTT 通道和终端

日志系统初始化后使用以下上行通道：

| 通道 | 名称 | 默认缓冲区 | 用途 |
| --- | --- | ---: | --- |
| 0 | Terminal | RTT 默认值 | 彩色日志和虚拟终端 |
| 1 | DataOut | 1024 字节 | 原始数据输出 |
| 2 | Reserved | 256 字节 | 预留数据输出 |

日志级别使用 RTT 通道 0 中的不同虚拟 Terminal，因此在 J-Link RTT Viewer
中可分别打开对应终端查看：

| 宏 | Terminal | 说明 |
| --- | ---: | --- |
| `LOG` | 0 | 普通日志 |
| `LOGINFO` | 1 | 信息日志（绿色） |
| `LOGWARNING` | 2 | 警告日志（黄色） |
| `LOGERROR` | 3 | 错误日志（红色） |

每条日志由 `SEGGER_RTT_TerminalOut()` 完整写入目标 Terminal，写入后自动恢复
之前的活动 Terminal，避免不同任务并发输出时互相串线。Terminal 是通道 0 内的
虚拟终端，不是新的 RTT 上行通道；RTT Viewer 仍只需要连接 `Terminal` 通道。
如果上位机只显示当前 Terminal，请在 Viewer 中启用 Terminal 0、1、2、3 四个
窗口（或通过终端筛选分别查看）。

```c
BSPLogInit();
BSPLogSelectTerminal(0);             // 选择通道 0 内的虚拟终端
LOG_CHANNEL_WRITE(BSP_LOG_CHANNEL_DATA, data, data_size);
```

`SEGGER_RTT_SetTerminal()` 选择的是通道 0 内的虚拟终端，终端编号不是独立 RTT
通道号。需要自定义缓冲区时，可调用 `BSPLogConfigUpBuffer()` 或
`BSPLogConfigDownBuffer()`；非 0 通道只能在首次配置时设置名称、地址和大小。
默认缓冲模式为 `SEGGER_RTT_MODE_NO_BLOCK_SKIP`，缓冲区满时跳过本次写入，不阻塞实时任务。

**若想启用RTT，必须通过`launch.json`的`debug-jlink`启动调试（不论使用什么调试器）。** 按照`VSCode+Ozone环境配置`完成配置之后的cmsis dap和daplink是可以支持Jlink全家桶的。

另外，若你使用的是cmsis-dap和daplink，**请在 *jlink* 调试任务启动之后再打开`log`任务。**（均在项目文件夹下的.vsocde/task.json中，有注释自行查看）。否则可能出线RTT viewer无法连接客户端的情况。

在ozone中查看log输出，直接打开console调试任务台和terminal调试中断便可看到调试输出。

> 由于ozone版本的原因，可能出现日志不换行或没有颜色。

## 自定义输出

你也可以自定义输出格式，详见Segger RTT的文档。

```c
int printf_log(const char *fmt, ...);
void Float2Str(char *str, float va); // 兼容旧接口，目标缓冲区至少为 BSP_LOG_FLOAT_STR_SIZE 字节
int Float2StrN(char *str, size_t size, float va); // 推荐：带缓冲区长度检查
```
                                                                                              
调用第一个函数，可以通过jlink或dap-link向调试器连接的上位机发送信息，格式和printf相同，示例如下：

```c
printf_log("Hello World!\n");
printf_log("Motor %d met some problem, error code %d!\n",3,1);
```

第二个函数可以将浮点类型转换成字符串以方便发送：

```c
float current_feedback=114.514;
char str_buff[BSP_LOG_FLOAT_STR_SIZE];
Float2Str(str_buff,current_feedback);
printf_log("Motor %d met some problem, error code %d!\n",3,1);
```

`Float2Str()` 输出固定 3 位小数并进行四舍五入。新代码建议使用
`Float2StrN()`，传入目标缓冲区大小；当返回值大于等于 `size` 时，输出被截断，
返回负值表示参数无效或数值超出支持范围。
