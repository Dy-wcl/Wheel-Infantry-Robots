# shoot_control_dr16

发射机构控制模块(DR16 遥控器).管理 CAN1 上的 2 台 M3508 摩擦轮 + 1 台 M2006 拨弹电机,使用位置式 PID;摩擦轮单速度环,拨弹电机速度位置双环.遥控数据来自全局 `dr16` 对象.

## 电机配置

| 机构 | 电机 | 设备 ID | CAN 总线 | 控制组 | 控制方式 |
| --- | --- | --- | --- | --- | --- |
| 左摩擦轮 | M3508 | 1 | CAN1 | 0x200 | 单速度环 |
| 右摩擦轮 | M3508 | 2 | CAN1 | 0x200 | 单速度环 |
| 拨弹 | M2006 | 3 | CAN1 | 0x200 | 速度位置双环 |

## API

| 函数 | 说明 |
| --- | --- |
| `err_t shoot_control_init(void)` | 初始化 CAN1 总线对象并注册 3 台电机(须在 `CAN_StartAll()` 之前;当前尚无调用方) |
| `void shoot_speed_pid_init(void)` | 初始化摩擦轮速度环与拨弹双环 PID |
| `pid_device_t* shoot_get_pid_device(index)` | 获取公共 PID 设备：0=左摩擦轮、1=右摩擦轮、2=拨弹，越界返回 NULL |
| `void dj_motor_pid_control_location(motor_id, target_angle)` | 兼容入口，仅索引 2 的拨弹电机支持位置控制，单位为转子角度（度） |
| `void Shoot_Mode(void)` | 模式控制主函数,控制任务 2ms 周期调用 |

## 拨弹角度闭环

`dj_motor_pid_control_location()` 以 `is_optimize=true` 转调公共 `pid_device_control_location()`：读取有效的 `position_deg` 反馈 → 计算单圈最短路径误差 → 位置环输出目标转子 RPM → 速度环输出电流指令 → 按电机型号限幅后写入组缓存。不累计减速器输出轴圈数，不在首次调用时重设目标或复位 PID。

新调用方可直接使用 `pid_device_control_speed(shoot_get_pid_device(0), rpm, ff)` 或 `pid_device_control_location(shoot_get_pid_device(2), angle_deg, ff, true)`；末参为 `false` 时直接使用目标和反馈角度，不取最短路径。初始化前提与统一接口说明见 [pid_device](../../../app/pid_device/pid_device.md)。

## PID 参数

各环使用 `PIDInstance` / `PIDInit()`，通过 `PID_DEVICE_POSITIONAL` 绑定公共设备；均启用 `PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter | PID_DerivativeFilter`。

各环 `Ki = Kd = 0`、死区为 0；输出滤波和微分滤波时间常数均为 0.0015 s（1.5 ms）。速度环积分限幅为 3000，位置环为 1000。采样间隔由 DWT 实测。位置式每轮计算绝对输出，切换算法后动态响应需要实机确认。

| 电机 | 环 | Kp | MaxOut |
| --- | --- | --- | --- |
| 摩擦轮(1/2 号) | 速度环 | 10.0 | 12000 |
| 拨弹(3 号) | 速度环(双环内环) | 8.0 | 10000(M2006 电流上限) |
| 拨弹(3 号) | 位置环(双环外环) | 8.0 | 3000(输出为目标速度) |

## DR16 发射模式

仅当左右拨杆都处于中位时，`dr16->dr16_cmd.ch_res` 才参与发射控制：

- `ch_res > 0.5` 持续约 10 ms：锁存当前角度加 30°，完成一次拨弹后拨弹电机停止，摩擦轮继续运行；保持该输入不会重复发射。
- `ch_res < -0.5`：摩擦轮低速运行，拨弹电机连续运行。
- `-0.5 <= ch_res <= 0.5`，或任一拨杆离开中位：清除单发状态并发送全零安全帧。

再次单发需要先回到死区或离开拨杆中位，再重新进入 `ch_res > 0.5`。

## 注意事项

- CAN2 的 0x200 组已被底盘 1-4 号电机占用,因此发射机构放在 CAN1;电机 ID 不能与底盘冲突.
- 依赖 `dr16_task` 提供的全局 `DR16_t* dr16`.
