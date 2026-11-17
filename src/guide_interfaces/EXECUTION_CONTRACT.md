# 执行层契约（方案 A）

适用对象：底盘 / 电机驱动执行层（消费 `guide_interfaces/GuideCommand` 的节点）

版本：v1.0 　对应接口：`guide_interfaces/msg/GuideCommand.msg`

---

## 1. 话题分工

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/guide_command` | `GuideCommand` | `safety_supervisor` | **执行层** | 唯一的运动指令来源，10 Hz |
| `/search_hint` | `SearchHint` | `safety_supervisor` | 上层（人工遥控 / 导航） | 回归建议，**执行层不订阅** |
| `/safety_reason` | `Int32` | `safety_supervisor` | 监控 / 日志 | 决策原因码，不参与运动决策 |
| `/guide_trajectory` | `nav_msgs/Path` | `trajectory_recorder` | 监控 / 导航 | 历史轨迹 |
| `/chassis/velocity` | `geometry_msgs/Twist` | `executor` | 底盘驱动 | 执行层输出的速度指令 |
| `/executor/status` | `std_msgs/String` | `executor` | 监控 / 日志 | 执行层状态，含看门狗判定 |

`/guide_command` 中的 `search_direction`、`regression_distance` 属于"给上层的建议"，
**执行层必须忽略**。它们之所以仍留在同一条消息里，是为了兼容早期版本的订阅者；
新代码请从 `/search_hint` 读取建议。

---

## 2. 执行层必须实现的行为

| `command` | 执行层动作 | 备注 |
|---|---|---|
| `MODE_STOP` (0) | 立即停车 | `emergency == true` 时**不允许任何减速过渡**，直接停 |
| `MODE_CAUTION` (4) | 限速继续当前方向 | 建议限速 0.1~0.2 m/s，转向动作可保持 |
| `MODE_FORWARD` (1) | 正常速度直行 | |
| `MODE_LEFT` (2) | 正常速度左转 | |
| `MODE_RIGHT` (3) | 正常速度右转 | |

### 禁止事项

1. **不得**依据 `reason` 改变运动行为。`reason` 只用于日志与遥测。
2. **不得**使用 `search_direction` / `regression_distance` 驱动任何运动。
3. **不得**对 `MODE_STOP` 做"减速滑行"处理；`emergency == true` 时是硬停。

### 必须实现：指令看门狗

`/guide_command` 以 10 Hz 发布。执行层必须自己判断指令是否新鲜：

```
若 now - last_command_timestamp > 0.5s  ->  立即停车并上报链路故障
```

这条不能依赖感知层保证——感知层进程崩溃时不会发出任何"我挂了"的通知，
只有执行层的本地看门狗能保证底盘停下来。

---

## 3. `emergency` 与 `reason` 的语义（供日志与调参）

| `reason` | 含义 | `emergency` |
|---|---|---|
| `REASON_NONE` (0) | 正常循迹 | false |
| `REASON_ULTRASONIC` (1) | 超声波 < 0.2 m 急停 | **true** |
| `REASON_DNN_OBSTACLE` (2) | 视觉/DNN 检测到障碍 | false |
| `REASON_BLIND_LOST` (3) | 盲道完全丢失 | false |
| `REASON_PERCEPTION_TIMEOUT` (4) | 感知链路失效（摄像头故障 / 感知进程崩溃） | **true** |
| `REASON_STUCK` (5) | 里程计显示停滞（疑似卡住） | false |

`emergency == true` 表示"无条件硬停"，只出现在 `REASON_ULTRASONIC` 和
`REASON_PERCEPTION_TIMEOUT` 两种情况。

---

## 4. 上层（人工遥控 / 导航）的可选用法

盲道丢失时，`safety_supervisor` 会输出 `MODE_STOP`，同时通过 `/search_hint`
给出回归建议：

| 字段 | 含义 |
|---|---|
| `direction` | -1 向左回归，0 直行回归，1 向右回归 |
| `distance` | 建议回归距离（米），目标不可信时为 -1.0 |
| `valid` | `true` 才表示建议可用 |
| `reason` | 触发原因，与 `GuideCommand.REASON_*` 一致 |

上层可据此提示盲人"向左回退约 1 米"或驱动导航模块执行回归。
**执行层不参与这个流程**，盲道丢失时它只负责停下来。

---

## 5. 执行层实现（`guide_executor` 包）

本仓库已提供执行层实现：`guide_executor/executor_node.py`。

### 节点结构

| 项 | 值 |
|---|---|
| 节点名 | `executor` |
| 输入 | `/guide_command`（`GuideCommand`，10 Hz） |
| 输出 | `/chassis/velocity`（`geometry_msgs/Twist`）、`/executor/status`（`String`） |
| 控制频率 | 50 Hz（独立定时器，不依赖上游心跳） |
| 独立启动 | `ros2 launch guide_executor executor.launch.py` |

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `watchdog_timeout` | 0.5 | 指令超时（秒），超时后自行停车 |
| `max_speed` | 0.30 | `MODE_FORWARD` 速度（m/s） |
| `caution_speed` | 0.15 | `MODE_CAUTION` 限速（m/s） |
| `turn_speed` | 0.40 | 转向角速度（rad/s） |
| `caution_ratio` | 0.5 | `MODE_CAUTION` 时转向角速度的折减比例 |
| `max_accel` | 0.5 | 线加速度上限（m/s²） |
| `max_angular_accel` | 1.0 | 角加速度上限（rad/s²） |
| `output_topic` | `/chassis/velocity` | 底盘速度话题；接 `ros2_control` 时改成 `/cmd_vel` |

### 行为约定

1. **只读 `command`**：`reason` / `confidence` / `search_direction` /
   `regression_distance` / `ultrasonic_*` 一律不读。
2. **未知取值按停车处理**：不是 fail-open，是 fail-safe。
3. **`MODE_STOP` 立即停车**，不走加速度斜坡；`emergency == true` 时同样硬停。
4. **`MODE_CAUTION` 沿用最近一次运动方向**的半速（直行半速、转向半角速度）。
   看门狗停车只表示"停住"、不代表朝向改变，因此不会清空该方向记忆。
5. **加速度斜坡**只作用于运动指令，避免速度突变；停车不走斜坡。
6. **看门狗**独立于上游：超过 `watchdog_timeout` 没收到新指令就自行停车，
   并在 `/executor/status` 标记 `watchdog=WATCHDOG_STOP`。
7. **退出时强制停车**：节点 `destroy_node` 会下发一次零速。

### 真机接入

`chassis_driver.py` 负责把速度下发到底盘话题。串口 / CAN 驱动在
`ChassisDriver.send()` 的注释处接入即可，感知与决策代码无需改动。

---

## 6. 变更记录

- v1.0：确立方案 A——执行层只认 `command` 五个取值；搜索建议移出执行层路径，
  通过 `/search_hint` 单独提供给上层。
- v1.1：补齐执行层实现 `guide_executor`（含看门狗、加速度斜坡、CAUTION 方向继承），
  并给出参数表与真机接入位置。
