# 导盲感知系统（ROS 2 Humble / RDK X5）

面向导盲场景的视觉循迹 + 超声波近距避障系统。摄像头识别盲道并输出转向指令，
超声波兜底近距障碍，独立安全仲裁节点保证任何环节失效时都会停车，执行层把指令
翻译成底盘速度。

```
                    ┌───────────────────────────────────────────────┐
   摄像头 ─────────►│ guide_node  视觉感知                          │
                    │  · 盲道 HSV 检测    · DNN 障碍物（板端可选）  │
                    └───────────────────┬───────────────────────────┘
                                        │ /perception_state
                                        ▼
   超声波 ─────────►┌───────────────────────────────────────────────┐
   /odom ──────────►│ safety_supervisor  独立安全仲裁              │
                    │  感知超时 › 超声急停 › 视觉障碍 › 盲道丢失    │
                    │  › 正常循迹 › 中距降速 › 停滞检测            │
                    └───────┬────────────────────────┬──────────────┘
                            │ /guide_command         │ /search_hint
                            ▼ （执行层只认 command）  ▼ （仅供上层参考）
                    ┌───────────────────┐   ┌───────────────────┐
                    │ executor  执行层   │   │ search_hint  上层  │
                    │  · 0.5s 指令看门狗 │   │  · 中文语音提示    │
                    │  · 加速度限幅      │   │  · 可接 TTS       │
                    └─────────┬─────────┘   └───────────────────┘
                              │ /chassis/velocity (Twist)
                              ▼
                          底盘驱动（串口 / CAN / ros2_control）
```

## 目录结构

```
src/
├── guide_interfaces/            接口定义（rosidl 自定义消息）
│   ├── msg/
│   │   ├── GuideCommand.msg     最终运动指令 + 决策原因 + 搜索建议
│   │   ├── PerceptionState.msg  视觉感知结果
│   │   └── SearchHint.msg       回归搜索建议（仅上层消费）
│   └── EXECUTION_CONTRACT.md    ★ 执行层契约（方案 A），改执行层前必读
├── guide_perception/            感知与决策
│   ├── guide_perception/
│   │   ├── guide_node.py            视觉感知（HSV 盲道 + DNN 障碍物）
│   │   ├── ultrasonic_node.py       超声波测距（模拟开关 + I2C/GPIO 桩）
│   │   ├── safety_supervisor_node.py 独立安全仲裁（七级优先级）
│   │   ├── trajectory_recorder_node.py 轨迹记录（/guide_trajectory）
│   │   ├── dnn_stub_pub.py          DNN 结果模拟
│   │   └── fake_image_pub.py        假图像源（无相机时用）
│   └── launch/guide_system.launch.py 一键启动
├── guide_executor/              执行层
│   ├── guide_executor/
│   │   ├── executor_node.py        只认 command 的执行层 + 看门狗
│   │   ├── chassis_driver.py       底盘下发接口（串口/CAN 接入点）
│   │   └── mock_chassis_node.py    差速模拟底盘（开发验证用）
│   └── launch/executor.launch.py
└── guide_upper/                 上层应用
    ├── guide_upper/search_hint_node.py  搜索建议 -> 中文提示
    └── launch/upper.launch.py
docs/
├── ONBOARD.md                   ★ 拿到真机后的接入清单（待办）
└── images/                      现场盲道实拍
```

## 环境要求

- Ubuntu 22.04 / ROS 2 Humble
- Python 3.10+
- 目标硬件：RDK X5（ARM64），开发机 x86 亦可运行（自动降级为 HSV 障碍物检测）

## 快速开始

```bash
# 1. 拉代码
git clone git@github.com:JinyangShen/visual_guide.git
cd visual_guide

# 2. 装依赖
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-cv-bridge ros-humble-rtabmap-ros
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译
colcon build

# 4. 一键启动（无相机、无硬件的纯开发模式）
source install/setup.bash
ros2 launch guide_perception guide_system.launch.py \
    use_rtabmap:=false use_mock_chassis:=true

# 5. 另开终端喂假图像
ros2 run guide_perception fake_image_pub
```

真机部署参数见 [docs/ONBOARD.md](docs/ONBOARD.md)。

## 话题一览

| 话题 | 类型 | 发布者 → 订阅者 | 说明 |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | 相机驱动 → `guide_node` | 摄像头输入 |
| `/perception_state` | `PerceptionState` | `guide_node` → `safety_supervisor` | 视觉感知结果 |
| `/ultrasonic/range` | `sensor_msgs/Range` | `ultrasonic_node` → `safety_supervisor` | 超声波测距，10 Hz |
| `/odom` | `nav_msgs/Odometry` | RTAB-Map / 底盘 → `safety_supervisor` | 定位，参与回归与停滞检测 |
| `/guide_command` | `GuideCommand` | `safety_supervisor` → **`executor`** | ★ 唯一的运动指令来源，10 Hz |
| `/search_hint` | `SearchHint` | `safety_supervisor` → 上层 | 回归建议，**执行层不订阅** |
| `/chassis/velocity` | `geometry_msgs/Twist` | `executor` → 底盘驱动 | 底盘速度指令 |
| `/executor/status` | `std_msgs/String` | `executor` → 监控 | 执行层状态（含看门狗判定） |
| `/search_hint_text` | `std_msgs/String` | `search_hint` → TTS/UI | 中文提示文本 |
| `/guide_trajectory` | `nav_msgs/Path` | `trajectory_recorder` → 监控 | 历史轨迹 |

## 决策优先级

安全仲裁节点按优先级从高到低判定，**任何时刻只有一个理由生效**：

| 优先级 | 条件 | 结果 | reason 码 |
|---|---|---|---|
| 1 | 视觉数据超时（1.0s） | 急停 `emergency=true` | `REASON_PERCEPTION_TIMEOUT` |
| 2 | 超声波 < 0.2 m | 急停 `emergency=true` | `REASON_ULTRASONIC` |
| 3 | 视觉检测到障碍物 | 停车 | `REASON_DNN_OBSTACLE` |
| 4 | 盲道完全丢失 | 停车 + 给出回归建议 | `REASON_BLIND_LOST` |
| 5 | 横向偏差在死区（±0.15 m） | 直行 | — |
| 6 | 偏差超死区 | 左转 / 右转 | — |
| 7 | 超声波 0.2~0.5 m 且打算直行 | 降速 `MODE_CAUTION` | — |

另有里程计参与的**停滞检测**：窗口内位移过小且本该在走 → 停车（`REASON_STUCK`），
采用锁存语义，恢复位移后自动解锁，避免指令抖动。

## 执行层契约（方案 A）

执行层**只依据 `GuideCommand.command` 这一个字段动作**，共 5 个取值：

```
MODE_STOP(0)    立即停车（emergency 时硬停，不做减速滑行）
MODE_FORWARD(1) 正常速度直行
MODE_LEFT(2)    正常速度左转
MODE_RIGHT(3)   正常速度右转
MODE_CAUTION(4) 限速继续当前方向
```

明确禁止：

- ❌ 依据 `reason` / `confidence` / `ultrasonic_*` 改变运动行为
- ❌ 使用 `search_direction` / `regression_distance` 驱动车轮（那属于上层建议）
- ❌ 对 `MODE_STOP` 做减速滑行处理
- ✅ 必须自带 0.5s 指令看门狗（上游断流即自行停车）

完整契约与接入说明见 [src/guide_interfaces/EXECUTION_CONTRACT.md](src/guide_interfaces/EXECUTION_CONTRACT.md)。

## 参数速查

**guide_node**（视觉感知）

| 参数 | 默认 | 说明 |
|---|---|---|
| `fusion_mode` | `inline` | `inline` 自行下发指令 / `external` 只发感知结果 |
| `h_min`/`h_max` | 16/40 | 盲道 HSV 色相范围（按现场盲道颜色调） |
| `s_min`/`v_min` | 80/80 | 盲道 HSV 饱和度/亮度下限 |
| `ultra_stop_dist` | 0.2 | 超声波急停距离（m） |
| `ultra_caution_dist` | 0.5 | 超声波降速距离（m） |
| `ultra_timeout` | 1.0 | 超声波数据超时（s） |

**safety_supervisor**（安全仲裁）

| 参数 | 默认 | 说明 |
|---|---|---|
| `perception_timeout` | 1.0 | 视觉数据超时（s） |
| `dead_zone` | 0.15 | 循迹死区（m） |
| `decision_rate` | 10.0 | 指令发布频率（Hz） |
| `stuck_window` | 3.0 | 停滞检测窗口（s） |
| `stuck_threshold` | 0.05 | 窗口内最小位移（m） |
| `max_regression_distance` | 5.0 | 回归搜索最大距离（m） |
| `search_angle_threshold` | 0.35 | 判定"需要转向"的方位角阈值（rad） |

**executor**（执行层）

| 参数 | 默认 | 说明 |
|---|---|---|
| `watchdog_timeout` | 0.5 | 指令看门狗超时（s） |
| `max_speed` | 0.30 | 直行速度（m/s） |
| `caution_speed` | 0.15 | 降速速度（m/s） |
| `turn_speed` | 0.40 | 转向角速度（rad/s） |
| `max_accel` | 0.5 | 线加速度上限（m/s²） |
| `output_topic` | `/chassis/velocity` | 底盘速度话题，接 `ros2_control` 改 `/cmd_vel` |

**ultrasonic_node**：`simulate`（默认 true，无硬件时保持）、`publish_rate`（10 Hz）、
`i2c_bus`/`i2c_addr`（SRF02）、`gpio_trig`/`gpio_echo`（HC-SR04）。

## 常用调试命令

```bash
ros2 topic echo /guide_command              # 看最终指令
ros2 topic echo /perception_state            # 看视觉感知结果
ros2 topic hz /chassis/velocity              # 执行层输出频率（应为 50 Hz）
ros2 topic echo /executor/status             # 看门狗状态
ros2 topic info /guide_command -v            # 确认只有执行层在订阅
ros2 node info /executor                    # 确认执行层只订 /guide_command
ros2 topic pub -r 10 /guide_command guide_interfaces/msg/GuideCommand '{command: 1}'
```

## 测试与 CI

```bash
colcon test --packages-select guide_perception guide_executor guide_upper
colcon test-result --verbose
```

现有自动化测试覆盖：执行层的指令映射与看门狗停车、仲裁节点的七级优先级交互、
上层提示消费端的节流行为。GitHub Actions 会在每次 push 自动跑 `colcon build` + `colcon test`。

## 文档

| 文档 | 内容 |
|---|---|
| [src/guide_interfaces/EXECUTION_CONTRACT.md](src/guide_interfaces/EXECUTION_CONTRACT.md) | 执行层契约（方案 A），改执行层前必读 |
| [docs/ONBOARD.md](docs/ONBOARD.md) | **拿到真机后的接入清单**：硬件确认、驱动实现、参数标定、安全验收 |
| [DEVLOG.md](DEVLOG.md) | 开发日志，记录每个阶段的设计取舍与踩坑 |
| [docs/images/](docs/images/) | 现场盲道实拍（正中间 / 偏左 / 偏右 / 有障碍物） |
| `guide_ws_src.zip` | `src/` 的源码快照，便于不装 git 也能整包拷到开发机（内容与仓库一致） |

## 已知限制

- 超声波与底盘的真实硬件读取仍是桩代码，需按 [ONBOARD.md](docs/ONBOARD.md) 接入
- 所有验证均在 x86 Ubuntu 完成，RDK X5 板端（DNN 节点、相机驱动、算力限制）尚未实测
- 速度、转向角速度、各项阈值均为估值，需要真机标定
