# 真机接入清单（拿到硬件后逐项执行）

本清单覆盖"当前所有桩代码/估值参数"到"能在盲道上安全跑起来"之间的全部工作。
每项都标注了**改哪个文件**和**怎么验证**。勾选框用 `- [ ]`。

---

## 0. 先要确认的硬件信息（找供应商/卖家问清楚）

| 项 | 需要的信息 | 备注 |
|---|---|---|
| 底盘 | 串口波特率 + 帧格式（帧头/长度/校验/字段顺序），或 CAN ID + 报文定义 | 有厂家协议文档最好 |
| 电机 | 轮径、轮距、最大线速度、最大角速度 | 用于换算速度上限 |
| 超声波 | 型号：SRF02（I2C）还是 HC-SR04（数字脉冲）？I2C 地址 / 接线 GPIO 编号 | 决定走哪条代码路径 |
| 摄像头 | 型号、驱动包名、RGB/Depth 话题名、是否支持对齐 | 决定 `guide_system.launch.py` 的话题参数 |
| 板卡 | RDK X5 的系统镜像版本、ROS 2 是否已预装 | 决定板端环境搭建步骤 |

---

## 1. 板端环境搭建（RDK X5）

- [ ] 确认板端 ROS 2 版本为 Humble（`ros2 --version`），不一致则换镜像
- [ ] 安装依赖：
      ```bash
      sudo apt install ros-humble-cv-bridge ros-humble-rtabmap-ros ros-humble-tf2-ros
      rosdep install --from-paths src --ignore-src -r -y
      ```
- [ ] 拉取本仓库代码到板端并 `colcon build`
- [ ] 确认地瓜 DNN 节点存在且能出结果：
      ```bash
      ros2 topic list | grep dnn
      ros2 topic echo /hobot_dnn_detection --once
      ```
      若板端没有该节点，`guide_node` 会自动降级为 HSV 启发式障碍物检测
      （功能可用但精度下降，README 已注明）
- [ ] 确认相机驱动话题名，填进 launch 参数：
      ```bash
      ros2 launch guide_perception guide_system.launch.py \
          rgb_topic:=/<实际彩色话题> \
          depth_topic:=/<实际深度话题> \
          camera_info_topic:=/<实际内参话题>
      ```

**验证**：`ros2 topic hz /image_raw` 有稳定帧率。

---

## 2. 超声波真实读取

改 `src/guide_perception/guide_perception/ultrasonic_node.py`。

### 2.1 SRF02（I2C）

- [ ] 确认 I2C 总线号与地址（常见 `/dev/i2c-1`、地址 `0x70`）
- [ ] 安装 `pip install smbus2`
- [ ] 在 `ultrasonic_node.py` 中实现 `read_srf02()`：
      读 2 字节（高字节厘米 + 低字节小数厘米）→ 换算米 → 超过 `max_range` 时返回 `NaN`
- [ ] launch 里关掉模拟：`simulate_ultrasonic:=false`，并传 `i2c_bus` / `i2c_addr`

### 2.2 HC-SR04（数字脉冲）

- [ ] 确认 Trig / Echo 的 GPIO 编号
- [ ] 安装 `pip install RPi.GPIO`（或板端对应库）
- [ ] 实现：拉高 Trig 10µs → 测量 Echo 脉宽 → 距离 = 脉宽 × 0.034/2（cm）→ 换算米
- [ ] 加超时保护：脉宽超过 25 ms 视为无回波，返回 `NaN`（否则会误报极近障碍）

**验证**：

```bash
ros2 topic echo /ultrasonic/range
# 用手在 0.3 m / 0.6 m 处遮挡，读数应在 ±5 cm 内
# 移开障碍后应变成 NaN 或保持最后一次有效值（不刷新）
```

---

## 3. 底盘下发实现

改 `src/guide_executor/guide_executor/chassis_driver.py` 的 `send()`。

### 3.1 串口方案

- [ ] `pip install pyserial`
- [ ] 实现帧编码（按厂家协议），例如 `struct.pack('<ff', v, w)` + 帧头帧尾
- [ ] 加串口写失败（超时/断线）时的处理：**立即停车并置故障标志**
- [ ] 考虑丢包：底盘侧最好也带看门狗（收到帧才更新速度，超时归零）

### 3.2 CAN 方案

- [ ] 用 `python-can` 或板端 SocketCAN
- [ ] 实现报文打包（速度一般乘 100 取整发送）
- [ ] 监听 CAN 错误帧，检测到底盘离线时停车

### 3.3 接入 ros2_control（若底盘已有 diff_drive_controller）

- [ ] launch 里改 `executor_output_topic:=/cmd_vel`
- [ ] 确认 `diff_drive_controller` 的 `wheel_separation` / `wheel_radius` 与实物一致
- [ ] 注意：`/cmd_vel` 是 `geometry_msgs/Twist`，与本项目输出类型一致，无需转换

**验证**：

```bash
ros2 topic echo /chassis/velocity      # 执行层输出
ros2 topic echo /cmd_vel                # 若走 ros2_control
# 急停测试：直接 kill 执行层节点，底盘必须立即停
```

---

## 4. 参数标定（在真实盲道上跑）

以下默认值都是估值，必须实测后回填到 launch 文件与 README：

| 参数 | 当前默认 | 标定方法 | 建议值范围 |
|---|---|---|---|
| `max_speed` | 0.30 m/s | 人在后跟随，调到"能及时刹住且不晃" | 0.2 ~ 0.4 |
| `turn_speed` | 0.40 rad/s | 观察过弯是否跟不上盲道 | 0.3 ~ 0.8 |
| `caution_speed` | 0.15 m/s | 降速后仍能保持行进 | 0.1 ~ 0.2 |
| `max_accel` | 0.5 m/s² | 起步是否打滑 | 0.3 ~ 0.8 |
| `ultra_stop_dist` | 0.2 m | 急停后车头到障碍物的距离 | ≥ 制动距离 + 0.1 |
| `ultra_caution_dist` | 0.5 m | 开始降速的合理距离 | 0.4 ~ 0.8 |
| `dead_zone` | 0.15 m | 盲道居中时偏差读数的波动幅度 | 按实测波动 × 2 |
| `h_min`/`h_max`/`s_min`/`v_min` | 16/40/80/80 | 用 ROS 工具取盲道像素 HSV 值 | 按现场盲道颜色 |
| `stuck_threshold` | 0.05 m | 人为挡住车，看是否判停滞 | 0.03 ~ 0.08 |

**标定方法提示**：把 `guide_node` 的 `fusion_mode` 设成 `inline`，
用 `ros2 topic echo /perception_state` 看 `lateral_error_m` 实时值来调死区与 HSV。

---

## 5. SLAM / 建图调优（RDK X5 算力有限）

- [ ] 确认 `rgbd_odometry` 不空转：`ros2 topic hz /odom` 稳定
- [ ] 特征数已限制为 400（`args: '--Kp/MaxFeatures 400 --Vis/MaxFeatures 400'`），
      若帧率仍低再降到 300，并观察定位是否漂移
- [ ] 无显示器环境保持 `rtabmap_viz:=false`、`rviz:=false`
- [ ] 若不需要建图，直接 `use_rtabmap:=false`，但 `/odom` 就没了 ——
      此时需要底盘自身发布里程计，或用 `mock_chassis_node` 之外的真实 odom 源
- [ ] 检查 `camera_link → base_link` 静态 TF 是否与实际安装位置一致
      （launch 里的 `static_transform_publisher` 目前是零位占位，要按实物改平移/旋转）

---

## 6. 安全验收测试（必须逐项通过）

在**空旷场地、有人随时能按下急停**的条件下执行：

| # | 测试项 | 操作 | 期望结果 |
|---|---|---|---|
| 1 | 急停距离 | 推车/放障碍物在正前方，机器人前进 | 在 `ultra_stop_dist` 之外就停住，**车头不碰障碍** |
| 2 | 断链停车 | `pkill -f executor_node` | 0.5s 内底盘归零 |
| 3 | 感知崩溃停车 | `pkill -f guide_node` | 机器人停止（仲裁节点独立生效） |
| 4 | 摄像头拔线 | 拔掉 USB/网线 | 1.0s 内停车，`reason=REASON_PERCEPTION_TIMEOUT` |
| 5 | 超声波拔线 | 断开超声波 | 不应误报急停；应降级为仅视觉 |
| 6 | 盲道遮挡 | 用纸板盖住盲道 | 停车 + 上层提示"建议向左/右回归" |
| 7 | 回归方向正确性 | 把车推离盲道后左偏 / 右偏 | `/search_hint` 的 `direction` 与实际偏离方向一致 |
| 8 | 停车不滑行 | 高速前进中触发 STOP | 立即停住，无惯性滑行 |
| 9 | 卡住检测 | 用手顶住车 | `reason=REASON_STUCK`，车不动 |
| 10 | 解锁恢复 | 松手 | 恢复行走，不抖动 |

**回放验证**：把 `/guide_command` 用 `ros2 bag record` 存下来，事后用
`ros2 bag play` 回放分析指令序列是否合理（无来回跳变）。

---

## 7. 固件/底盘侧建议

- [ ] 底盘固件也加"指令超时自动停车"（双保险，优先于上位机看门狗）
- [ ] 底盘固件做速度/加速度限幅，非法帧直接丢弃并停车
- [ ] 记录里程计原始轮速，便于与 `/odom` 交叉验证

---

## 8. 收尾

- [ ] 把标定后的参数回填到 `guide_system.launch.py` 默认值
- [ ] 更新 README「参数速查」表为实测值
- [ ] 在 DEVLOG.md 追加真机调试记录
- [ ] 把真机照片/视频归档到 `docs/images/`
- [ ] 提交并推送：`git add -A && git commit -m "..." && git push origin main`

---

## 附：桩代码位置速查

| 桩 | 文件 | 需要替换的函数 |
|---|---|---|
| 超声波 I2C | `src/guide_perception/guide_perception/ultrasonic_node.py` | `read_srf02()` |
| 超声波 GPIO | 同上 | `read_hcsr04()` |
| 底盘下发 | `src/guide_executor/guide_executor/chassis_driver.py` | `ChassisDriver.send()` |
| DNN 障碍物 | `src/guide_perception/guide_perception/guide_node.py` | `dnn_stub_pub` 降级分支（板端有真节点时自动启用） |
| 相机 TF | `src/guide_perception/launch/guide_system.launch.py` | `static_transform_publisher` 的零位参数 |
