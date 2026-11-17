
## Day 2 — 2026-11-11

### 目标
创建 guide_perception 桩节点，验证 /guide_command 话题管道。

### 完成
- `guide_perception`（ament_python）包创建，依赖 rclpy / sensor_msgs / guide_interfaces。
- 桩节点 `guide_node`：订阅 `/image_raw`（暂不处理），以 10 Hz 固定发布 `MODE_FORWARD`。
- 三面板（tmux）验证：`ros2 topic hz /guide_command` 实测 9.999 Hz；`ros2 topic echo` 可见 `command: 1` 持续刷新。

### 说明
- 今天不碰图像算法，只验证话题管道。
- `/image_raw` 暂无发布者，不影响 `/guide_command` 发布。

## Day 4 — 2026-11-12

### 完成
- 弃用质心法，改用 `cv2.fitLine` 对盲道轮廓做直线拟合，计算画面底部的横向偏差。
- 加入丢失容忍机制：连续丢失 <= 5 帧 (0.5s) 输出 `MODE_CAUTION`，> 5 帧输出 `MODE_STOP`。
- 解决了斜弯处质心偏移乱跳的问题。

### 已知限制
- 仍无法识别路口的断裂/交叉盲道（需要更高级的消失点/透视变换处理，赛前暂不深入）。

## Day 5 — 2026-11-13

### 完成
- 新增 `detect_obstacle`：通过检测画面正下方中心区域的“非黄色像素占比”，作为障碍物侵入的启发式判定。
- 新增 `fuse_decision`：建立安全优先级（障碍物/完全丢失 > 短暂丢失 > 循线）。
- 盲道循线参数与障碍物判定参数全部通过 ROS 2 参数暴露。

### 已知限制
- 障碍物判定高度依赖盲道底色，如果现场地面全是非黄色（比如纯灰地砖），可能会误报，需要现场调大 `obstacle_area_thresh`。

## Day 5 — 2026-11-13

### 完成
- 新增 `dnn_stub_pub` 节点（带 `ai_msgs` 导入保护），用于在板端模拟 DNN 检测结果。
- `guide_node` 升级为自适应模式：优先订阅 `/hobot_dnn_detection` 获取 YOLO 障碍物结果；如果环境缺失 `ai_msgs`（如 x86），自动降级为 HSV 启发式障碍物检测。
- 实现了安全决策融合：DNN 障碍物优先级最高，其次盲道丢失，最后是循线偏移。
- 验证了 x86 环境下的降级逻辑（HSV 模式），通道和指令输出正常。

## Day 11 — 2026-11-17

### 目标
打通"感知—决策—执行"全链路，并补上此前缺失的执行层。

### 完成
- 新增 `guide_executor`（ament_python）包，执行层节点 `executor_node`：
  - 只依据 `GuideCommand.command` 动作，忽略 reason / confidence /
    search_direction / regression_distance 等全部辅助字段（方案 A 契约）。
  - 自带 0.5s 指令看门狗：上游断流即自行停车，不依赖感知层来告知。
  - `MODE_STOP` 立即停车（`emergency` 时硬停），不做减速滑行。
  - 加速度斜坡限幅避免速度突变；未知 command 按停车处理（fail-safe）。
  - 输出 `/chassis/velocity`（Twist）与 `/executor/status`（状态监控）。
- 新增 `chassis_driver.py`（底盘下发接口，串口 / CAN 接入点已预留）与
  `mock_chassis_node.py`（差速模型模拟底盘，供无真机时闭环验证）。
- 新增 `executor.launch.py`；`guide_system.launch.py` 增加 `use_executor`、
  `use_mock_chassis`、`executor_output_topic`、`executor_watchdog_timeout`。
- 感知侧此前已完成：超声波节点、安全仲裁节点、轨迹记录节点，
  以及 `/search_hint` 搜索建议通道（执行层不订阅）。

### 验证
- `colcon test`：guide_perception / guide_executor 两个包 lint 全绿
  （顺带修掉 guide_perception 里 3 处既有 flake8 问题）。
- 执行层实测（Ubuntu，注入指令并采样 `/chassis/velocity`）：
  - FORWARD → lin_x=0.30；LEFT → ang_z=+0.40；RIGHT → ang_z=-0.40；
    CAUTION → lin_x=0.15（或沿用方向的半角速度）；STOP → 全零。
  - 契约：携带 `search_direction` / `regression_distance` 的指令不影响运动结果。
  - 看门狗：断流 1.5s 后 lin_x=0，状态标记 `watchdog=WATCHDOG_STOP`。
  - 订阅关系：`ros2 node info /executor` 仅有 `/guide_command` 一个订阅。
- 闭环：mock 底盘按 `/chassis/velocity` 积分出 `/odom`。

### 踩坑
- `rclpy.node.Node` 必须 `from rclpy.node import Node`；只 `import rclpy`
  会在类定义处抛 AttributeError。
- `RcutilsLogger.info()` 不接受多个位置参数，只能传单串或 f-string。
- `ros2 topic echo --once` 里角速度在 `angular.z`，不是 `angular.x`。
- `ros2 daemon stop` 之后要 `ros2 daemon start`，否则 CLI 报 `!rclpy.ok()`。

## Day 12 — 2026-11-17

### 目标
把"可交付"补齐：文档、自动化测试、CI、上层消费端，并明确真机阶段的待办。

### 完成
- 新增 `guide_upper` 包：上层提示节点 `search_hint_node`。
  这是方案 A 里"给上层的建议"通道的消费端——执行层不订阅 `/search_hint`，
  由这里把回归建议转成中文提示（带节流），可直接接 TTS 或遥控台界面。
  `guide_system.launch.py` 增加 `use_upper` 参数默认启动它。
- 自动化测试（此前只有 lint）：
  - `guide_executor/test/test_executor_node.py`：指令映射、未知指令 fail-safe、
    CAUTION 方向继承、加速度限幅，以及"发指令→有速度、断流→看门狗停车"的集成测试。
  - `guide_perception/test/test_supervisor_node.py`：七级优先级逐条验证，
    含"超声波压过视觉障碍""转向时不降速""超声波超时后不参与降速"等交互。
  - `guide_upper/test/test_search_hint_node.py`：三种回归方向的提示文本、
    `valid=false` 忽略、节流行为。
- 新增 GitHub Actions CI（`.github/workflows/ros2-ci.yml`）：
  ROS 2 Humble 环境下 `colcon build` + `colcon test` + 上传测试产物。
- 重写 README：架构图、目录结构、快速开始、话题一览、决策优先级、
  执行层契约、参数速查、调试命令、已知限制。
- 新增 `docs/ONBOARD.md`：真机接入清单（硬件信息表、板端环境、超声波/底盘真实实现、
  参数标定表、SLAM 调优、10 项安全验收测试、桩代码位置速查）。
- 现场盲道实拍 5 张归档到 `docs/images/`。
- `EXECUTION_CONTRACT.md` 补 v1.2：解释执行层看门狗（0.5s）与仲裁感知超时（1.0s）
  的先后关系，日志排查时如何区分两种停车。

### 工程约定
- 仓库内统一 LF 行尾（新增 `.gitattributes`），Windows 与 Linux 混编不再产生整文件 diff。
- 中文 docstring 与 pydocstyle 的英文句号规则冲突，两个包的 `test_pep257.py`
  统一豁免 D213/D400/D415，其余规则照常生效。

### 验证
- `colcon test`：34 项测试 0 失败（3 项跳过为 ament_copyright 例行跳过）。
- 整链实测（`use_rtabmap:=false use_mock_chassis:=true`）：
  - 8 个节点全部起来，含新增的 `/search_hint`。
  - 话题连接符合方案 A：`/search_hint` Publisher=1（safety_supervisor）
    / Subscription=1（search_hint），**执行层不订阅**；
    `/guide_command` Pub=1/Sub=1（只被 executor 消费）；`/chassis/velocity` Pub=1/Sub=1。
  - 注入 `valid=true` 的建议 → `/search_hint_text` 输出
    「盲道丢失，建议向左回归 1.5 米（请人工确认或等待重新识别）」；
    注入 `valid=false` → 无输出（按契约忽略）。
  - `/executor/status` 正常：`mode=STOP v=0.000 w=0.000 emergency=True watchdog=OK`。
  - launch 日志无 error / exception。

### 踩坑
- 上层提示文案原本拼成「建议向向左回归」，方向词重复。改成
  `建议{向左/向右/直行}回归`，三个方向的句子都通顺。
- `.gitignore` 里的 `images/` 会连 `docs/images/` 一起排除（gitignore 规则按目录名
  匹配任意层级），现场实拍因此没进库。改成 `/images/` 只忽略仓库根目录那份。
- 批量同步到 Ubuntu 时 `scp` 目标目录写错，launch 文件被放到包根目录而不是
  `launch/`，导致真实位置仍是旧版——整链验证时 `/search_hint` 节点起不来。
  同步脚本要用绝对目标路径，并在同步后立刻 grep 关键符号确认。
- 长任务不能靠 ssh 后台跑：ssh 断开或工具回收进程组时，launch 与子进程一起死。
  验证脚本要一次性跑完并把结果回传，或者放进 tmux。
