
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
