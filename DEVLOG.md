
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
