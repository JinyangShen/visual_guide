std_msgs/Header header

# ============================================================
# 执行层契约（方案 A）
# ------------------------------------------------------------
# 执行层（底盘/电机驱动）只依据 command 字段动作，必须忽略
# search_direction 与 regression_distance 这两个"搜索建议"字段。
# 搜索建议通过独立话题 /search_hint 发布，供上层（人工遥控 / 导航）
# 参考，执行层不订阅该话题。
#
#   command = MODE_STOP    -> 立即停车；emergency=true 时不允许任何降级过渡
#   command = MODE_CAUTION -> 限速继续当前方向
#   command = MODE_FORWARD -> 正常速度直行
#   command = MODE_LEFT    -> 正常速度左转
#   command = MODE_RIGHT   -> 正常速度右转
#
# reason / confidence / * 字段仅用于日志、遥测与阈值调参，
# 执行层不应依据它们改变运动行为。
# 执行层必须自带指令看门狗：/guide_command 为 10Hz 发布，
# 超过 0.5s 未收到新指令即视为链路异常并自行停车。
# 完整说明见 guide_interfaces/EXECUTION_CONTRACT.md
# ============================================================

uint8 MODE_STOP=0
uint8 MODE_FORWARD=1
uint8 MODE_LEFT=2
uint8 MODE_RIGHT=3
uint8 MODE_CAUTION=4

uint8 command
float32 lateral_error_m
bool blind_path_lost
bool obstacle_near
uint8 obstacle_side
float32 confidence

# ---- 超声波融合字段（RDK X5 扩展） ----
# 最近一次有效超声波距离（米）；无有效数据时为 -1.0
float32 ultrasonic_dist
# 超声波数据是否有效（节点未启动/超时则为 false）
bool ultrasonic_valid
# 是否由超声波触发的急停/减速（true 表示决策被超声波覆盖）
bool ultrasonic_override

# ---- 定位字段（来自 /odom，供上层记录轨迹） ----
float32 odom_x
float32 odom_y
float32 odom_yaw

# ---- 安全仲裁字段（独立 safety_supervisor 扩展） ----
# 决策来源：0=视觉节点内部融合(inline)，1=安全仲裁节点(external)
uint8 SOURCE_INLINE=0
uint8 SOURCE_SUPERVISOR=1
uint8 source

# 停止/减速原因（便于日志分析与阈值调参）
uint8 REASON_NONE=0
uint8 REASON_ULTRASONIC=1        # 超声波近距急停
uint8 REASON_DNN_OBSTACLE=2      # 视觉/DNN 检测到障碍
uint8 REASON_BLIND_LOST=3        # 盲道完全丢失
uint8 REASON_PERCEPTION_TIMEOUT=4# 视觉数据超时（摄像头故障）
uint8 REASON_STUCK=5             # 里程计显示停滞（疑似卡住）
uint8 REASON_LOW_CONFIDENCE=6    # 感知置信度过低
uint8 reason

# 感知是否有效（external 模式下超时为 false）
bool perception_valid

# ---- 里程计参与决策的输出 ----
# 以下两个字段属于"给上层的建议"，执行层必须忽略（见文件头契约说明）。
# 盲道丢失时建议的回归方向：-1=向左回归，0=直行回归，1=向右回归
int8 search_direction
# 建议回归的距离（米）；无回归建议时为 -1.0
float32 regression_distance
# 是否为急停（true 时下游必须立即停车，不允许降级处理）
bool emergency
