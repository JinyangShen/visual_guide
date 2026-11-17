# 盲道丢失时的搜索建议（仅供上层参考：人工遥控 / 导航层）
#
# 方案 A 契约：
#   - 执行层（底盘/电机驱动）不订阅本话题，只消费 /guide_command 的 command 字段
#   - 上层（人工遥控台 / 导航模块）可订阅本话题，把回归方向转成人工干预或导航指令
#   - 无有效建议时 valid=false，此时 direction/distance 无意义
std_msgs/Header header

# 建议的回归方向：-1=向左回归，0=直行回归，1=向右回归
int8 direction
# 建议回归的距离（米）；无建议时为 -1.0
float32 distance
# 触发原因（与 GuideCommand.REASON_* 取值一致）
uint8 reason
# 是否有有效搜索建议
bool valid
