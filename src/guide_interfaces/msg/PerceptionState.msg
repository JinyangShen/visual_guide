std_msgs/Header header

# 盲道横向偏差（-1.0 ~ 1.0，负值表示盲道偏左）
float32 lateral_error_m
# 本帧是否检测到盲道
bool blind_path_found
# 盲道是否已连续丢失（超过容忍帧数）
bool blind_path_lost
# 是否检测到近距障碍物
bool obstacle_near
# 障碍物所在侧（0=无，1=左，2=右）
uint8 obstacle_side
# 视觉感知置信度
float32 confidence
# 图像处理是否成功（false 表示本帧视觉不可信）
bool image_valid
