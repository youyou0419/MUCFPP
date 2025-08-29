# ResolutionStatus.msg
# LDR Resolution状态消息定义


# LDR标识符
int32 ldr_id

# 涉及的无人机ID列表
string[] uav_ids

# Resolution状态：true表示进入Resolution阶段，false表示退出
bool in_resolution


# LDR区域边界信息
float64 aabb_min_x
float64 aabb_max_x  
float64 aabb_min_y
float64 aabb_max_y