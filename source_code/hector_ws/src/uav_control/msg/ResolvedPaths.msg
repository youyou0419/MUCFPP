# 自定义消息类型：LDR解决后的路径信息
# 文件路径: uav_control/msg/ResolvedPaths.msg

Header header
int32 ldr_id
string[] uav_ids
UAVPath[] uav_paths