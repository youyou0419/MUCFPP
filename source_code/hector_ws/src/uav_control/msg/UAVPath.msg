# 单个无人机的路径信息
# 文件路径: uav_control/msg/UAVPath.msg

string uav_id
geometry_msgs/Point[] path_points
int32[] time_indices