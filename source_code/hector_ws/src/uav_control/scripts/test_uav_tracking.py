#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from uav_control.msg import ResolvedPaths, UAVPath

def create_test_paths():
    """创建测试路径数据"""
    paths = ResolvedPaths()
    paths.ldr_id = 1  # 测试用的LDR ID
    
    # UAV1的路径 (简单直线)
    uav1 = UAVPath()
    uav1.uav_id = "uav1"
    uav1.time_indices = [0, 1,2, 9]
    uav1.path_points = [
        Point(-1.16, -0.89, 1.0),  # 起始点
        Point(-1.77, -0.57, 1.0),    # 中间点
        Point(3.69, -0.28, 1.0),
        Point(2.97,2.77,1.0)     # 终点
    ]
    
    # UAV2的路径 (更复杂的路径)
    uav2 = UAVPath()
    uav2.uav_id = "uav2"
    uav2.time_indices = [0, 1,2, 5, 9]
    uav2.path_points = [
        Point(0.34, 0.45, 1.0),
        Point(-2.37, 0.71, 1.0),
        Point(-4.22, -1.19, 1.0),
        Point(-4.72, -2.05, 1.0),
        Point(-3.16,- 3.06,1.0)
    ]
    
    # UAV3的路径 (最多路径点)
    uav3 = UAVPath()
    uav3.uav_id = "uav3"
    uav3.time_indices = [0, 1,2, 3, 4, 5, 9]
    uav3.path_points = [
        Point(0.97, -1.28, 1.0),
        Point(2.85, -4.30, 1.0),
        Point(3.58, -4.32, 1.0),
        Point(-0.29, -0.59, 1.0),
        Point(-0.08, -0.69, 1.0),
        Point(-3.54, 0.54, 1.0),
        Point(-3.16,2.96,1.0)
    ]
    
    # UAV4的路径
    uav4 = UAVPath()
    uav4.uav_id = "uav4"
    uav4.time_indices = [0,2,3,4,9]
    uav4.path_points = [
        Point(-1.13, 1.71, 1.0),
        Point(-1.09, 4.17, 1.0),
        Point(-1.23, -0.29, 1.0),
        Point(1.83, -0.63, 1.0),
        Point(2.97, -2.97, 1.0)
    ]
    
    paths.uav_paths = [uav1, uav2, uav3, uav4]
    return paths

def main():
    rospy.init_node('path_publisher_test')
    pub = rospy.Publisher('/resolved_paths', ResolvedPaths, queue_size=1)
    
    # 等待订阅者连接
    rospy.sleep(1)
    
    # 创建测试路径
    test_paths = create_test_paths()
    
    # 发布路径
    pub.publish(test_paths)
    rospy.loginfo("Published test paths to /resolved_paths")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass