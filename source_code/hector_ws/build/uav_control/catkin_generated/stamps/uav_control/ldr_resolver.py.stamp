#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LDR冲突解脱ROS节点
负责ROS通信和调用核心算法模块
使用服务调用方式代替话题发布
增加了LDR可视化功能
"""

import rospy
from uav_control.msg import LDRInfo, LDRInfoArray, ResolvedPaths, UAVPath, ResolutionStatus, LDRCompletion
from uav_control.srv import PathTracking, PathTrackingRequest  # 新增的服务类型
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import String, Header, Int32MultiArray, Bool
from gazebo_msgs.msg import ModelStates  # 新增导入
from visualization_msgs.msg import MarkerArray, Marker  # 新增：可视化消息类型
import numpy as np
import threading
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from quad_star.srv import PathPlan, PathPlanRequest
from std_srvs.srv import Empty, EmptyRequest
import time

# 导入核心算法模块
from ldr_algorithm import LDRAlgorithm


class LDRResolver:
    """LDR冲突解脱ROS节点类"""
    
    def __init__(self):
        rospy.init_node('ldr_resolver')
        print("[ROS] LDR冲突解脱器启动")
        
        # 获取ROS参数
        stability_threshold = rospy.get_param('~stability_threshold', 4.0)
        safety_distance = rospy.get_param('~safety_distance', 1)
        
        # 初始化核心算法模块
        self.algorithm = LDRAlgorithm(
            stability_threshold=stability_threshold,
            safety_distance=safety_distance
        )
        
        # 新增：无人机状态存储和锁
        self.uav_positions = {}
        self.position_lock = threading.Lock()
        
        # 新增：可视化相关变量
        self.active_visualizations = {}  # 存储活动的可视化LDR
        self.visualization_lock = threading.Lock()
        
        # ROS通信设置
        self.ldr_sub = rospy.Subscriber('/ldr_info', LDRInfoArray, self.ldr_callback)
        
        # 新增：订阅无人机状态
        self.state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        
        # Resolution状态广播（保留状态广播功能）
        self.resolution_status_pub = rospy.Publisher('/resolution_status', ResolutionStatus, queue_size=10)
        
        # 新增：LDR完成广播发布者
        self.ldr_completion_pub = rospy.Publisher('/ldr_completion', LDRCompletion, queue_size=10)
        
        # 新增：可视化发布者
        self.ldr_viz_pub = rospy.Publisher('/ldr_instances', MarkerArray, queue_size=1)
        self.aabb_viz_pub = rospy.Publisher('/aabb_visualization', MarkerArray, queue_size=1)
        
        # 等待路径规划服务
        rospy.wait_for_service('/path_planning_service')
        self.path_plan_client = rospy.ServiceProxy('/path_planning_service', PathPlan)
        print("[ROS] 已连接路径规划服务")
        
        # 等待路径跟踪服务
        rospy.wait_for_service('/path_tracking_service')
        self.path_tracking_client = rospy.ServiceProxy('/path_tracking_service', PathTracking)
        print("[ROS] 已连接路径跟踪服务")
        
        # 初始化任务服务客户端和停止发布者
        self.mission_clients = {}
        self.stop_publishers = {}
        self._init_uav_services()
        
        print(f"[ROS] 参数设置 - 稳定阈值={stability_threshold}s, 安全距离={safety_distance}m")
        print("[ROS] 可视化功能已启用")
        
        # 启动稳定性检测线程
        self.stability_check_thread = threading.Thread(target=self.stability_check_loop)
        self.stability_check_thread.daemon = True
        self.stability_check_thread.start()
        print("[ROS] 稳定性检测线程已启动")

    def state_callback(self, msg):
        """更新无人机状态"""
        with self.position_lock:
            for i, model_name in enumerate(msg.name):
                if model_name.startswith('uav'):
                    pos = msg.pose[i].position
                    self.uav_positions[model_name] = (pos.x, pos.y)

    def _init_uav_services(self):
        """初始化无人机服务客户端和发布者"""
        uav_list = ['uav1', 'uav2', 'uav3', 'uav4']  # 可根据需要扩展
        
        for uav_name in uav_list:
            # 初始化任务开始服务客户端
            service_name = f"/{uav_name}/start_mission"
            try:
                rospy.wait_for_service(service_name, timeout=2.0)
                self.mission_clients[uav_name] = rospy.ServiceProxy(service_name, Empty)
                print(f"[ROS] 已连接 {uav_name} 任务服务")
            except rospy.ROSException:
                print(f"[ROS] 警告: 无法连接 {service_name} 服务，将在需要时重试")
                self.mission_clients[uav_name] = None
            
            # 初始化停止信号发布者
            stop_topic = f"/{uav_name}/needstop"
            self.stop_publishers[uav_name] = rospy.Publisher(stop_topic, Bool, queue_size=1, latch=True)
            print(f"[ROS] 已创建 {uav_name} 停止信号发布者")

    def stability_check_loop(self):
        """独立的稳定性检测循环"""
        while not rospy.is_shutdown():
            # 检查稳定性
            to_resolve = self.algorithm.check_stability()
            
            # 处理需要解决的LDR
            for ldr_id in to_resolve:
                self.broadcast_resolution_status_for_ldr(ldr_id, True)
                threading.Thread(target=self.resolve_ldr, args=(ldr_id,)).start()
            
            time.sleep(0.1)

    def ldr_callback(self, msg):
        """处理接收到的LDR信息"""
        print(f"\n[ROS] 收到LDR信息，包含{len(msg.ldr_infos)}个LDR实例")
        
        # 转换ROS消息格式为算法需要的格式
        ldr_info_list = []
        for ldr_info in msg.ldr_infos:
            positions = [(pos.x, pos.y) for pos in ldr_info.positions]
            ldr_data = {
                'uav_ids': ldr_info.uav_ids,
                'positions': positions,
                'min_x': ldr_info.min_x,
                'max_x': ldr_info.max_x,
                'min_y': ldr_info.min_y,
                'max_y': ldr_info.max_y
            }
            ldr_info_list.append(ldr_data)
        
        # 调用算法处理
        result = self.algorithm.process_ldr_info(ldr_info_list)
        print(f"[ROS] 算法处理结果: {result['message']}")

    def broadcast_resolution_status_for_ldr(self, ldr_id, in_resolution):
        """为指定LDR广播Resolution状态"""
        if in_resolution:
            # 进入Resolution状态，查找对应的无人机
            with self.algorithm.lock:
                if ldr_id in self.algorithm.active_ldrs:
                    uav_ids = self.algorithm.active_ldrs[ldr_id].uav_ids
                    message = f"LDR {ldr_id} 进入Resolution阶段"
                    self.broadcast_resolution_status(uav_ids, ldr_id, in_resolution, message)
        else:
            # 退出Resolution状态
            with self.algorithm.lock:
                if ldr_id in self.algorithm.resolution_ldrs:
                    uav_ids = self.algorithm.resolution_ldrs[ldr_id].uav_ids
                    message = f"LDR {ldr_id} 退出Resolution阶段"
                    self.broadcast_resolution_status(uav_ids, ldr_id, in_resolution, message)

    def _calculate_uav_aabb_with_buffer(self, uav_ids, buffer_distance=2.0):
        """计算无人机位置的AABB包围盒并添加缓存距离"""
        try:
            with self.position_lock:
                # 获取所有无人机的当前位置
                valid_positions = []
                missing_uavs = []
                
                for uav_id in uav_ids:
                    if uav_id in self.uav_positions:
                        valid_positions.append(self.uav_positions[uav_id])
                    else:
                        missing_uavs.append(uav_id)
                        print(f"[ROS] 警告: 无法获取 {uav_id} 的当前位置")
                
                if not valid_positions:
                    print(f"[ROS] 错误: 无法获取任何无人机的位置信息")
                    return None
                
                if missing_uavs:
                    print(f"[ROS] 警告: 缺少以下无人机的位置信息: {missing_uavs}")
                
                # 计算AABB包围盒
                x_coords = [pos[0] for pos in valid_positions]
                y_coords = [pos[1] for pos in valid_positions]
                
                min_x = min(x_coords) - buffer_distance
                max_x = max(x_coords) + buffer_distance
                min_y = min(y_coords) - buffer_distance
                max_y = max(y_coords) + buffer_distance
                
                aabb_info = {
                    'min_x': min_x,
                    'max_x': max_x,
                    'min_y': min_y,
                    'max_y': max_y,
                }
                
                print(f"[ROS] 计算AABB包围盒: ({min_x:.2f}, {min_y:.2f}) -> ({max_x:.2f}, {max_y:.2f})")
                print(f"    包含{len(valid_positions)}/{len(uav_ids)}个无人机位置，缓存距离: {buffer_distance}m")
                
                return aabb_info
                
        except Exception as e:
            print(f"[ROS] 计算AABB包围盒时发生异常: {str(e)}")
            return None

    def broadcast_resolution_status(self, uav_ids, ldr_id, in_resolution, message=""):
        """广播Resolution状态（使用自定义ResolutionStatus消息类型）"""
        try:
            # 创建ResolutionStatus消息
            status_msg = ResolutionStatus()

            # 设置消息头
            # status_msg.header = Header()
            # status_msg.header.stamp = rospy.Time.now()
             # 计算AABB包围盒信息
            aabb_info = self._calculate_uav_aabb_with_buffer(uav_ids, buffer_distance=2.0)
            status_msg.aabb_min_x = aabb_info['min_x']
            status_msg.aabb_max_x = aabb_info['max_x']
            status_msg.aabb_min_y = aabb_info['min_y']
            status_msg.aabb_max_y = aabb_info['max_y']
            # 设置LDR信息
            status_msg.ldr_id = ldr_id
            status_msg.uav_ids = uav_ids
            status_msg.in_resolution = in_resolution
            # status_msg.status_change_time = rospy.Time.now()
            # status_msg.message = message
            
            # 发布消息
            self.resolution_status_pub.publish(status_msg)
            
            status_str = "进入" if in_resolution else "退出"
            print(f"[ROS] 广播{status_str}Resolution阶段 - LDR {ldr_id}: {uav_ids}")
            if message:
                print(f"    消息: {message}")
            
        except Exception as e:
            print(f"[ROS] 广播Resolution状态时发生异常: {str(e)}")

    def broadcast_ldr_completion(self, ldr_id, uav_ids, message=""):
        """新增：广播LDR完成消息"""
        try:
            # 创建LDRCompletion消息
            completion_msg = LDRCompletion()
            
            # 设置消息头
            completion_msg.header = Header()
            completion_msg.header.stamp = rospy.Time.now()
            completion_msg.header.frame_id = "world"
            
            # 设置LDR完成信息
            completion_msg.ldr_id = ldr_id
            completion_msg.uav_ids = uav_ids
            completion_msg.message = message if message else f"LDR {ldr_id} 成功完成解脱"
            
            # 发布消息
            self.ldr_completion_pub.publish(completion_msg)
            
            print(f"[ROS] 广播LDR完成消息 - LDR {ldr_id}: {uav_ids}")
            print(f"    消息: {completion_msg.message}")
            
        except Exception as e:
            print(f"[ROS] 广播LDR完成消息时发生异常: {str(e)}")

    def visualize_ldr_instance(self, ldr_id, uav_ids, aabb_info):
        """新增：可视化LDR实例"""
        try:
            with self.visualization_lock:
                # 创建可视化标记数组
                marker_array = MarkerArray()
                
                # 创建LDR实例标记（红色半透明立方体）
                ldr_marker = Marker()
                ldr_marker.header.frame_id = "world"
                ldr_marker.header.stamp = rospy.Time.now()
                ldr_marker.ns = "ldr_instances"
                ldr_marker.id = ldr_id
                ldr_marker.type = Marker.CUBE
                ldr_marker.action = Marker.ADD
                
                # 计算中心点和尺寸
                center_x = (aabb_info['min_x'] + aabb_info['max_x']) / 2.0
                center_y = (aabb_info['min_y'] + aabb_info['max_y']) / 2.0
                size_x = aabb_info['max_x'] - aabb_info['min_x']
                size_y = aabb_info['max_y'] - aabb_info['min_y']
                
                ldr_marker.pose.position.x = center_x
                ldr_marker.pose.position.y = center_y
                ldr_marker.pose.position.z = 1.0  # 适当高度
                ldr_marker.pose.orientation.w = 1.0
                
                ldr_marker.scale.x = size_x
                ldr_marker.scale.y = size_y
                ldr_marker.scale.z = 0.1  # 薄层
                
                ldr_marker.color.r = 1.0
                ldr_marker.color.g = 0.0
                ldr_marker.color.b = 0.0
                ldr_marker.color.a = 0.5  # 半透明
                
                ldr_marker.lifetime = rospy.Duration(5)  # 持续显示直到删除
                
                marker_array.markers.append(ldr_marker)
                
                # 创建文本标记显示无人机ID
                text_marker = Marker()
                text_marker.header = ldr_marker.header
                text_marker.ns = "ldr_text"
                text_marker.id = ldr_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = center_x
                text_marker.pose.position.y = center_y
                text_marker.pose.position.z = 2.0  # 文字高度
                text_marker.pose.orientation.w = 1.0
                
                text_content = f"LDR {ldr_id}: " + " ".join(uav_ids)
                text_marker.text = text_content
                
                text_marker.scale.z = 0.5  # 文字大小
                text_marker.color.r = 0
                text_marker.color.g = 0
                text_marker.color.b = 0
                text_marker.color.a = 1.0
                
                text_marker.lifetime = rospy.Duration(5)  # 持续显示直到删除
                
                marker_array.markers.append(text_marker)
                
                # 创建AABB边框可视化（绿色线框）
                aabb_marker = Marker()
                aabb_marker.header = ldr_marker.header
                aabb_marker.ns = "aabb_boxes"
                aabb_marker.id = ldr_id
                aabb_marker.type = Marker.LINE_STRIP
                aabb_marker.action = Marker.ADD
                
                aabb_marker.scale.x = 0.1  # 线宽
                aabb_marker.color.r = 0.0
                aabb_marker.color.g = 1.0
                aabb_marker.color.b = 0.0
                aabb_marker.color.a = 1.0
                
                aabb_marker.lifetime = rospy.Duration(5)  # 持续显示直到删除
                
                # 绘制矩形边框
                z_height = 0.5
                points = [
                    Point(aabb_info['min_x'], aabb_info['min_y'], z_height),
                    Point(aabb_info['max_x'], aabb_info['min_y'], z_height),
                    Point(aabb_info['max_x'], aabb_info['max_y'], z_height),
                    Point(aabb_info['min_x'], aabb_info['max_y'], z_height),
                    Point(aabb_info['min_x'], aabb_info['min_y'], z_height)  # 闭合矩形
                ]
                aabb_marker.points = points
                
                # 发布LDR实例可视化
                self.ldr_viz_pub.publish(marker_array)
                
                # 单独发布AABB边框
                aabb_array = MarkerArray()
                aabb_array.markers.append(aabb_marker)
                self.aabb_viz_pub.publish(aabb_array)
                
                # 记录活动的可视化
                self.active_visualizations[ldr_id] = {
                    'uav_ids': uav_ids,
                    'aabb_info': aabb_info,
                    'timestamp': rospy.Time.now()
                }
                
                print(f"[VIZ] LDR {ldr_id} 可视化已发布")
                print(f"    中心点: ({center_x:.2f}, {center_y:.2f})")
                print(f"    尺寸: {size_x:.2f} x {size_y:.2f}")
                print(f"    涉及无人机: {uav_ids}")
                
        except Exception as e:
            print(f"[VIZ] 可视化LDR {ldr_id} 时发生异常: {str(e)}")

    def clear_ldr_visualization(self, ldr_id):
        """新增：清除LDR可视化"""
        try:
            with self.visualization_lock:
                if ldr_id in self.active_visualizations:
                    # 创建删除标记
                    marker_array = MarkerArray()
                    
                    # 删除LDR实例标记
                    delete_marker = Marker()
                    delete_marker.header.frame_id = "world"
                    delete_marker.header.stamp = rospy.Time.now()
                    delete_marker.ns = "ldr_instances"
                    delete_marker.id = ldr_id
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)
                    
                    # 删除文本标记
                    delete_text = Marker()
                    delete_text.header = delete_marker.header
                    delete_text.ns = "ldr_text"
                    delete_text.id = ldr_id
                    delete_text.action = Marker.DELETE
                    marker_array.markers.append(delete_text)
                    
                    # 删除AABB边框标记
                    delete_aabb = Marker()
                    delete_aabb.header = delete_marker.header
                    delete_aabb.ns = "aabb_boxes"
                    delete_aabb.id = ldr_id
                    delete_aabb.action = Marker.DELETE
                    
                    # 发布删除标记
                    self.ldr_viz_pub.publish(marker_array)
                    
                    aabb_array = MarkerArray()
                    aabb_array.markers.append(delete_aabb)
                    self.aabb_viz_pub.publish(aabb_array)
                    
                    # 从活动可视化记录中移除
                    del self.active_visualizations[ldr_id]
                    
                    print(f"[VIZ] LDR {ldr_id} 可视化已清除")
                else:
                    print(f"[VIZ] 警告: LDR {ldr_id} 的可视化不存在")
                    
        except Exception as e:
            print(f"[VIZ] 清除LDR {ldr_id} 可视化时发生异常: {str(e)}")

    def clear_all_visualizations(self):
        """新增：清除所有LDR可视化"""
        try:
            with self.visualization_lock:
                ldr_ids_to_clear = list(self.active_visualizations.keys())
                for ldr_id in ldr_ids_to_clear:
                    self.clear_ldr_visualization(ldr_id)
                print(f"[VIZ] 已清除所有可视化 ({len(ldr_ids_to_clear)} 个)")
        except Exception as e:
            print(f"[VIZ] 清除所有可视化时发生异常: {str(e)}")

    def resolve_ldr(self, ldr_id):
        """解决LDR冲突（委托给算法模块）"""
        print(f"\n[ROS] 开始解决LDR {ldr_id} 的冲突")
        
        # 新增：在开始解决LDR时发布可视化
        try:
            with self.algorithm.lock:
                if ldr_id in self.algorithm.active_ldrs:
                    ldr_info = self.algorithm.active_ldrs[ldr_id]
                    uav_ids = ldr_info.uav_ids
                    
                    # 计算AABB包围盒
                    aabb_info = self._calculate_uav_aabb_with_buffer(uav_ids, buffer_distance=2.0)
                    if aabb_info:
                        # 发布可视化
                        self.visualize_ldr_instance(ldr_id, uav_ids, aabb_info)
                        print(f"[VIZ] LDR {ldr_id} 开始解决，可视化已启动")
                    else:
                        print(f"[VIZ] 警告: 无法计算LDR {ldr_id} 的AABB信息")
        except Exception as e:
            print(f"[VIZ] 启动LDR {ldr_id} 可视化时发生异常: {str(e)}")
        
        # 定义路径规划回调函数
        def path_planning_callback(start, goal):
            return self.call_path_planning_service(start, goal)
        
        # 调用算法解决冲突
        result = self.algorithm.resolve_ldr_conflict(ldr_id, path_planning_callback)
        
        if result['success']:
            print("[ROS] 算法解决成功，调用路径跟踪服务")
            tracking_success = self.call_path_tracking_service(result['paths'], result['ldr_id'], result['uav_ids'])
            
            if tracking_success:
                print("[ROS] 路径跟踪成功完成")
                # 处理完成后的操作
                self._handle_ldr_completion(result['ldr_id'], result['uav_ids'])
            else:
                print("[ROS] 路径跟踪失败")
                # 如果跟踪失败，清除可视化并让算法退出Resolution状态
                self.clear_ldr_visualization(ldr_id)
                self.algorithm._exit_resolution_state(ldr_id)
                self.broadcast_resolution_status_for_ldr(ldr_id, False)
        else:
            print(f"[ROS] 算法解决失败: {result['message']}")
            # 清除可视化
            self.clear_ldr_visualization(ldr_id)
            # 广播退出Resolution状态
            if 'uav_ids' in result:
                self.broadcast_resolution_status(result['uav_ids'], ldr_id, False, f"算法解决失败: {result['message']}")

    def call_path_tracking_service(self, paths, ldr_id, uav_ids):
        """调用路径跟踪服务"""
        print(f"\n[ROS] 调用路径跟踪服务 - LDR {ldr_id}")
        
        try:
            # 构建服务请求
            req = PathTrackingRequest()
            
            # 设置消息头
            req.header = Header()
            req.header.stamp = rospy.Time.now()
            req.header.frame_id = "world"
            
            # 设置LDR信息
            req.ldr_id = ldr_id
            req.uav_ids = uav_ids
            
            # 转换路径信息
            uav_path_msgs = []
            for uav_id in uav_ids:
                if uav_id in paths:
                    path_info = paths[uav_id]
                    
                    uav_path_msg = UAVPath()
                    uav_path_msg.uav_id = uav_id
                    
                    # 转换路径点
                    path_points = []
                    for point in path_info['path']:
                        ros_point = Point()
                        ros_point.x = float(point[0])
                        ros_point.y = float(point[1])
                        ros_point.z = 1.5  # 默认高度
                        path_points.append(ros_point)
                    
                    uav_path_msg.path_points = path_points
                    
                    # 设置时间索引
                    if 'time_indices' in path_info:
                        uav_path_msg.time_indices = path_info['time_indices']
                        print(f"  无人机 {uav_id}: 时间索引包含 {len(path_info['time_indices'])} 个时间点")
                    else:
                        print(f"  警告: 无人机 {uav_id} 缺少时间索引信息")
                    
                    uav_path_msgs.append(uav_path_msg)
                    print(f"  无人机 {uav_id}: 路径包含 {len(path_info['path'])} 个航点")
                else:
                    print(f"  警告: 无人机 {uav_id} 未找到路径信息")
            
            req.uav_paths = uav_path_msgs
            
            # 调用服务
            print(f"[ROS] 正在调用路径跟踪服务...")
            resp = self.path_tracking_client(req)
            
            if resp.success:
                print(f"[ROS] 路径跟踪服务调用成功")
                print(f"  执行时间: {resp.execution_time:.2f}秒")
                print(f"  返回消息: {resp.message}")
                return True
            else:
                print(f"[ROS] 路径跟踪服务调用失败: {resp.message}")
                return False
                
        except rospy.ServiceException as e:
            print(f"[ROS] 路径跟踪服务调用异常: {str(e)}")
            return False
        except Exception as e:
            print(f"[ROS] 构建服务请求时发生异常: {str(e)}")
            return False

    def _handle_ldr_completion(self, ldr_id, uav_ids):
        """处理LDR完成后的操作"""
        print(f"\n[ROS] 处理LDR {ldr_id} 完成后的操作")
        
        # 新增：清除LDR可视化
        self.clear_ldr_visualization(ldr_id)
        print(f"[VIZ] LDR {ldr_id} 完成，可视化已清除")
        
        # 调用算法完成LDR解决
        result = self.algorithm.complete_ldr_resolution(ldr_id, uav_ids)
        
        if result['success']:
            # 新增：在删除LDR之前，先广播LDR完成消息
            completion_message = f"LDR {ldr_id} 成功完成解脱，涉及无人机: {', '.join(uav_ids)}"
            self.broadcast_ldr_completion(ldr_id, uav_ids, completion_message)
            
            # 广播这些无人机退出Resolution阶段
            self.broadcast_resolution_status(uav_ids, ldr_id, False, "LDR解决完成")
            
            # 操作1: 为每个无人机调用start_mission服务
            self._call_mission_services(uav_ids, ldr_id)
            
            # 操作2: 为所有无人机发布needstop=false
            self._publish_stop_signals(uav_ids, False)
            
            print(f"[ROS] LDR {ldr_id} 已完成处理")
        else:
            print(f"[ROS] 警告: {result['message']}")

    def _call_mission_services(self, uav_ids, ldr_id):
        """为无人机调用start_mission服务"""
        print(f"[ROS] 为LDR {ldr_id} 中的无人机调用任务开始服务")
        
        for uav_id in uav_ids:
            try:
                # 获取无人机当前位置和目标位置
                current_pos = self._get_uav_current_position(uav_id)
                goal_pos = self.algorithm.default_goals.get(uav_id, (0, 0))
                
                print(f"  {uav_id}: 当前位置{current_pos} -> 目标位置{goal_pos}")
                
                # 修正：设置ROS参数到正确的命名空间
                # C++节点使用私有句柄，完整路径是 /uav_id/mission_control/param_name
                param_prefix = f"/{uav_id}/mission_control/"
                rospy.set_param(f"{param_prefix}start_x", current_pos[0])
                rospy.set_param(f"{param_prefix}start_y", current_pos[1])
                rospy.set_param(f"{param_prefix}goal_x", goal_pos[0])
                rospy.set_param(f"{param_prefix}goal_y", goal_pos[1])
                
                print(f"  {uav_id}: 已设置参数到 {param_prefix}")
                
                # 调用服务
                if uav_id in self.mission_clients and self.mission_clients[uav_id] is not None:
                    req = EmptyRequest()
                    resp = self.mission_clients[uav_id](req)
                    print(f"  {uav_id}: 任务服务调用成功")
                else:
                    # 尝试重新连接服务
                    service_name = f"/{uav_id}/start_mission"
                    try:
                        rospy.wait_for_service(service_name, timeout=1.0)
                        self.mission_clients[uav_id] = rospy.ServiceProxy(service_name, Empty)
                        req = EmptyRequest()
                        resp = self.mission_clients[uav_id](req)
                        print(f"  {uav_id}: 重连服务并调用成功")
                    except rospy.ROSException:
                        print(f"  {uav_id}: 无法连接任务服务 {service_name}")
                        
            except Exception as e:
                print(f"  {uav_id}: 调用任务服务失败 - {str(e)}")

    def _get_uav_current_position(self, uav_id):
        """获取无人机当前位置"""
        try:
            with self.position_lock:
                if uav_id in self.uav_positions:
                    return self.uav_positions[uav_id]
            
            # 如果找不到，返回默认起点
            default_starts = {
                'uav1': (-9.0, -9.0),
                'uav2': (9.0, 9.0),
                'uav3': (9.0, -9.0),
                'uav4': (-9.0, 9.0)
            }
            return default_starts.get(uav_id, (0, 0))
            
        except Exception as e:
            print(f"[ROS] 获取 {uav_id} 当前位置失败: {str(e)}")
            return (0, 0)

    def _publish_stop_signals(self, uav_ids, stop_signal):
        """为无人机发布停止信号"""
        print(f"[ROS] 为无人机发布停止信号: {stop_signal}")
        
        for uav_id in uav_ids:
            try:
                if uav_id in self.stop_publishers:
                    msg = Bool()
                    msg.data = stop_signal
                    self.stop_publishers[uav_id].publish(msg)
                    print(f"  {uav_id}: 发布needstop={stop_signal}")
                else:
                    print(f"  {uav_id}: 停止信号发布者未找到")
            except Exception as e:
                print(f"  {uav_id}: 发布停止信号失败 - {str(e)}")

    def call_path_planning_service(self, start, goal):
        """调用路径规划服务获取路径"""
        try:
            req = PathPlanRequest()
            req.start_x = start[0]
            req.start_y = start[1]
            req.goal_x = goal[0]
            req.goal_y = goal[1]
            
            print(f"[ROS] 路径规划服务调用: 起点({start[0]:.2f}, {start[1]:.2f}) -> 终点({goal[0]:.2f}, {goal[1]:.2f})")
            resp = self.path_plan_client(req)
            
            if resp.success:
                path_points = [(p.x, p.y) for p in resp.path]
                print(f"[ROS] 路径规划成功，获得{len(path_points)}个航点")
                return path_points
            else:
                print(f"[ROS] 路径规划失败: {resp.message}")
                return None
        except rospy.ServiceException as e:
            print(f"[ROS] 路径规划服务调用失败: {str(e)}")
            return None

    def get_system_status(self):
        """获取系统状态（用于调试）"""
        status = self.algorithm.get_status()
        print(f"[ROS] 系统状态:")
        print(f"  活动LDR数量: {status['active_ldrs_count']}")
        print(f"  Resolution LDR数量: {status['resolution_ldrs_count']}")
        print(f"  Resolution中的无人机: {status['resolution_uavs']}")
        print(f"  活动LDR ID: {status['active_ldr_ids']}")
        print(f"  Resolution LDR ID: {status['resolution_ldr_ids']}")
        
        # 新增：可视化状态
        with self.visualization_lock:
            print(f"  活动可视化数量: {len(self.active_visualizations)}")
            if self.active_visualizations:
                print(f"  可视化LDR ID: {list(self.active_visualizations.keys())}")
        
        return status

    def shutdown(self):
        """新增：关闭时清理资源"""
        print("[ROS] LDR解脱器正在关闭...")
        
        # 清除所有可视化
        self.clear_all_visualizations()
        
        print("[ROS] 资源清理完成")


if __name__ == '__main__':
    try:
        print("===== LDR冲突解脱ROS系统启动 (带可视化功能) =====")
        resolver = LDRResolver()
        
        # 可选：定期打印系统状态（用于调试）
        def status_timer():
            while not rospy.is_shutdown():
                time.sleep(10)  # 每10秒打印一次状态
                resolver.get_system_status()
        
        # 启动状态监控线程（可选）
        # status_thread = threading.Thread(target=status_timer)
        # status_thread.daemon = True
        # status_thread.start()
        
        print("[ROS] LDR解脱器准备就绪，等待LDR信息...")
        
        # 注册关闭处理函数
        def signal_handler():
            resolver.shutdown()
        
        rospy.on_shutdown(signal_handler)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("[ROS] 节点被中断")
    except Exception as e:
        print(f"[ROS] 启动失败: {str(e)}")
    finally:
        print("[ROS] LDR冲突解脱系统关闭")