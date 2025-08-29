#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LDR冲突解脱核心算法模块
包含LDR状态管理、路径规划算法、几何计算等核心逻辑
"""

import numpy as np
import time
import random
import threading
from copy import deepcopy
from shapely.geometry import Polygon, LineString, Point as ShapelyPoint
import sys
import os

# 添加路径规划器路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from path_planner import UAVPathPlannerComplete


class LDRState:
    """LDR状态类"""
    def __init__(self):
        self.id = None
        self.uav_ids = []
        self.positions = {}
        self.aabb = {}
        self.last_update = None
        self.resolved = False
        self.in_resolution = False  # 标记是否处于Resolution阶段


class LDRAlgorithm:
    """LDR冲突解脱核心算法类"""
    
    def __init__(self, stability_threshold=4.0, safety_distance=1):
        """
        初始化算法参数
        
        Args:
            stability_threshold: 稳定性阈值（秒）
            safety_distance: 安全距离（米）
        """
        self.stability_threshold = stability_threshold
        self.safety_distance = safety_distance
        
        # LDR状态存储
        self.active_ldrs = {}  # {ldr_id: LDRState}
        self.resolution_ldrs = {}  # {ldr_id: LDRState} 处于Resolution阶段的LDR
        self.resolution_uavs = set()  # 跟踪当前处于resolution状态的无人机
        self.ldr_counter = 0
        self.lock = threading.Lock()
        
        # 障碍物定义
        self.obstacles = [
            [(-0.6, 1), (-0.6, 10), (0.6, 10), (0.6, 1)],
            [(-0.6, -1), (0.6, -1), (0.6, -10), (-0.6, -10)]
        ]
        self.obstacle_polys = [Polygon(obs) for obs in self.obstacles]
        
        # 默认目标点
        self.default_goals = {
            'uav1': (9.0, 9.0),
            'uav2': (-9.0, -9.0),
            'uav3': (-9.0, 9.0),
            'uav4': (9.0, -9.0)
        }
        
        print(f"[算法初始化] 稳定阈值={self.stability_threshold}s, 安全距离={self.safety_distance}m")
        print("[算法初始化] 障碍物和默认目标点加载完成")

    def process_ldr_info(self, ldr_info_list):
        """
        处理LDR信息列表
        
        Args:
            ldr_info_list: LDR信息列表，每个元素包含 {uav_ids, positions, min_x, max_x, min_y, max_y}
        
        Returns:
            dict: 处理结果 {status: str, message: str}
        """
        print(f"\n[算法] 收到LDR信息，包含{len(ldr_info_list)}个LDR实例")
        
        with self.lock:
            for ldr_info in ldr_info_list:
                self._process_single_ldr(ldr_info)
            self._check_ldr_relationships()
        
        return {"status": "success", "message": f"处理了{len(ldr_info_list)}个LDR"}

    def _process_single_ldr(self, ldr_info):
        """处理单个LDR信息"""
        print(f"[算法] 正在处理LDR，无人机列表: {ldr_info['uav_ids']}")
        
        # 检查是否包含处于resolution状态的无人机
        resolution_uavs_in_new_ldr = set(ldr_info['uav_ids']) & self.resolution_uavs
        if resolution_uavs_in_new_ldr:
            print(f"[算法] 新LDR包含正在resolution中的无人机 {list(resolution_uavs_in_new_ldr)}，忽略此LDR")
            return
        
        # 查找已存在的LDR
        existing_id = None
        for ldr_id, ldr in self.active_ldrs.items():
            if set(ldr.uav_ids) == set(ldr_info['uav_ids']):
                existing_id = ldr_id
                break
        
        if existing_id is not None:
            print(f"[算法] 发现已有LDR {existing_id}，检查是否可以更新")
            if self.active_ldrs[existing_id].in_resolution:
                print(f"[算法] LDR {existing_id} 处于Resolution阶段，忽略位置更新")
                return
            else:
                self._update_ldr(existing_id, ldr_info)
        else:
            print("[算法] 未找到匹配LDR，创建新实例")
            self._create_new_ldr(ldr_info)

    def _update_ldr(self, ldr_id, ldr_info):
        """更新现有LDR"""
        print(f"[算法] 更新LDR {ldr_id} 的数据")
        ldr = self.active_ldrs[ldr_id]
        
        if ldr.in_resolution:
            print(f"[算法] LDR {ldr_id} 处于Resolution阶段，忽略更新")
            return
        
        ldr.last_update = time.time()
        ldr.positions = {uav_id: pos for uav_id, pos in zip(ldr_info['uav_ids'], ldr_info['positions'])}
        ldr.aabb = {
            'min_x': ldr_info['min_x'],
            'max_x': ldr_info['max_x'],
            'min_y': ldr_info['min_y'],
            'max_y': ldr_info['max_y']
        }
        ldr.resolved = False
        
        print(f"[算法] LDR {ldr_id} 数据已刷新，重置稳定性计时")

    def _create_new_ldr(self, ldr_info):
        """创建新LDR"""
        new_ldr = LDRState()
        new_ldr.id = self.ldr_counter
        new_ldr.uav_ids = ldr_info['uav_ids']
        new_ldr.positions = {uav_id: pos for uav_id, pos in zip(ldr_info['uav_ids'], ldr_info['positions'])}
        new_ldr.aabb = {
            'min_x': ldr_info['min_x'],
            'max_x': ldr_info['max_x'],
            'min_y': ldr_info['min_y'],
            'max_y': ldr_info['max_y']
        }
        new_ldr.last_update = time.time()
        new_ldr.resolved = False
        new_ldr.in_resolution = False
        
        self.active_ldrs[self.ldr_counter] = new_ldr
        print(f"[算法] 新LDR {self.ldr_counter} 创建成功，包含无人机: {new_ldr.uav_ids}")
        self.ldr_counter += 1

    def _check_ldr_relationships(self):
        """检查LDR之间的关系，处理包含关系"""
        print("[算法] 开始检查LDR之间的包含关系")
        
        to_remove = set()
        
        for ldr_id, ldr in list(self.active_ldrs.items()):
            for other_id, other_ldr in list(self.active_ldrs.items()):
                if ldr_id == other_id:
                    continue
                
                if set(ldr.uav_ids).issubset(set(other_ldr.uav_ids)):
                    print(f"[算法] LDR {ldr_id} 的无人机 {ldr.uav_ids} 是 LDR {other_id} {other_ldr.uav_ids} 的子集，将被移除")
                    to_remove.add(ldr_id)
                    break
        
        for ldr_id in to_remove:
            self.active_ldrs.pop(ldr_id, None)
            print(f"[算法] 已移除LDR {ldr_id}")

    def check_stability(self):
        """
        检查LDR是否稳定
        
        Returns:
            list: 需要解决的LDR ID列表
        """
        current_time = time.time()
        to_resolve = []
        
        with self.lock:
            for ldr_id, ldr in list(self.active_ldrs.items()):
                if ldr.in_resolution:
                    continue
                    
                elapsed = current_time - ldr.last_update
                
                if elapsed >= self.stability_threshold and not ldr.resolved:
                    to_resolve.append(ldr_id)
                    ldr.resolved = True
                    ldr.in_resolution = True
                    
                    # 将这些无人机添加到resolution集合中
                    for uav_id in ldr.uav_ids:
                        self.resolution_uavs.add(uav_id)
                    
                    print(f"[算法] LDR {ldr_id} 已达到稳定状态，准备解决冲突")
        
        return to_resolve

    def resolve_ldr_conflict(self, ldr_id, path_planning_callback=None):
        """
        解决LDR冲突
        
        Args:
            ldr_id: LDR ID
            path_planning_callback: 路径规划回调函数，格式为 callback(start, goal) -> path_points
        
        Returns:
            dict: 解决结果 {success: bool, paths: dict, message: str, ldr_id: int, uav_ids: list}
        """
        print(f"\n[算法] 开始解决LDR {ldr_id} 的冲突")
        
        with self.lock:
            if ldr_id not in self.active_ldrs:
                return {"success": False, "message": f"LDR {ldr_id} 已不存在", "ldr_id": ldr_id}
            
            ldr = self.active_ldrs[ldr_id]
            if not ldr.in_resolution:
                return {"success": False, "message": f"LDR {ldr_id} 不在Resolution阶段", "ldr_id": ldr_id}
        
        try:
            # 准备路径规划输入
            print(f"[算法] 准备{len(ldr.uav_ids)}架无人机的路径规划输入")
            uav_paths = []
            
            for uav_id in ldr.uav_ids:
                start = ldr.positions[uav_id]
                goal = self.default_goals.get(uav_id, (0, 0))
                
                print(f"[算法] 无人机 {uav_id} 从 {start} 到 {goal}")
                
                # 如果提供了路径规划回调，使用它获取路径
                if path_planning_callback:
                    path = path_planning_callback(start, goal)
                    if path:
                        print(f"[算法] 无人机 {uav_id} 获得路径，包含{len(path)}个航点")
                        intersection = self.find_path_aabb_intersection(path, ldr.aabb)
                        adjusted_goal = intersection if intersection else goal
                    else:
                        adjusted_goal = goal
                        print(f"[算法] 无人机 {uav_id} 路径规划失败，使用默认目标")
                else:
                    adjusted_goal = goal
                
                uav_paths.append({
                    'id': uav_id,
                    'start': start,
                    'original_goal': goal,
                    'adjusted_goal': adjusted_goal
                })
            
            # 调整目标点以满足安全约束
            print("[算法] 调整目标点以满足安全约束")
            self.adjust_goals(uav_paths, ldr)
            
            # 执行路径规划
            print("[算法] 执行路径规划")
            planner = UAVPathPlannerComplete(
                obstacles=self.obstacles,
                num_particles=15,
                waypoints=10,
                max_iterations=100,
                min_distance=self.safety_distance,
                enable_optimization=True
            )
            
            planning_data = [{
                'id': p['id'],
                'start': p['start'],
                'goal': p['adjusted_goal'] 
            } for p in uav_paths]
            
            result = planner.plan_and_optimize_paths(planning_data)
            
            if result['success']:
                print("[算法] 路径规划完成")
                
                # 将LDR移至resolution_ldrs
                with self.lock:
                    self.resolution_ldrs[ldr_id] = ldr
                
                return {
                    "success": True,
                    "paths": result['final_paths'],
                    "message": "路径规划成功",
                    "ldr_id": ldr_id,
                    "uav_ids": ldr.uav_ids
                }
            else:
                print("[算法] 路径规划失败")
                self._exit_resolution_state(ldr_id)
                return {
                    "success": False,
                    "message": "路径规划失败",
                    "ldr_id": ldr_id,
                    "uav_ids": ldr.uav_ids
                }
            
        except Exception as e:
            print(f"[算法] 解决LDR {ldr_id} 时发生异常: {str(e)}")
            self._exit_resolution_state(ldr_id)
            return {
                "success": False,
                "message": f"解决冲突时发生异常: {str(e)}",
                "ldr_id": ldr_id
            }

    def _exit_resolution_state(self, ldr_id):
        """退出Resolution状态"""
        with self.lock:
            if ldr_id in self.active_ldrs:
                ldr = self.active_ldrs[ldr_id]
                ldr.in_resolution = False
                # 从resolution集合中移除这些无人机
                for uav_id in ldr.uav_ids:
                    self.resolution_uavs.discard(uav_id)

    def complete_ldr_resolution(self, ldr_id, uav_ids):
        """
        完成LDR解决
        
        Args:
            ldr_id: LDR ID
            uav_ids: 完成的无人机ID列表
        
        Returns:
            dict: 完成结果 {success: bool, message: str}
        """
        print(f"\n[算法] 完成LDR {ldr_id} 解决，无人机: {uav_ids}")
        
        with self.lock:
            if ldr_id in self.resolution_ldrs:
                # 从resolution状态中移除这些无人机
                for uav_id in uav_ids:
                    self.resolution_uavs.discard(uav_id)
                
                # 从resolution_ldrs中移除
                del self.resolution_ldrs[ldr_id]
                
                # 也从active_ldrs中移除（如果还在的话）
                if ldr_id in self.active_ldrs:
                    del self.active_ldrs[ldr_id]
                
                print(f"[算法] LDR {ldr_id} 已完成，移除Resolution状态")
                return {"success": True, "message": f"LDR {ldr_id} 已完成"}
            else:
                print(f"[算法] 警告：LDR {ldr_id} 不在Resolution列表中")
                return {"success": False, "message": f"LDR {ldr_id} 不在Resolution列表中"}

    def find_path_aabb_intersection(self, path_points, aabb):
        """计算路径与AABB包围盒的交点"""
        print(f"[算法] AABB边界: min_x={aabb['min_x']:.2f}, max_x={aabb['max_x']:.2f}, min_y={aabb['min_y']:.2f}, max_y={aabb['max_y']:.2f}")
        
        aabb_edges = [
            ((aabb['min_x'], aabb['min_y']), (aabb['min_x'], aabb['max_y'])),
            ((aabb['max_x'], aabb['min_y']), (aabb['max_x'], aabb['max_y'])),
            ((aabb['min_x'], aabb['min_y']), (aabb['max_x'], aabb['min_y'])),
            ((aabb['min_x'], aabb['max_y']), (aabb['max_x'], aabb['max_y']))
        ]
        
        end_point = path_points[-1]
        if (aabb['min_x'] <= end_point[0] <= aabb['max_x'] and 
            aabb['min_y'] <= end_point[1] <= aabb['max_y']):
            print("[算法] 路径终点已在AABB内，直接使用终点")
            return (end_point[0], end_point[1])
        
        all_intersections = []
        
        for i in range(len(path_points)-1):
            segment_start = path_points[i]
            segment_end = path_points[i+1]
            
            for j, (edge_start, edge_end) in enumerate(aabb_edges):
                intersection = self.line_segment_intersection(segment_start, segment_end, edge_start, edge_end)
                if intersection:
                    all_intersections.append((intersection, i))
        
        if all_intersections:
            start_point = path_points[0]
            closest_intersection = min(all_intersections, 
                                     key=lambda x: self.distance(start_point, x[0]))
            print(f"[算法] 选择最近交点: ({closest_intersection[0][0]:.2f}, {closest_intersection[0][1]:.2f})")
            return closest_intersection[0]
        
        print("[算法] 未找到交点，计算最近边界点")
        closest_point = self.find_closest_boundary_point(path_points[0], aabb)
        return closest_point

    def line_segment_intersection(self, p1, p2, p3, p4):
        """计算两条线段的交点"""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        
        if abs(denom) < 1e-10:
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        
        if 0 <= t <= 1 and 0 <= u <= 1:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return (x, y)
        
        return None

    def find_closest_boundary_point(self, point, aabb):
        """找到离给定点最近的AABB边界点"""
        x, y = point
        min_x, max_x = aabb['min_x'], aabb['max_x']
        min_y, max_y = aabb['min_y'], aabb['max_y']
        
        candidates = [
            (min_x, max(min_y, min(max_y, y))),
            (max_x, max(min_y, min(max_y, y))),
            (max(min_x, min(max_x, x)), min_y),
            (max(min_x, min(max_x, x)), max_y)
        ]
        
        closest = min(candidates, key=lambda p: (p[0]-x)**2 + (p[1]-y)**2)
        return closest

    def adjust_goals(self, uav_paths, ldr):
        """调整目标点以满足安全约束"""
        print("  [算法] 开始调整目标点位置")
        
        changed = True
        iteration = 0
        while changed:
            iteration += 1
            changed = False
            print(f"  [算法] 第{iteration}轮调整开始")
            
            # 检查无人机之间的距离
            for i in range(len(uav_paths)):
                for j in range(i+1, len(uav_paths)):
                    g1 = uav_paths[i]['adjusted_goal']
                    g2 = uav_paths[j]['adjusted_goal']
                    dist = self.distance(g1, g2)
                    
                    if dist < self.safety_distance:
                        print(f"    无人机 {uav_paths[i]['id']} 和 {uav_paths[j]['id']} 目标距离 {dist:.2f}m < 安全距离")
                        if random.random() < 0.5:
                            new_goal = self.move_goal_along_boundary(uav_paths[i]['adjusted_goal'], ldr.aabb)
                            uav_paths[i]['adjusted_goal'] = new_goal
                        else:
                            new_goal = self.move_goal_along_boundary(uav_paths[j]['adjusted_goal'], ldr.aabb)
                            uav_paths[j]['adjusted_goal'] = new_goal
                        changed = True
            
            # 检查与障碍物的距离
            for p in uav_paths:
                goal_point = ShapelyPoint(p['adjusted_goal'])
                too_close = any(goal_point.distance(obs) <= 0 for obs in self.obstacle_polys)
                
                if too_close:
                    print(f"    无人机 {p['id']} 目标点太靠近障碍物")
                    p['adjusted_goal'] = self.move_goal_along_boundary(p['adjusted_goal'], ldr.aabb)
                    changed = True
            
            if iteration > 20:
                print("  [算法] 目标点调整达到最大迭代次数")
                break

    def move_goal_along_boundary(self, goal, aabb):
        """沿AABB边界移动目标点"""
        x, y = goal
        step = 0.5
        
        if abs(x - aabb['min_x']) < 0.1:
            return (aabb['min_x'], y + step)
        elif abs(x - aabb['max_x']) < 0.1:
            return (aabb['max_x'], y - step)
        elif abs(y - aabb['min_y']) < 0.1:
            return (x - step, aabb['min_y'])
        elif abs(y - aabb['max_y']) < 0.1:
            return (x + step, aabb['max_y'])
        else:
            dists = [
                (abs(x - aabb['min_x']), 'left'),
                (abs(x - aabb['max_x']), 'right'),
                (abs(y - aabb['min_y']), 'bottom'),
                (abs(y - aabb['max_y']), 'top')
            ]
            _, closest = min(dists)
            
            if closest == 'left':
                return (aabb['min_x'], y)
            elif closest == 'right':
                return (aabb['max_x'], y)
            elif closest == 'bottom':
                return (x, aabb['min_y'])
            else:
                return (x, aabb['max_y'])

    def distance(self, p1, p2):
        """计算两点间距离"""
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def get_status(self):
        """
        获取当前状态
        
        Returns:
            dict: 状态信息
        """
        with self.lock:
            return {
                "active_ldrs_count": len(self.active_ldrs),
                "resolution_ldrs_count": len(self.resolution_ldrs),
                "resolution_uavs": list(self.resolution_uavs),
                "active_ldr_ids": list(self.active_ldrs.keys()),
                "resolution_ldr_ids": list(self.resolution_ldrs.keys())
            }