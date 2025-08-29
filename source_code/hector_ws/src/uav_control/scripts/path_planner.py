import numpy as np
import math
import time
import random
from shapely.geometry import LineString, Polygon
from shapely.ops import nearest_points


class Particle:
    def __init__(self, uav_count, waypoints, start_positions, goal_positions, bounds, obstacles=None):
        self.uav_count = uav_count
        self.waypoints = waypoints
        self.dimensions = 2
        self.position = np.zeros((uav_count, waypoints, self.dimensions))
        self.obstacles = obstacles

        self.obstacle_polygons = []
        if obstacles:
            for obstacle in obstacles:
                self.obstacle_polygons.append(Polygon(obstacle))

        for i in range(uav_count):
            self.position[i] = self.initialize_path(start_positions[i], goal_positions[i], waypoints, bounds, obstacles)

        self.velocity = np.random.uniform(-1, 1, (uav_count, waypoints, self.dimensions))
        self.best_position = self.position.copy()
        self.best_fitness = float('inf')
        self.fitness = self.calculate_fitness(start_positions, goal_positions)

        if self.fitness < self.best_fitness:
            self.best_fitness = self.fitness
            self.best_position = self.position.copy()

    def initialize_path(self, start, goal, waypoints, bounds, obstacles=None):
        dim = waypoints - 2
        if dim <= 0:
            return np.array([start, goal])

        path = np.zeros((waypoints, 2))
        path[0] = start
        path[-1] = goal

        x_min = min(start[0], goal[0])
        x_max = max(start[0], goal[0])
        y_min = min(start[1], goal[1])
        y_max = max(start[1], goal[1])

        if bounds is not None:
            x_min = max(x_min, bounds[0][0])
            x_max = min(x_max, bounds[1][0])
            y_min = max(y_min, bounds[0][1])
            y_max = min(y_max, bounds[1][1])

        x_values = np.random.uniform(x_min, x_max, dim)
        x_values.sort()
        y_values = np.random.uniform(y_min, y_max, dim)
        y_values.sort()

        if start[0] > goal[0]:
            x_values = x_values[::-1]
        if start[1] > goal[1]:
            y_values = y_values[::-1]

        for i in range(dim):
            path[i + 1] = [x_values[i], y_values[i]]

        return path

    def calculate_fitness(self, start_positions, goal_positions):
        total_fitness = 0
        path_length = 0
        for i in range(self.uav_count):
            for j in range(1, self.waypoints):
                path_length += np.linalg.norm(self.position[i, j] - self.position[i, j - 1]) * 100

        collision_penalty = self.check_collisions()
        # dynamics_penalty = self.check_dynamics_constraints()
        uniformity_penalty = self.path_length_uniformity_fitness()
        total_fitness = collision_penalty +path_length+uniformity_penalty

        return total_fitness

    def check_collisions(self):
        collision_penalty = 0
        safety_distance = 1.0

        for i in range(self.uav_count):
            for j in range(i + 1, self.uav_count):
                for t in range(self.waypoints - 1):
                    seg1 = self.position[i, t:t + 2]
                    seg2 = self.position[j, t:t + 2]

                    line1 = LineString([(seg1[0][0], seg1[0][1]), (seg1[1][0], seg1[1][1])])
                    line2 = LineString([(seg2[0][0], seg2[0][1]), (seg2[1][0], seg2[1][1])])
                    min_dist = line1.distance(line2)

                    if min_dist < safety_distance:
                            collision_penalty += 3000

        if self.obstacles:
            obstacle_polygons = [Polygon(obstacle) for obstacle in self.obstacles]
            for i in range(self.uav_count):
                for t in range(self.waypoints - 1):
                    seg = self.position[i, t:t + 2]
                    line = LineString([(seg[0][0], seg[0][1]), (seg[1][0], seg[1][1])])

                    for obstacle_poly in obstacle_polygons:
                        if line.intersects(obstacle_poly):
                            collision_penalty += 3000

        return collision_penalty

    def check_dynamics_constraints(self):
        dynamics_penalty = 0
        v_max = 2.0
        a_max = 1.0
        max_turn_angle = np.pi / 4
        dt = 2

        for i in range(self.uav_count):
            for t in range(1, self.waypoints):
                displacement = self.position[i, t] - self.position[i, t - 1]
                v = displacement / dt
                v_norm = np.linalg.norm(v)

                if v_norm > v_max:
                    dynamics_penalty += (v_norm - v_max) * 100

                if t > 1:
                    v_prev = (self.position[i, t - 1] - self.position[i, t - 2]) / dt
                    a = (v - v_prev) / dt
                    a_norm = np.linalg.norm(a)

                    if a_norm > a_max:
                        dynamics_penalty += (a_norm - a_max) * 100

                    if v_norm > 0.0 and np.linalg.norm(v_prev) > 0.0:
                        cos_angle = np.dot(v, v_prev) / (v_norm * np.linalg.norm(v_prev))
                        angle = np.arccos(np.clip(cos_angle, -1, 1))

                        if angle > max_turn_angle:
                            dynamics_penalty += (angle - max_turn_angle) * 100
        return dynamics_penalty

    def path_length_uniformity_fitness(self):
        uniformity_penalty = 0
        for i in range(self.position.shape[0]):
            path = self.position[i]
            segments = path[1:] - path[:-1]
            segment_lengths = np.linalg.norm(segments, axis=1)

            if len(segment_lengths) <= 1:
                continue

            std_dev = np.std(segment_lengths)
            mean_length = np.mean(segment_lengths)
            if mean_length > 0:
                cv = std_dev / mean_length
                uniformity_penalty += 10 * cv ** 2

        return uniformity_penalty

    def apply_rotation_based_avoidance(self, position, bounds, max_iterations=3):
        corrected_position = position.copy()
        if not self.obstacle_polygons:
            return corrected_position

        for iteration in range(max_iterations):
            intersections_found = False

            for uav_idx in range(self.uav_count):
                for seg_idx in range(self.waypoints - 1):
                    intersection_info = self.detect_segment_intersection(corrected_position, uav_idx, seg_idx)

                    if intersection_info['intersecting']:
                        intersections_found = True
                        corrected_position = self.rotate_segment_to_avoid_obstacle(
                            corrected_position, uav_idx, seg_idx,
                            intersection_info['obstacle'], bounds
                        )

            if not intersections_found:
                break

        return corrected_position

    def rotate_segment_to_avoid_obstacle(self, position, uav_idx, seg_idx, obstacle, bounds):
        updated_position = position.copy()

        p1 = position[uav_idx, seg_idx]
        p2 = position[uav_idx, seg_idx + 1]

        can_move_p1 = seg_idx != 0
        can_move_p2 = seg_idx + 1 != self.waypoints - 1

        if not can_move_p1 and not can_move_p2:
            return updated_position

        rotation_angles = [15, 30, 45, 60, 90, -15, -30, -45, -60, -90]

        for angle in rotation_angles:
            success = False

            if can_move_p1 and can_move_p2:
                obstacle_centroid = np.array([obstacle.centroid.x, obstacle.centroid.y])
                dist_p1 = np.linalg.norm(p1 - obstacle_centroid)
                dist_p2 = np.linalg.norm(p2 - obstacle_centroid)

                if dist_p1 < dist_p2:
                    new_p1 = self.rotate_point_around_anchor(p1, p2, angle)
                    if self.is_point_in_bounds(new_p1, bounds):
                        test_segment = LineString([new_p1, p2])
                        if not test_segment.intersects(obstacle):
                            updated_position[uav_idx, seg_idx] = new_p1
                            success = True
                else:
                    new_p2 = self.rotate_point_around_anchor(p2, p1, angle)
                    if self.is_point_in_bounds(new_p2, bounds):
                        test_segment = LineString([p1, new_p2])
                        if not test_segment.intersects(obstacle):
                            updated_position[uav_idx, seg_idx + 1] = new_p2
                            success = True

            elif can_move_p1:
                new_p1 = self.rotate_point_around_anchor(p1, p2, angle)
                if self.is_point_in_bounds(new_p1, bounds):
                    test_segment = LineString([new_p1, p2])
                    if not test_segment.intersects(obstacle):
                        updated_position[uav_idx, seg_idx] = new_p1
                        success = True

            elif can_move_p2:
                new_p2 = self.rotate_point_around_anchor(p2, p1, angle)
                if self.is_point_in_bounds(new_p2, bounds):
                    test_segment = LineString([p1, new_p2])
                    if not test_segment.intersects(obstacle):
                        updated_position[uav_idx, seg_idx + 1] = new_p2
                        success = True

            if success:
                break

        return updated_position

    def rotate_point_around_anchor(self, point, anchor, angle_degrees):
        angle_rad = math.radians(angle_degrees)
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)

        translated_point = point - anchor
        rotated_x = translated_point[0] * cos_angle - translated_point[1] * sin_angle
        rotated_y = translated_point[0] * sin_angle + translated_point[1] * cos_angle
        rotated_point = np.array([rotated_x, rotated_y]) + anchor
        return rotated_point

    def is_point_in_bounds(self, point, bounds):
        return (bounds[0][0] <= point[0] <= bounds[1][0] and
                bounds[0][1] <= point[1] <= bounds[1][1])

    def detect_segment_intersection(self, position, uav_idx, seg_idx):
        p1 = position[uav_idx, seg_idx]
        p2 = position[uav_idx, seg_idx + 1]
        segment = LineString([p1, p2])

        for obstacle_poly in self.obstacle_polygons:
            if segment.intersects(obstacle_poly):
                return {
                    'intersecting': True,
                    'obstacle': obstacle_poly,
                    'segment': segment,
                    'p1': p1,
                    'p2': p2
                }
        return {'intersecting': False}

    def calculate_obstacle_gradient(self):
        gradient = np.zeros_like(self.position)

        if not self.obstacle_polygons:
            return gradient

        for uav_i in range(self.uav_count):
            for t in range(self.waypoints - 1):
                p1 = self.position[uav_i, t]
                p2 = self.position[uav_i, t + 1]
                segment = LineString([p1, p2])

                for obstacle_poly in self.obstacle_polygons:
                    if segment.intersects(obstacle_poly):
                        seg_midpoint = np.array([(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2])
                        obstacle_centroid = np.array([obstacle_poly.centroid.x, obstacle_poly.centroid.y])
                        displacement_direction = seg_midpoint - obstacle_centroid
                        direction_norm = np.linalg.norm(displacement_direction)
                        if direction_norm > 0:
                            displacement_direction = displacement_direction / direction_norm
                        else:
                            displacement_direction = np.array([1.0, 0.0])

                        repulsion_magnitude = 1.0
                        repulsion = displacement_direction * repulsion_magnitude

                        gradient[uav_i, t] += repulsion
                        gradient[uav_i, t + 1] += repulsion

        return gradient

    def calculate_inter_uav_gradient(self):
        gradient = np.zeros_like(self.position)
        safety_distance = 1.0

        segments = {}
        for uav_i in range(self.uav_count):
            segments[uav_i] = []
            for t in range(self.waypoints - 1):
                seg = LineString([
                    (self.position[uav_i, t, 0], self.position[uav_i, t, 1]),
                    (self.position[uav_i, t + 1, 0], self.position[uav_i, t + 1, 1])
                ])
                segments[uav_i].append(seg)

        for uav_i in range(self.uav_count):
            for uav_j in range(uav_i + 1, self.uav_count):
                for t in range(self.waypoints - 1):
                    seg_i = segments[uav_i][t]
                    seg_j = segments[uav_j][t]

                    min_dist = seg_i.distance(seg_j)
                    if min_dist < safety_distance:
                        nearest_pts = nearest_points(seg_i, seg_j)
                        dx = nearest_pts[0].x - nearest_pts[1].x
                        dy = nearest_pts[0].y - nearest_pts[1].y
                        direction = np.array([dx, dy])

                        norm = np.linalg.norm(direction)
                        if norm > 0:
                            direction = direction / norm
                            repulsion = direction * (safety_distance - min_dist)

                            gradient[uav_i, t] += repulsion
                            gradient[uav_i, t + 1] += repulsion
                            gradient[uav_j, t] -= repulsion
                            gradient[uav_j, t + 1] -= repulsion

        return gradient

    def update_velocity(self, global_best_position, w=0.6, c1=1.5, c2=1.5, c3=1.0, c4=1.0):
        r1 = np.random.random((self.uav_count, self.waypoints, self.dimensions))
        r2 = np.random.random((self.uav_count, self.waypoints, self.dimensions))
        cognitive = c1 * r1 * (self.best_position - self.position)
        social = c2 * r2 * (global_best_position - self.position)

        inter_uav_grad = self.calculate_inter_uav_gradient()
        obstacle_grad = self.calculate_obstacle_gradient()
        collision_gradient = inter_uav_grad + 0.2 * obstacle_grad
        random_exploration = self.calculate_random_exploration(0.1)

        self.velocity = w * self.velocity + 0.5 * cognitive + 0.5 * social + collision_gradient + random_exploration

        self.velocity[:, 0, :] = 0
        self.velocity[:, -1, :] = 0

    def calculate_random_exploration(self, exploration_factor=0.3):
        gaussian_noise = np.random.normal(0, 0.5, self.position.shape)
        adaptive_strength = self.calculate_adaptive_exploration_strength()
        exploration_velocity = 1 * gaussian_noise * adaptive_strength
        exploration_velocity *= exploration_factor
        exploration_velocity[:, 0, :] = 0
        exploration_velocity[:, -1, :] = 0
        return exploration_velocity

    def calculate_adaptive_exploration_strength(self):
        if self.fitness == 0:
            strength = 0
        else:
            normalized_fitness = min(np.log(1 + self.fitness) / 2, 5.0)
            strength = 0 + 2 * normalized_fitness / 5.0

        return np.full((self.uav_count, self.waypoints, 1), strength)

    def update_position(self, bounds, obstacles):
        new_position = self.position + self.velocity
        corrected_position = self.apply_rotation_based_avoidance(new_position, bounds)
        corrected_position = np.clip(corrected_position, bounds[0], bounds[1])
        self.position = corrected_position


class PathOptimizer:
    """路径优化器 - 负责路径点缩减"""

    def __init__(self, paths, obstacles, min_distance=1.0):
        self.original_paths = [np.array(path) for path in paths]
        self.optimized_paths = [path.copy() for path in self.original_paths]
        # 添加航点时间索引记录
        self.waypoint_time_indices = [list(range(len(path))) for path in paths]
        self.obstacles = obstacles
        self.min_distance = min_distance
        self.obstacle_polygons = [Polygon(obstacle) for obstacle in obstacles] if obstacles else []
        self.uav_count = len(paths)

    def check_obstacle_collision(self, start_point, end_point):
        if not self.obstacle_polygons:
            return False
        line = LineString([(start_point[0], start_point[1]), (end_point[0], end_point[1])])
        for obstacle_poly in self.obstacle_polygons:
            if line.intersects(obstacle_poly):
                return True
        return False

    def check_segment_distance(self, seg1_start, seg1_end, seg2_start, seg2_end):
        line1 = LineString([(seg1_start[0], seg1_start[1]), (seg1_end[0], seg1_end[1])])
        line2 = LineString([(seg2_start[0], seg2_start[1]), (seg2_end[0], seg2_end[1])])
        return line1.distance(line2) >= self.min_distance

    def get_active_segments(self, uav_idx, start_time, end_time):
        """获取在指定时间段内活跃的路径段"""
        segments = []
        path = self.optimized_paths[uav_idx]
        time_indices = self.waypoint_time_indices[uav_idx]

        for i in range(len(path) - 1):
            seg_start_time = time_indices[i]
            seg_end_time = time_indices[i + 1]

            # 检查时间段是否重叠
            if not (seg_end_time <= start_time or seg_start_time >= end_time):
                segments.append((path[i], path[i + 1], seg_start_time, seg_end_time))

        return segments

    def check_collision_with_other_uavs(self, uav_idx, start_point, end_point, start_time, end_time):
        """检查与其他无人机的碰撞，基于时间同步"""
        for other_uav in range(self.uav_count):
            if other_uav == uav_idx:
                continue

            other_segments = self.get_active_segments(other_uav, start_time, end_time)

            for seg_start, seg_end, seg_start_time, seg_end_time in other_segments:
                if not self.check_segment_distance(start_point, end_point, seg_start, seg_end):
                    return False
        return True

    def optimize_path(self, uav_idx):
        """优化单个无人机的路径，保留时间索引信息"""
        path = self.optimized_paths[uav_idx]
        time_indices = self.waypoint_time_indices[uav_idx]

        waypoints_to_keep = [0]  # 保留的航点在当前路径中的索引
        time_indices_to_keep = [time_indices[0]]  # 保留的航点对应的原始时间索引

        i = 0
        while i < len(path) - 1:
            best_skip = 0
            best_end_idx = i + 1

            # 尝试跳过更多的航点
            for skip in range(2, len(path) - i):
                if i + skip >= len(path):
                    break

                start_point = path[i]
                end_point = path[i + skip]
                start_time = time_indices[i]
                end_time = time_indices[i + skip]

                # 检查障碍物碰撞
                if self.check_obstacle_collision(start_point, end_point):
                    break

                # 检查与其他无人机的碰撞（基于时间同步）
                if not self.check_collision_with_other_uavs(uav_idx, start_point, end_point, start_time, end_time):
                    break

                best_skip = skip
                best_end_idx = i + skip

            if best_skip > 1:
                i = best_end_idx
                waypoints_to_keep.append(i)
                time_indices_to_keep.append(time_indices[i])
            else:
                i += 1
                waypoints_to_keep.append(i)
                time_indices_to_keep.append(time_indices[i])

        # 确保包含最后一个航点
        if waypoints_to_keep[-1] != len(path) - 1:
            waypoints_to_keep.append(len(path) - 1)
            time_indices_to_keep.append(time_indices[len(path) - 1])

        # 去重并排序
        combined = list(zip(waypoints_to_keep, time_indices_to_keep))
        combined = sorted(list(set(combined)))
        waypoints_to_keep, time_indices_to_keep = zip(*combined) if combined else ([], [])

        waypoints_to_keep = list(waypoints_to_keep)
        time_indices_to_keep = list(time_indices_to_keep)

        # 更新优化后的路径和时间索引
        self.optimized_paths[uav_idx] = path[waypoints_to_keep]
        self.waypoint_time_indices[uav_idx] = time_indices_to_keep

        return waypoints_to_keep

    def optimize_all_paths(self):
        """优化所有路径并返回详细结果"""
        priorities = list(range(self.uav_count))
        random.seed(1)
        random.shuffle(priorities)

        results = {}
        for uav_idx in priorities:
            kept_waypoints = self.optimize_path(uav_idx)
            results[uav_idx] = {
                'original_waypoints': len(self.original_paths[uav_idx]),
                'optimized_waypoints': len(self.optimized_paths[uav_idx]),
                'kept_waypoint_indices': kept_waypoints,
                'original_time_indices': self.waypoint_time_indices[uav_idx],
                'compression_rate': (len(self.original_paths[uav_idx]) - len(self.optimized_paths[uav_idx])) / len(
                    self.original_paths[uav_idx]) * 100
            }

        return results


class UAVPathPlannerComplete:
    """
    完整的UAV路径规划与优化系统

    结合PSO路径规划和路径点缩减优化
    """

    def __init__(self, obstacles=None, bounds=None, num_particles=30, waypoints=10,
                 max_iterations=100, min_distance=1.0, enable_optimization=True, **kwargs):
        """
        初始化完整路径规划系统

        Args:
            obstacles: 障碍物列表，每个障碍物为顶点坐标列表
            bounds: 飞行区域边界 [min_bounds, max_bounds]
            num_particles: PSO粒子数量
            waypoints: 初始路径点数量
            max_iterations: PSO最大迭代次数
            min_distance: 最小安全距离
            enable_optimization: 是否启用路径点缩减优化
        """
        self.obstacles = obstacles
        self.bounds = bounds
        self.num_particles = num_particles
        self.waypoints = waypoints
        self.max_iterations = max_iterations
        self.min_distance = min_distance
        self.enable_optimization = enable_optimization

        # 其他参数
        for key, value in kwargs.items():
            setattr(self, key, value)

    def plan_and_optimize_paths(self, uav_data):
        """
        完整的路径规划与优化流程

        Args:
            uav_data: 无人机数据列表
                格式: [
                    {'id': 'uav_1', 'start': [x1, y1], 'goal': [x2, y2]},
                    {'id': 'uav_2', 'start': [x3, y3], 'goal': [x4, y4]},
                    ...
                ]

        Returns:
            dict: 完整规划结果
                格式: {
                    'success': bool,
                    'planning_result': {...},  # PSO规划结果
                    'optimization_result': {...},  # 路径优化结果
                    'final_paths': {
                        'uav_1': {
                            'path': [[x1, y1], [x2, y2], ...],
                            'time_indices': [0, 2, 5, ...],  # 对应原始路径的时间索引
                            'waypoint_count': int
                        },
                        ...
                    },
                    'statistics': {...},
                    'computation_time': float
                }
        """

        start_time = time.time()

        # 步骤1: PSO路径规划
        planning_result = self._run_pso_planning(uav_data)

        if not planning_result['success']:
            return {
                'success': False,
                'error': 'PSO路径规划失败',
                'planning_result': planning_result,
                'computation_time': time.time() - start_time
            }

        # 步骤2: 路径点缩减优化
        optimization_result = None
        final_paths = {}

        if self.enable_optimization:
            optimization_result = self._run_path_optimization(planning_result['paths'])

            # 构建最终路径结果
            for uav_id in planning_result['paths'].keys():
                uav_idx = list(planning_result['paths'].keys()).index(uav_id)
                final_paths[uav_id] = {
                    'path': optimization_result['optimized_paths'][uav_idx].tolist(),
                    'time_indices': optimization_result['time_indices'][uav_idx],
                    'waypoint_count': len(optimization_result['optimized_paths'][uav_idx])
                }
        else:
            # 不进行优化，直接使用PSO结果
            for uav_id, path in planning_result['paths'].items():
                final_paths[uav_id] = {
                    'path': path,
                    'time_indices': list(range(len(path))),  # 顺序时间索引
                    'waypoint_count': len(path)
                }

        # 计算统计信息
        statistics = self._calculate_statistics(final_paths, planning_result, optimization_result)

        computation_time = time.time() - start_time

        return {
            'success': True,
            'planning_result': planning_result,
            'optimization_result': optimization_result,
            'final_paths': final_paths,
            'statistics': statistics,
            'computation_time': computation_time
        }

    def _run_pso_planning(self, uav_data):
        """运行PSO路径规划"""
        # 提取无人机信息
        uav_ids = [uav['id'] for uav in uav_data]
        start_positions = [np.array(uav['start']) for uav in uav_data]
        goal_positions = [np.array(uav['goal']) for uav in uav_data]
        uav_count = len(uav_data)

        # 自动计算边界（如果未提供）
        if self.bounds is None:
            all_points = np.vstack(start_positions + goal_positions)
            min_bounds = np.min(all_points, axis=0) 
            max_bounds = np.max(all_points, axis=0) 
            bounds = [min_bounds, max_bounds]
        else:
            bounds = self.bounds

        # 初始化PSO
        particles = []
        for _ in range(self.num_particles):
            particle = Particle(uav_count, self.waypoints, start_positions,
                                goal_positions, bounds, self.obstacles)
            particles.append(particle)

        # 找到全局最优
        global_best_position = None
        global_best_fitness = float('inf')

        for particle in particles:
            if particle.best_fitness < global_best_fitness:
                global_best_fitness = particle.best_fitness
                global_best_position = particle.best_position.copy()

        # 优化迭代
        for iteration in range(self.max_iterations):
            for particle in particles:
                particle.update_velocity(global_best_position)
                particle.update_position(bounds, self.obstacles)
                particle.fitness = particle.calculate_fitness(start_positions, goal_positions)

                if particle.fitness < particle.best_fitness:
                    particle.best_fitness = particle.fitness
                    particle.best_position = particle.position.copy()

            # 更新全局最优
            for particle in particles:
                if particle.best_fitness < global_best_fitness:
                    global_best_fitness = particle.best_fitness
                    global_best_position = particle.best_position.copy()

            # 检查是否找到无碰撞解
            if global_best_fitness < 10000:
                break

        # 构建PSO规划结果
        paths = {}
        for i, uav_id in enumerate(uav_ids):
            path_points = global_best_position[i].tolist()
            paths[uav_id] = path_points

        success = global_best_fitness < 10000

        return {
            'success': success,
            'paths': paths,
            'fitness': global_best_fitness,
            'iterations': iteration + 1,
            'uav_count': uav_count,
            'waypoints': self.waypoints
        }

    def _run_path_optimization(self, pso_paths):
        """运行路径点缩减优化"""
        # 将PSO结果转换为路径列表
        paths = [pso_paths[uav_id] for uav_id in sorted(pso_paths.keys())]

        # 创建路径优化器
        optimizer = PathOptimizer(paths, self.obstacles, self.min_distance)

        # 执行优化
        optimization_details = optimizer.optimize_all_paths()

        return {
            'optimized_paths': optimizer.optimized_paths,
            'time_indices': optimizer.waypoint_time_indices,
            'optimization_details': optimization_details
        }

    def _calculate_statistics(self, final_paths, planning_result, optimization_result):
        """计算统计信息"""
        stats = {
            'uav_count': len(final_paths),
            'per_uav_stats': {}
        }

        total_original = 0
        total_optimized = 0

        for uav_id, path_info in final_paths.items():
            path_array = np.array(path_info['path'])
            segments = path_array[1:] - path_array[:-1]
            segment_lengths = np.linalg.norm(segments, axis=1)
            total_length = np.sum(segment_lengths)

            # 计算压缩率
            if self.enable_optimization and optimization_result:
                uav_idx = list(final_paths.keys()).index(uav_id)
                original_count = optimization_result['optimization_details'][uav_idx]['original_waypoints']
                compression_rate = optimization_result['optimization_details'][uav_idx]['compression_rate']
            else:
                original_count = len(path_info['path'])
                compression_rate = 0.0

            total_original += original_count
            total_optimized += path_info['waypoint_count']

            stats['per_uav_stats'][uav_id] = {
                'path_length': float(total_length),
                'original_waypoints': original_count,
                'final_waypoints': path_info['waypoint_count'],
                'compression_rate': float(compression_rate),
                'avg_segment_length': float(np.mean(segment_lengths)),
                'max_segment_length': float(np.max(segment_lengths)),
                'min_segment_length': float(np.min(segment_lengths))
            }

        stats['overall_compression_rate'] = (
                    (total_original - total_optimized) / total_original * 100) if total_original > 0 else 0.0
        stats['total_original_waypoints'] = total_original
        stats['total_final_waypoints'] = total_optimized

        return stats

    def get_path_by_uav_id(self, result, uav_id):
        """
        根据UAV ID获取其路径信息

        Args:
            result: plan_and_optimize_paths返回的结果
            uav_id: 无人机ID

        Returns:
            dict: 该无人机的路径信息
        """
        if not result['success'] or uav_id not in result['final_paths']:
            return None

        return result['final_paths'][uav_id]


