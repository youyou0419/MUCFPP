#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point, Pose
from gazebo_msgs.msg import ModelStates
from uav_control.msg import UAVPath
from uav_control.srv import PathTracking, PathTrackingResponse  # 新的服务类型
from std_msgs.msg import String
import threading
from collections import deque
from scipy.spatial.distance import cdist
import time

class HighPrecisionFilter:
    """高精度数据滤波器"""
    def __init__(self, window_size=5, filter_type='kalman'):
        self.window_size = window_size
        self.filter_type = filter_type
        
        self.pos_history = deque(maxlen=window_size)
        self.vel_history = deque(maxlen=window_size)
        self.time_history = deque(maxlen=window_size)
        
        self.kalman_initialized = False
        self.state = np.zeros(6, dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 1000.0
        self.Q = np.eye(6, dtype=np.float64) * 0.01
        self.R = np.eye(3, dtype=np.float64) * 0.1
        
    def add_measurement(self, pos, vel, timestamp):
        """添加新的测量数据"""
        pos_array = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        vel_array = np.array([vel.linear.x, vel.linear.y, vel.linear.z], dtype=np.float64)
        
        self.pos_history.append(pos_array)
        self.vel_history.append(vel_array)
        self.time_history.append(float(timestamp))
        
        if self.filter_type == 'kalman':
            return self._kalman_filter(pos_array, timestamp)
        else:
            return self._moving_average_filter()
    
    def _kalman_filter(self, measurement, timestamp):
        """Kalman滤波器实现"""
        if not self.kalman_initialized:
            self.state[:3] = measurement
            self.kalman_initialized = True
            return measurement, np.zeros(3, dtype=np.float64)
        
        if len(self.time_history) >= 2:
            dt = self.time_history[-1] - self.time_history[-2]
            dt = max(dt, 0.001)
        else:
            dt = 0.02
        
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
        
        H = np.zeros((3, 6), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        
        y = measurement - H @ self.state
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(6, dtype=np.float64) - K @ H) @ self.P
        
        return self.state[:3], self.state[3:]
    
    def _moving_average_filter(self):
        """移动平均滤波器"""
        if len(self.pos_history) < 2:
            return self.pos_history[-1], np.zeros(3, dtype=np.float64)
        
        weights = np.linspace(0.5, 1.0, len(self.pos_history), dtype=np.float64)
        weights = weights / np.sum(weights)
        
        filtered_pos = np.zeros(3, dtype=np.float64)
        for i, pos in enumerate(self.pos_history):
            filtered_pos += weights[i] * pos
        
        if len(self.pos_history) >= 2 and len(self.time_history) >= 2:
            dt = self.time_history[-1] - self.time_history[-2]
            if dt > 0.001:
                filtered_vel = (self.pos_history[-1] - self.pos_history[-2]) / dt
            else:
                filtered_vel = np.zeros(3, dtype=np.float64)
        else:
            filtered_vel = np.zeros(3, dtype=np.float64)
        
        return filtered_pos, filtered_vel

class CentralControlledUAVTracker:
    """中央控制的无人机跟踪器"""
    def __init__(self, uav_id, path_points, time_indices, uav_speed=0.5, min_separation=1.0):
        self.uav_id = uav_id
        self.path_points = path_points
        self.time_indices = time_indices
        self.uav_speed = uav_speed
        self.min_separation = min_separation
        
        self.filter = HighPrecisionFilter(window_size=10, filter_type='kalman')
        
        self.states = {
            "INACTIVE": 0,
            "WAITING": 1,
            "FLYING": 2,
            "FINISHED": 3
        }
        self.current_state = self.states["INACTIVE"]
        
        self.current_waypoint_idx = 0
        self.current_segment_idx = None
        self.segment_progress = 0.0
        
        self.raw_pos = None
        self.raw_vel = None
        self.filtered_pos = None
        self.filtered_vel = None
        self.target_pos = None
        self.start_point = None
        
        if path_points:
            self.start_point = np.array([path_points[0].x, path_points[0].y, path_points[0].z], dtype=np.float64)
            self.target_pos = self.start_point.copy()
        
        self.position_tolerance = 0.15
        self.velocity_tolerance = 0.1
        self.max_speed = 1.0
        self.cruise_speed = 1.0
        self.approach_speed = 0.5
        self.epsilon = 1e-6
        
        self.stationary_start_time = None
        self.stationary_duration = 0.1
        
        self.pid_kp = 1.5
        self.pid_ki = 0.1
        self.pid_kd = 0.3
        self.integral_error = np.zeros(3, dtype=np.float64)
        self.prev_error = np.zeros(3, dtype=np.float64)
        self.prev_time = None
    
    def update_measurements(self, pos, vel, timestamp):
        """更新测量数据并进行滤波"""
        self.raw_pos = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        self.raw_vel = np.array([vel.linear.x, vel.linear.y, vel.linear.z], dtype=np.float64)
        
        self.filtered_pos, self.filtered_vel = self.filter.add_measurement(pos, vel, timestamp)
    
    def get_current_time_index(self):
        """获取当前时间索引"""
        if self.current_state == self.states["WAITING"]:
            return self.time_indices[self.current_waypoint_idx]
        elif self.current_state == self.states["FLYING"]:
            return self.time_indices[self.current_segment_idx + 1]
        return None
    
    def get_current_position(self):
        """获取当前位置"""
        if self.filtered_pos is not None:
            return self.filtered_pos
        elif self.current_state == self.states["WAITING"]:
            point = self.path_points[self.current_waypoint_idx]
            return np.array([point.x, point.y, point.z], dtype=np.float64)
        return None
    
    def can_start_next_segment(self):
        """检查是否可以开始下一段"""
        return (self.current_state == self.states["WAITING"] and
                self.current_waypoint_idx < len(self.path_points) - 1)
    
    def is_at_target(self, target=None):
        """使用球形范围的高精度位置检查"""
        if self.filtered_pos is None:
            return False
        
        if target is None:
            target = self.target_pos
        
        if target is None:
            return False
        
        distance = np.linalg.norm(self.filtered_pos - target)
        return distance < self.position_tolerance
    
    def is_stationary(self):
        """高精度静止检查"""
        if not self.is_at_target() or self.filtered_vel is None:
            return False
        
        vel_magnitude = np.linalg.norm(self.filtered_vel)
        return vel_magnitude < self.velocity_tolerance
    
    def update_stationary_status(self):
        """更新静止状态计时器"""
        if self.is_stationary():
            if self.stationary_start_time is None:
                self.stationary_start_time = rospy.Time.now()
            return (rospy.Time.now() - self.stationary_start_time).to_sec() >= self.stationary_duration
        else:
            self.stationary_start_time = None
            return False
    
    def start_flight_to_segment(self, segment_idx):
        """开始飞行到指定段"""
        if segment_idx < len(self.path_points) - 1:
            self.current_state = self.states["FLYING"]
            self.current_segment_idx = segment_idx
            self.segment_progress = 0.0
            
            next_point = self.path_points[segment_idx + 1]
            self.target_pos = np.array([next_point.x, next_point.y, next_point.z], dtype=np.float64)
            
            self.integral_error = np.zeros(3, dtype=np.float64)
            self.prev_error = np.zeros(3, dtype=np.float64)
            self.prev_time = None
            
            return True
        return False
    
    def update_flying_state(self, dt):
        """更新飞行状态"""
        if self.current_state != self.states["FLYING"] or self.current_segment_idx is None:
            return
        
        start_point = self.path_points[self.current_segment_idx]
        end_point = self.path_points[self.current_segment_idx + 1]
        
        start_pos = np.array([start_point.x, start_point.y, start_point.z], dtype=np.float64)
        end_pos = np.array([end_point.x, end_point.y, end_point.z], dtype=np.float64)
        
        segment_distance = np.linalg.norm(end_pos - start_pos)
        segment_time = segment_distance / self.uav_speed if segment_distance > 0 else 0
        
        if segment_time > 0:
            self.segment_progress += dt / segment_time
        
        if self.segment_progress >= 1.0 or self.is_at_target():
            self.segment_progress = 1.0
            self.current_state = self.states["WAITING"]
            self.current_waypoint_idx = self.current_segment_idx + 1
            self.current_segment_idx = None
            
            if self.current_waypoint_idx >= len(self.path_points) - 1:
                self.current_state = self.states["FINISHED"]
                rospy.loginfo(f"{self.uav_id} completed all waypoints")
            else:
                rospy.loginfo(f"{self.uav_id} reached waypoint {self.current_waypoint_idx}")
    
    def calculate_velocity_command_pid(self):
        """使用PID控制器的高精度速度控制"""
        cmd = Twist()
        
        if (self.current_state in [self.states["INACTIVE"], self.states["FINISHED"], 
                                  self.states["WAITING"]] or
            self.filtered_pos is None or self.target_pos is None):
            return cmd
        
        current_time = rospy.Time.now().to_sec()
        
        error = self.target_pos - self.filtered_pos
        distance = np.linalg.norm(error)
        
        if self.prev_time is not None:
            dt = current_time - self.prev_time
            dt = max(dt, 1e-6)
            
            self.integral_error += error * dt
            integral_limit = 5.0
            self.integral_error = np.clip(self.integral_error, -integral_limit, integral_limit)
            
            derivative_error = (error - self.prev_error) / dt
            
            pid_output = (self.pid_kp * error + 
                         self.pid_ki * self.integral_error + 
                         self.pid_kd * derivative_error)
        else:
            pid_output = self.pid_kp * error
        
        if distance > 2.0:
            max_cmd_speed = self.cruise_speed
        elif distance > 0.5:
            max_cmd_speed = self.cruise_speed * (distance / 2.0)
        else:
            max_cmd_speed = self.approach_speed
        
        cmd_magnitude = np.linalg.norm(pid_output)
        if cmd_magnitude > max_cmd_speed:
            pid_output = pid_output * (max_cmd_speed / cmd_magnitude)
        
        cmd.linear.x = float(pid_output[0])
        cmd.linear.y = float(pid_output[1])
        cmd.linear.z = float(pid_output[2])
        
        self.prev_error = error.copy()
        self.prev_time = current_time
        
        return cmd

class CentralUAVController:
    """中央无人机控制器（服务服务器版本）"""
    def __init__(self):
        rospy.init_node('central_uav_controller')
        
        self.uav_trackers = {}
        self.system_phase = "INIT"
        self.min_separation = 1.0
        
        # 当前跟踪任务状态
        self.current_ldr_id = None
        self.tracking_start_time = None
        self.tracking_completed = False
        self.tracking_lock = threading.Lock()
        
        # ROS接口 - 服务服务器代替话题订阅
        self.path_tracking_server = rospy.Service('/path_tracking_service', PathTracking, self.path_tracking_callback)
        self.state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        
        self.cmd_pubs = {}
        self.data_lock = threading.Lock()
        self.controller_ready = False
        
        # 控制循环定时器
        self.control_timer = None
        
        rospy.loginfo("Central UAV Controller (Service Server Version) initialized")
    
    def path_tracking_callback(self, req):
        """处理路径跟踪服务请求"""
        rospy.loginfo(f"收到路径跟踪请求 - LDR {req.ldr_id}")
        
        response = PathTrackingResponse()
        
        try:
            with self.tracking_lock:
                # 重置状态
                self.current_ldr_id = req.ldr_id
                self.tracking_start_time = time.time()
                self.tracking_completed = False
                
                # 停止之前的控制循环
                if self.control_timer is not None:
                    self.control_timer.shutdown()
                
                # 清理之前的跟踪器和发布者
                with self.data_lock:
                    self.uav_trackers.clear()
                    for pub in self.cmd_pubs.values():
                        try:
                            pub.unregister()
                        except:
                            pass
                    self.cmd_pubs.clear()
                
                # 设置新的跟踪器
                self._setup_trackers(req.uav_paths)
                
                if len(self.cmd_pubs) == len(self.uav_trackers):
                    self.controller_ready = True
                    self.system_phase = "COORDINATED_TRACKING"
                    
                    # 启动控制循环
                    self.control_timer = rospy.Timer(rospy.Duration(0.01), self.control_loop)
                    
                    rospy.loginfo(f"开始执行LDR {req.ldr_id} 的路径跟踪")
                    
                    # 等待路径跟踪完成
                    success, execution_time, message = self._wait_for_completion()
                    
                    response.success = success
                    response.execution_time = execution_time
                    response.message = message
                    
                else:
                    response.success = False
                    response.execution_time = 0.0
                    response.message = f"设置跟踪器失败: {len(self.cmd_pubs)} 发布者 vs {len(self.uav_trackers)} 跟踪器"
                    
        except Exception as e:
            rospy.logerr(f"路径跟踪服务处理异常: {e}")
            response.success = False
            response.execution_time = 0.0
            response.message = f"处理异常: {str(e)}"
        
        # 清理资源
        self._cleanup_tracking()
        
        return response
    
    def _setup_trackers(self, uav_paths):
        """设置无人机跟踪器"""
        rospy.loginfo(f"设置 {len(uav_paths)} 个无人机跟踪器")
        
        for uav_path in uav_paths:
            tracker = CentralControlledUAVTracker(
                uav_path.uav_id,
                [Point(p.x, p.y, p.z) for p in uav_path.path_points],
                uav_path.time_indices,
                uav_speed=0.5,
                min_separation=self.min_separation
            )
            tracker.current_state = tracker.states["WAITING"]
            self.uav_trackers[uav_path.uav_id] = tracker
            
            try:
                self.cmd_pubs[uav_path.uav_id] = rospy.Publisher(
                    f"/{uav_path.uav_id}/cmd_vel", Twist, queue_size=1
                )
                rospy.loginfo(f"为 {uav_path.uav_id} 创建发布者")
            except Exception as e:
                rospy.logerr(f"为 {uav_path.uav_id} 创建发布者失败: {e}")
    
    def _wait_for_completion(self, timeout=60.0):
        """等待路径跟踪完成"""
        rospy.loginfo("等待路径跟踪完成...")
        
        start_time = time.time()
        check_interval = 0.5  # 检查间隔
        
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # 检查超时
            if current_time - start_time > timeout:
                rospy.logwarn("路径跟踪超时")
                return False, current_time - start_time, "执行超时"
            
            # 检查是否完成
            with self.data_lock:
                if self._check_all_finished():
                    execution_time = current_time - start_time
                    rospy.loginfo(f"路径跟踪完成，耗时 {execution_time:.2f} 秒")
                    return True, execution_time, "路径跟踪成功完成"
            
            time.sleep(check_interval)
        
        return False, time.time() - start_time, "节点关闭"
    
    def _check_all_finished(self):
        """检查所有无人机是否完成"""
        if not self.uav_trackers:
            return False
        
        for tracker in self.uav_trackers.values():
            if tracker.current_state != tracker.states["FINISHED"]:
                return False
        
        return True
    
    def _cleanup_tracking(self):
        """清理跟踪资源"""
        rospy.loginfo("清理跟踪资源")
        
        with self.tracking_lock:
            # 停止控制循环
            if self.control_timer is not None:
                self.control_timer.shutdown()
                self.control_timer = None
            
            # 停止所有无人机
            with self.data_lock:
                for uav_id in self.cmd_pubs:
                    try:
                        stop_cmd = Twist()
                        self.cmd_pubs[uav_id].publish(stop_cmd)
                    except:
                        pass
            
            self.controller_ready = False
            self.system_phase = "INIT"
            self.current_ldr_id = None
            self.tracking_completed = False
    
    def state_callback(self, msg):
        """更新无人机状态"""
        if not self.controller_ready:
            return
            
        current_time = rospy.Time.now().to_sec()
        
        with self.data_lock:
            for uav_id, tracker in self.uav_trackers.items():
                try:
                    idx = msg.name.index(uav_id)
                    pos = msg.pose[idx].position
                    vel = msg.twist[idx]
                    
                    tracker.update_measurements(pos, vel, current_time)
                    
                except ValueError:
                    rospy.logwarn_throttle(1.0, f"{uav_id} not found in model states")
    
    def get_separation_distances(self):
        """计算所有无人机对之间的分离距离"""
        positions = []
        uav_ids = []
        
        for uav_id, tracker in self.uav_trackers.items():
            pos = tracker.get_current_position()
            if pos is not None:
                positions.append(pos)
                uav_ids.append(uav_id)
        
        if len(positions) < 2:
            return {}
        
        distances = {}
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = np.linalg.norm(positions[i] - positions[j])
                distances[(uav_ids[i], uav_ids[j])] = dist
        
        return distances
    
    def can_uav_start_flight(self, uav_id):
        """判断指定无人机是否可以开始飞行"""
        target_tracker = self.uav_trackers[uav_id]
        
        if not target_tracker.can_start_next_segment():
            return False
        
        target_time_idx = target_tracker.get_current_time_index()
        
        all_waiting = True
        waiting_time_indices = []
        flying_end_time_indices = []
        
        for tracker_id, tracker in self.uav_trackers.items():
            if tracker.current_state == tracker.states["FLYING"]:
                all_waiting = False
                flying_end_time_indices.append(tracker.time_indices[tracker.current_segment_idx + 1])
            elif tracker.current_state == tracker.states["WAITING"]:
                waiting_time_indices.append(tracker.get_current_time_index())
        
        if all_waiting and len(set(waiting_time_indices)) == 1:
            return True
        
        for end_time_idx in flying_end_time_indices:
            if end_time_idx <= target_time_idx:
                return False
        
        for other_time_idx in waiting_time_indices:
            if other_time_idx != target_time_idx and other_time_idx <= target_time_idx:
                return False
        
        return True
    
    def control_loop(self, event):
        """中央协调控制循环"""
        if not self.controller_ready:
            return
            
        try:
            dt = 0.01
            
            with self.data_lock:
                if self.system_phase == "COORDINATED_TRACKING":
                    self._handle_coordinated_tracking(dt)
                
                self._publish_commands()
            
            # 定期日志输出（降低频率）
            if hasattr(self, '_last_log_time'):
                if time.time() - self._last_log_time > 2.0:
                    self._log_system_status()
                    self._last_log_time = time.time()
            else:
                self._last_log_time = time.time()
            
        except Exception as e:
            rospy.logerr(f"Error in control loop: {e}")
    
    def _handle_coordinated_tracking(self, dt):
        """处理协调路径跟踪"""
        for uav_id, tracker in self.uav_trackers.items():
            if tracker.current_state == tracker.states["WAITING"] and self.can_uav_start_flight(uav_id):
                if tracker.start_flight_to_segment(tracker.current_waypoint_idx):
                    rospy.loginfo(f"{uav_id} started flight from WP{tracker.current_waypoint_idx} to WP{tracker.current_waypoint_idx + 1}")
        
        for tracker in self.uav_trackers.values():
            if tracker.current_state == tracker.states["FLYING"]:
                tracker.update_flying_state(dt)
        
        # 检查是否全部完成
        if self._check_all_finished():
            self.system_phase = "COMPLETED"
            rospy.loginfo("All UAVs completed their coordinated paths")
    
    def _publish_commands(self):
        """发布高精度控制命令"""
        for uav_id, tracker in self.uav_trackers.items():
            if uav_id not in self.cmd_pubs:
                rospy.logwarn_throttle(5.0, f"Publisher for {uav_id} not found, skipping command")
                continue
                
            try:
                cmd = tracker.calculate_velocity_command_pid()
                self.cmd_pubs[uav_id].publish(cmd)
            except Exception as e:
                rospy.logerr(f"Error publishing command for {uav_id}: {e}")
    
    def _log_system_status(self):
        """系统状态日志"""
        if not self.controller_ready:
            return
            
        try:
            with self.data_lock:
                distances = self.get_separation_distances()
                min_dist = min(distances.values()) if distances else float('inf')
                violations = sum(1 for dist in distances.values() if dist < self.min_separation)
                
                status_msgs = [f"=== Central UAV Controller Status (LDR {self.current_ldr_id}) ==="]
                status_msgs.append(f"System Phase: {self.system_phase}")
                status_msgs.append(f"Minimum Separation: {min_dist:.3f}m (Required: {self.min_separation}m)")
                status_msgs.append(f"Safety Violations: {violations}/{len(distances)} pairs")
                
                if self.tracking_start_time:
                    elapsed = time.time() - self.tracking_start_time
                    status_msgs.append(f"Tracking Time: {elapsed:.1f}s")
                
                finished_count = sum(1 for t in self.uav_trackers.values() 
                                   if t.current_state == t.states["FINISHED"])
                total_count = len(self.uav_trackers)
                status_msgs.append(f"Progress: {finished_count}/{total_count} UAVs finished")
                
                status_msgs.append("Individual UAV Status:")
                
                for uav_id, tracker in self.uav_trackers.items():
                    state_name = next(k for k,v in tracker.states.items() if v == tracker.current_state)
                    
                    if tracker.filtered_pos is not None:
                        pos = tracker.filtered_pos
                        vel_mag = np.linalg.norm(tracker.filtered_vel) if tracker.filtered_vel is not None else 0.0
                        
                        if tracker.current_state == tracker.states["WAITING"]:
                            time_idx = tracker.get_current_time_index()
                            can_fly = self.can_uav_start_flight(uav_id)
                            status = (f"  {uav_id}: {state_name} at WP{tracker.current_waypoint_idx} "
                                     f"(t{time_idx}) [{'Ready' if can_fly else 'Blocked'}] "
                                     f"Pos:[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}] Vel:{vel_mag:.3f}m/s")
                        elif tracker.current_state == tracker.states["FLYING"]:
                            progress_pct = tracker.segment_progress * 100
                            status = (f"  {uav_id}: {state_name} segment {tracker.current_segment_idx} "
                                     f"[{progress_pct:.1f}%] "
                                     f"Pos:[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}] Vel:{vel_mag:.3f}m/s")
                        else:
                            status = (f"  {uav_id}: {state_name} "
                                     f"Pos:[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}] Vel:{vel_mag:.3f}m/s")
                        
                        status_msgs.append(status)
                    else:
                        status_msgs.append(f"  {uav_id}: {state_name} [No position data]")
                
                rospy.loginfo("\n".join(status_msgs))
        except Exception as e:
            rospy.logerr(f"Error in status logging: {e}")

if __name__ == '__main__':
    try:
        controller = CentralUAVController()
        rospy.loginfo("Central UAV Controller (Service Server) ready for path tracking requests")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Central UAV Controller shutdown")