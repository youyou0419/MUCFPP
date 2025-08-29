#!/usr/bin/env python
import rospy
from quad_star.srv import PathPlan, PathPlanResponse
from geometry_msgs.msg import Point
import numpy as np
import heapq
import math

class AStarPlanner:
    def __init__(self):
        rospy.init_node('a_star_planner')
        
        # Parameters
        self.map_size = rospy.get_param('~map_size', [-10.0, 10.0, -10.0, 10.0])  # x_min, x_max, y_min, y_max
        self.resolution = rospy.get_param('~resolution', 0.1)  # meters/cell
        self.drone_size = rospy.get_param('~drone_size', 0.8)  # meters
        self.obstacles = rospy.get_param('~obstacles', [[0, 5.5, 0.2, 9], [0, -5.5, 0.2, 9]])
        
        # Build the grid map once at startup
        self.grid_map, self.drone_radius_grid = self.build_grid_map(
            self.obstacles, self.map_size, self.resolution, self.drone_size)
        
        # Service for path planning
        self.plan_service = rospy.Service('/path_planning_service', PathPlan, self.handle_plan_request)
        
        rospy.loginfo("A* Path Planning Service initialized")

    def build_grid_map(self, obstacles, map_size, resolution, drone_size=0.8):
        """
        Build a 2D grid map considering drone size
        """
        x_min, x_max, y_min, y_max = map_size
        grid_width = int((x_max - x_min) / resolution)
        grid_height = int((y_max - y_min) / resolution)
        grid_map = np.zeros((grid_width, grid_height), dtype=np.uint8)

        # Calculate drone radius in grid cells (rounded up)
        drone_radius_grid = int(np.ceil(drone_size / (2 * resolution)))

        for obs in obstacles:
            x, y, w, d = obs
            # Expand obstacle by drone size
            w_expanded = w + drone_size/2
            d_expanded = d + drone_size/2
            # Calculate expanded obstacle bounds in grid
            epsilon = 1e-10
            x_start = int((x - x_min - w_expanded / 2) / resolution + epsilon)
            x_end = math.ceil((x - x_min + w_expanded / 2) / resolution)
            y_start = int((y - y_min - d_expanded / 2) / resolution + epsilon)
            y_end = math.ceil((y - y_min + d_expanded / 2) / resolution)
            # Ensure within bounds
            x_start, x_end = max(0, x_start), min(grid_width, x_end)
            y_start, y_end = max(0, y_start), min(grid_height, y_end)
            # Mark obstacle
            grid_map[x_start:x_end, y_start:y_end] = 1

        return grid_map, drone_radius_grid

    def is_valid_position(self, grid_map, pos, drone_radius_grid):
        """Check if position is valid considering drone size"""
        x, y = pos
        # Check all grid cells the drone would occupy
        for dx in range(-drone_radius_grid, drone_radius_grid + 1):
            for dy in range(-drone_radius_grid, drone_radius_grid + 1):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < grid_map.shape[0] and 0 <= ny < grid_map.shape[1]):
                    return False
                if grid_map[nx, ny] == 1:
                    return False
        return True

    def find_nearest_free(self, grid_map, pos, drone_radius_grid, max_radius=10):
        """Find nearest free position considering drone size"""
        x, y = pos
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    new_pos = (x + dx, y + dy)
                    if self.is_valid_position(grid_map, new_pos, drone_radius_grid):
                        return new_pos
        return None

    def heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def a_star(self, grid_map, start, goal, drone_radius_grid):
        """A* algorithm considering drone size"""
        # Check start and goal validity
        if not self.is_valid_position(grid_map, start, drone_radius_grid):
            rospy.logwarn(f"Start position {start} is invalid")
            return None
        if not self.is_valid_position(grid_map, goal, drone_radius_grid):
            rospy.logwarn(f"Goal position {goal} is invalid")
            return None

        # 8-direction movement
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if self.is_valid_position(grid_map, neighbor, drone_radius_grid):
                    move_cost = 1.0 if (dx == 0 or dy == 0) else np.sqrt(2)
                    tentative_g = g_score[current] + move_cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        return None

    def handle_plan_request(self, req):
        """Service handler for path planning requests"""
        response = PathPlanResponse()
        
        # Convert start and goal positions to grid coordinates
        epsilon = 1e-10
        start_phy = (req.start_x, req.start_y)
        goal_phy = (req.goal_x, req.goal_y)
        
        start_grid = (int((start_phy[0] - self.map_size[0]) / self.resolution + epsilon),
                     int((start_phy[1] - self.map_size[2]) / self.resolution + epsilon))
        goal_grid = (int((goal_phy[0] - self.map_size[0]) / self.resolution + epsilon),
                    int((goal_phy[1] - self.map_size[2]) / self.resolution + epsilon))
        
        # Adjust start/goal if needed
        if not self.is_valid_position(self.grid_map, start_grid, self.drone_radius_grid):
            new_start = self.find_nearest_free(self.grid_map, start_grid, self.drone_radius_grid)
            if new_start:
                start_grid = new_start
                rospy.logwarn(f"Adjusted start to grid position {start_grid}")
            else:
                response.success = False
                response.message = "Cannot find valid start position!"
                rospy.logerr(response.message)
                return response
        
        if not self.is_valid_position(self.grid_map, goal_grid, self.drone_radius_grid):
            new_goal = self.find_nearest_free(self.grid_map, goal_grid, self.drone_radius_grid)
            if new_goal:
                goal_grid = new_goal
                rospy.logwarn(f"Adjusted goal to grid position {goal_grid}")
            else:
                response.success = False
                response.message = "Cannot find valid goal position!"
                rospy.logerr(response.message)
                return response
        
        # Run A* algorithm
        path_grid = self.a_star(self.grid_map, start_grid, goal_grid, self.drone_radius_grid)
        
        if path_grid:
            # Convert path to physical coordinates
            path_points = []
            for p in path_grid:
                point = Point()
                point.x = p[0] * self.resolution + self.map_size[0]
                point.y = p[1] * self.resolution + self.map_size[2]
                point.z = 1.5  # Since this is 2D planning, z is set to 0
                path_points.append(point)
            # 添加降落路径点（z从2降到0）
            landing_point = Point()
            landing_point.x = path_points[-1].x  # 保持x,y不变
            landing_point.y = path_points[-1].y
            landing_point.z = 0.3  # 降落高度
            path_points.append(landing_point)
            
            response.success = True
            response.message = f"Path found with {len(path_points)} waypoints"
            response.path = path_points  # Assuming SimplePathPlanResponse has a 'path' field
            rospy.loginfo(response.message)
        else:
            response.success = False
            response.message = "No valid path found!"
            rospy.logerr(response.message)
        
        return response

if __name__ == "__main__":
    planner = AStarPlanner()
    rospy.spin()