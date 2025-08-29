from __future__ import annotations
import rospy 
from quad_star.srv import PathPlan, PathPlanResponse
from geometry_msgs.msg import Point
import numpy as np
from typing import List, Tuple, Union
import math

class Node:
    def __init__(self, grid_pos: tuple[int, int], g: float, h: float, parent: Node = None):
        self.grid_pos = grid_pos
        self.g = g
        self.h = h
        self.parent = parent
        self.f = self.g + self.h
class JPSPlanner:
    def __init__(self):
        rospy.init_node("JPS_planner")
        self.env = self.create_environment()
        self.occupied_grid, self.grid_center_pixels, self.grid_shape = self.generate_occupancy_grid_with_inflation(
            self.env['width'], self.env['height'], self.env['resolution'], self.env['obstacles'],0.5
        )
        self.plan_service = rospy.Service('/path_planning_service', PathPlan, self.handle_plan_request)
        rospy.loginfo("JPS Path Planning Service initialized")


    def create_environment(self):
        return {
            'width': 20.0,  # 米
            'height': 20.0,  # 米
            'resolution':0.1,  # 米/栅格
            'obstacles': [
                {'center': (0, 5.5), 'size': (0.2, 9)},
                {'center': (0, -5.5), 'size': (0.2, 9)},
                {'center': (0, 10.1), 'size': (20, 0.2)},
                {'center': (0, -10.1), 'size': (20, 0.2)},
                {'center': (10.1, 0), 'size': (0.2, 20.4)},
                {'center': (-10.1, 0), 'size': (0.2, 20.4)},
            ]
        }
    
    def generate_occupancy_grid_with_inflation(self,width, height, resolution, obstacles, inflation_radius=0.4):
        """
        生成物理中心为(0,0)米的占据栅格地图，支持区分原始障碍物和膨胀区域

        参数:
            width (float): 物理总宽度(米)
            height (float): 物理总高度(米)
            resolution (float): 每个栅格代表的物理尺寸(米/栅格)
            obstacles (list): 障碍物列表，每个障碍物表示为字典:
                            {'center': (x, y), 'size': (length, width)}
                            center: 障碍物中心物理坐标(米)，(0,0)表示地图中心
                            size: 障碍物长和宽(米)
            inflation_radius (float): 障碍物膨胀半径(米)，用于安全距离

        返回:
            tuple: (grid, grid_center_pixels, grid_shape)
                grid: 二维占据栅格，0=自由空间，1=原始障碍物，2=膨胀区域
                grid_center_pixels: 网格中心像素坐标
                grid_shape: 网格形状
        """
        # 计算栅格尺寸
        grid_width = int(np.ceil(width / resolution))
        grid_height = int(np.ceil(height / resolution))
        grid_shape = (grid_height, grid_width)

        # 初始化栅格（使用uint8可以表示0-255的值）
        grid = np.zeros(grid_shape, dtype=np.uint8)

        # 计算网格中心像素坐标
        grid_center_pixels = (self.get_grid_center(grid_shape))

        # 计算膨胀半径对应的栅格数
        inflation_cells = int(np.ceil(inflation_radius / resolution)) if inflation_radius > 0 else 0

        # 处理障碍物
        for obstacle in obstacles:
            # 转换物理坐标到网格索引
            center_col, center_row = self.world_to_grid(
                obstacle['center'],
                resolution,
                grid_center_pixels
            )

            # 计算障碍物边界
            half_length = int(np.ceil(obstacle['size'][0] / (2 * resolution)))
            half_width = int(np.ceil(obstacle['size'][1] / (2 * resolution)))

            # 标记原始障碍物区域为1
            orig_row_start = max(0, center_row - half_width)
            orig_row_end = min(grid_height, center_row + half_width)
            orig_col_start = max(0, center_col - half_length)
            orig_col_end = min(grid_width, center_col + half_length)
            grid[orig_row_start:orig_row_end, orig_col_start:orig_col_end] = 1

            # 标记膨胀区域为2（但不覆盖原始障碍物）
            if inflation_cells > 0:
                infl_row_start = max(0, center_row - half_width - inflation_cells)
                infl_row_end = min(grid_height, center_row + half_width + inflation_cells)
                infl_col_start = max(0, center_col - half_length - inflation_cells)
                infl_col_end = min(grid_width, center_col + half_length + inflation_cells)

                # 只标记非原始障碍物的区域为2
                for r in range(infl_row_start, infl_row_end):
                    for c in range(infl_col_start, infl_col_end):
                        if grid[r, c] == 0:  # 只标记自由空间
                            grid[r, c] = 2

        return grid, grid_center_pixels, grid_shape
    
    def get_grid_center(self, grid_shape: Tuple[int, int]) -> Tuple[float, float]:
        return (grid_shape[1] - 1) / 2, (grid_shape[0] - 1) / 2  # (col_center, row_center)
    
    def world_to_grid(self,
                world_coord: Tuple[float, float],
                resolution: float,
                grid_center_pixels: Tuple[float, float]
        ) -> Tuple[int, int]:
            x, y = world_coord
            col = int(np.round(grid_center_pixels[0] + x / resolution))
            row = int(np.round(grid_center_pixels[1] + y / resolution) ) # Y轴翻转
            return (col, row)

    def grid_to_world(self,
        grid_coord: Tuple[int, int],
        resolution: float,
        grid_center_pixels: Tuple[float, float]
    ) -> Tuple[float, float]:
            col, row = grid_coord
            x = (col - grid_center_pixels[0]) * resolution
            y = (row - grid_center_pixels[1]) * resolution# Y轴翻转
            return (x, y)
    
    def setup_pathfinding(self,occupied_grid, grid_center_pixels,start_world,goal_world):
        jps = JPS(occupied_grid.shape[0],occupied_grid.shape[1])
        start_grid = self.world_to_grid(start_world, self.env['resolution'], grid_center_pixels)
        goal_grid = self.world_to_grid(goal_world,self.env['resolution'], grid_center_pixels)

        return {
            'jps': jps,
            'start_point': (start_grid[0], start_grid[1]),
            'goal_point': (goal_grid[0], goal_grid[1]),
            'start_grid': start_grid,
            'goal_grid': goal_grid
        }
    def handle_plan_request(self, req):
        response = PathPlanResponse()
        start_phy = (req.start_x, req.start_y)
        goal_phy = (req.goal_x, req.goal_y)
        # 打印请求信息
        rospy.loginfo(f"\n[Path Planning] Received request:\n"
                 f"Start: ({req.start_x:.2f}, {req.start_y:.2f})\n"
                 f"Goal: ({req.goal_x:.2f}, {req.goal_y:.2f})")
        self.pathfinding = self.setup_pathfinding(self.occupied_grid, self.grid_center_pixels,start_phy,goal_phy)

        searched_path = self.pathfinding['jps'].run(self.pathfinding['start_point'],self.pathfinding['goal_point'],np.array(self.occupied_grid).T)
        if searched_path:
            line_points = [
            np.array(self.grid_to_world((x, y), self.env['resolution'], self.grid_center_pixels))
            for x, y in zip(searched_path[0], searched_path[1])  ]
            # 打印世界坐标
            rospy.loginfo("[World Coordinates] Converted path points:")
            for i, point in enumerate(line_points):
                rospy.loginfo(f"  Point {i}: world=({point[0]:.2f}, {point[1]:.2f})")
            from geometry_msgs.msg import Point

            # 转换为 geometry_msgs/Point 列表
            path_points = []
            for point in line_points:
                p = Point()
                p.x = float(point[0])  # 确保转为Python float类型
                p.y = float(point[1])
                p.z = 1.5  # 固定高度值（根据需求调整）
                path_points.append(p)

            # 添加降落点（可选）
            landing_point = Point()
            landing_point.x = path_points[-1].x
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



class JPS:
    def __init__(self, width: int, height: int):
        self.start_grid_pos = (0, 0)
        self.goal_grid_pos = (0, 0)
        self.width = width
        self.height = height

        self.open = {}  # pos:node
        self.close = {}
        self.map=None
        self.motion_directions = [[1, 0], [0, 1], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]

    def run(self, start_grid_pos: tuple[int, int], goal_grid_pos: tuple[int, int],map):
        self.map=map
        self.start_grid_pos = start_grid_pos
        self.goal_grid_pos = goal_grid_pos
        start_node = Node(start_grid_pos, 0, self.getH(self.start_grid_pos, self.goal_grid_pos))
        self.open[start_node.grid_pos] = start_node

        while True:

            # 寻路失败，结束循环
            if not self.open:
                print("未找到路径")
                break

            current_node = min(self.open.values(), key=lambda x: x.f)  # f值最小的节点

            # 找到路径， 返回结果
            if current_node.grid_pos == self.goal_grid_pos:
                path = self.findPath(current_node)
                print("找到路径！")
                return path

            # 扩展节点
            self.extendNode(current_node)

            # 更新节点
            self.close[current_node.grid_pos] = current_node
            del self.open[current_node.grid_pos]

    def extendNode(self, current_node: Node):
        """
        根据当前节点，扩展节点（只有跳点才可以扩展）
        input
        ----------
        current_node: 当前节点对象
        """
        neighbours = self.getPruneNeighbours(current_node)
        for n in neighbours:
            jp = self.getJumpNode(n, current_node.grid_pos)  # 跳点
            if jp:
                if jp in self.close:
                    continue

                new_node = Node(jp, current_node.g + self.getG(jp, current_node.grid_pos),
                                self.getH(jp, self.goal_grid_pos), current_node)
                if jp in self.open:
                    if new_node.f < self.open[jp].f:
                        self.open[jp].parent = current_node
                        self.open[jp].f = new_node.f
                else:
                    self.open[jp] = new_node

    def getJumpNode(self, now: tuple[int, int], pre: tuple[int, int]) -> Union[tuple[int, int], None]:
        """
        计算跳点
        input
        ----------
        now: 当前节点坐标
        pre: 上一节点坐标
        output
        ----------
        若有跳点，返回跳点坐标；若无跳点，则返回None
        """
        x_direction = int((now[0] - pre[0]) / abs(now[0] - pre[0])) if now[0] - pre[0] != 0 else 0
        y_direction = int((now[1] - pre[1]) / abs(now[1] - pre[1])) if now[1] - pre[1] != 0 else 0

        if now == self.goal_grid_pos:  # 如果当前节点是终点，则为跳点（条件1）
            return now

        if self.hasForceNeighbours(now, pre):  # 如果当前节点包含强迫邻居，则为跳点（条件2）
            return now

        if abs(x_direction) + abs(y_direction) == 2:  # 若为斜线移动，则判断水平和垂直方向是否有满足上述条件1和2的点，若有，则为跳点（条件3）

            if (self.getJumpNode((now[0] + x_direction, now[1]), now) or
                    self.getJumpNode((now[0], now[1] + y_direction), now)):
                return now

        if self.isPass(now[0] + x_direction, now[1] + y_direction):  # 若当前节点未找到跳点，朝当前方向前进一步，继续寻找跳点，直至不可达
            jp = self.getJumpNode((now[0] + x_direction, now[1] + y_direction), now)
            if jp:
                return jp

        return None

    def hasForceNeighbours(self, now: tuple[int, int], pre: tuple[int, int]) -> bool:
        """
        根据当前节点坐标和上一节点坐标判断当前节点是否拥有强迫邻居
        input
        ----------
        now: 当前坐标
        pre: 上一坐标
        output
        ----------
        若拥有强迫邻居，则返回True；否则返回False
        """
        x_direction = now[0] - pre[0]
        y_direction = now[1] - pre[1]

        # 若为直线移动
        if abs(x_direction) + abs(y_direction) == 1:

            if abs(x_direction) == 1:  # 水平移动
                if (self.isPass(now[0] + x_direction, now[1] + 1) and not self.isPass(now[0], now[1] + 1)) or \
                        (self.isPass(now[0] + x_direction, now[1] - 1) and not self.isPass(now[0], now[1] - 1)):
                    return True
                else:
                    return False

            elif abs(y_direction) == 1:  # 垂直移动

                if (self.isPass(now[0] + 1, now[1] + y_direction) and not self.isPass(now[0] + 1, now[1])) or \
                        (self.isPass(now[0] - 1, now[1] + y_direction) and not self.isPass(now[0] - 1, now[1])):
                    return True
                else:
                    return False

            else:
                raise Exception("错误，直线移动中只能水平或垂直移动！")

        # 若为斜线移动
        elif abs(x_direction) + abs(y_direction) == 2:
            if (self.isPass(now[0] + x_direction, now[1] - y_direction) and not self.isPass(now[0],
                                                                                            now[1] - y_direction)) or \
                    (self.isPass(now[0] - x_direction, now[1] + y_direction) and not self.isPass(now[0] - x_direction,
                                                                                                 now[1])):
                return True
            else:
                return False

        else:
            raise Exception("错误，只能直线移动或斜线移动！")

    def getH(self, current: tuple[int, int], goal: tuple[int, int], func: str = "Euclidean") -> float:
        """
        根据当前坐标和终点坐标计算启发值，包含：
            曼哈顿距离（Manhattan Distance）
            欧几里得距离（Euclidean Distance）  默认
            切比雪夫距离（Chebyshev Distance）
        input
        ----------
        current: 当前节点坐标
        goal: 目标节点坐标
        func: 启发函数，默认为欧几里得距离
        """
        current_x = current[0]
        current_y = current[1]
        goal_x = goal[0]
        goal_y = goal[1]

        if func == "Manhattan":  # 适用于格点地图上，移动限制为上下左右四个方向的情况
            h = abs(current_x - goal_x) + abs(current_y - goal_y)

        elif func == "Euclidean":  # 适用于可以自由移动至任何方向的情况，例如在平面上自由移动
            h = math.hypot(current_x - goal_x, current_y - goal_y)

        elif func == "Chebyshev":  # 适用于八方向移动的格点地图（上下左右及对角线）
            h = max(abs(current_x - goal_x), abs(current_y - goal_y))

        else:
            raise Exception("错误，不支持该启发函数。目前支持：Manhattan、Euclidean（默认）、Chebyshev。")

        return h

    def getG(self, pos1: tuple[int, int], pos2: tuple[int, int]) -> float:
        """
        根据两坐标计算代价值（直走或斜走，直接计算欧几里得距离就可）
        input
        ----------
        pos1: 坐标1
        pos2: 坐标2
        output
        ----------
        返回代价值
        """
        return math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])

    def getPruneNeighbours(self, current_node: Node) -> list[tuple[int, int]]:
        """
        获得裁剪之后的邻居节点坐标列表（包含自然邻居和强迫邻居，其相关概念参考：https://fallingxun.github.io/post/algorithm/algorithm_jps/）
        input
        ----------
        current_node: 当前节点对象
        output
        ----------
        prune_neighbours: 裁剪邻居坐标列表
        """
        prune_neighbours = []

        if current_node.parent:
            motion_x = int((current_node.grid_pos[0] - current_node.parent.grid_pos[0]) / abs(
                current_node.grid_pos[0] - current_node.parent.grid_pos[0])) if current_node.grid_pos[0] - \
                                                                                current_node.parent.grid_pos[
                                                                                    0] != 0 else 0  # 方向不分大小，所以要除以长度
            motion_y = int((current_node.grid_pos[1] - current_node.parent.grid_pos[1]) / abs(
                current_node.grid_pos[1] - current_node.parent.grid_pos[1])) if current_node.grid_pos[1] - \
                                                                                current_node.parent.grid_pos[
                                                                                    1] != 0 else 0  # 方向不分大小，所以要除以长度

            if abs(motion_x) + abs(motion_y) == 1:  # 直线

                # 自然邻居
                if self.isPass(current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] + motion_y):
                    prune_neighbours.append((current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] + motion_y))

                # 强迫邻居
                if abs(motion_x) == 0:  # 垂直走
                    if not self.isPass(current_node.grid_pos[0] + 1, current_node.grid_pos[1]) and self.isPass(
                            current_node.grid_pos[0] + 1, current_node.grid_pos[1] + motion_y):
                        prune_neighbours.append((current_node.grid_pos[0] + 1, current_node.grid_pos[1] + motion_y))
                    if not self.isPass(current_node.grid_pos[0] - 1, current_node.grid_pos[1]) and self.isPass(
                            current_node.grid_pos[0] - 1, current_node.grid_pos[1] + motion_y):
                        prune_neighbours.append((current_node.grid_pos[0] - 1, current_node.grid_pos[1] + motion_y))
                else:  # 水平走
                    if not self.isPass(current_node.grid_pos[0], current_node.grid_pos[1] + 1) and self.isPass(
                            current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] + 1):
                        prune_neighbours.append((current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] + 1))
                    if not self.isPass(current_node.grid_pos[0], current_node.grid_pos[1] - 1) and (
                    current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] - 1):
                        prune_neighbours.append((current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] - 1))

            elif abs(motion_x) + abs(motion_y) == 2:  # 对角线

                # 自然邻居
                if self.isPass(current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] + motion_y):
                    prune_neighbours.append((current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] + motion_y))
                if self.isPass(current_node.grid_pos[0] + motion_x, current_node.grid_pos[1]):
                    prune_neighbours.append((current_node.grid_pos[0] + motion_x, current_node.grid_pos[1]))
                if self.isPass(current_node.grid_pos[0], current_node.grid_pos[1] + motion_y):
                    prune_neighbours.append((current_node.grid_pos[0], current_node.grid_pos[1] + motion_y))

                # 强迫邻居
                if not self.isPass(current_node.grid_pos[0] - motion_x, current_node.grid_pos[1]) and self.isPass(
                        current_node.grid_pos[0] - motion_x, current_node.grid_pos[1] + motion_y):
                    prune_neighbours.append((current_node.grid_pos[0] - motion_x, current_node.grid_pos[1] + motion_y))
                if not self.isPass(current_node.grid_pos[0], current_node.grid_pos[1] - motion_y) and self.isPass(
                        current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] - motion_y):
                    prune_neighbours.append((current_node.grid_pos[0] + motion_x, current_node.grid_pos[1] - motion_y))

            else:
                raise Exception("错误，只能对角线和直线行走！")

        else:
            for dir in self.motion_directions:
                if self.isPass(current_node.grid_pos[0] + dir[0], current_node.grid_pos[1] + dir[1]):
                    prune_neighbours.append((current_node.grid_pos[0] + dir[0], current_node.grid_pos[1] + dir[1]))

        return prune_neighbours

    def isPass(self, grid_x: int, grid_y: int) -> bool:
        """
        判断该栅格坐标是否可以通过
        input
        ----------
        grid_x: 栅格x坐标
        grid_y: 栅格y坐标
        output
        ----------
        若可以通过，为True，反之为False
        """
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            if self.map[grid_x][grid_y] == 0 or [grid_x, grid_y] == self.goal_grid_pos:
                return True
            else:
                return False
        else:
            return False

    def findPath(self, node: Node) -> list[list[int]]:
        """
        根据给定节点回溯到开始点，找到路径
        """
        path_x = [node.grid_pos[0]]
        path_y = [node.grid_pos[1]]
        while node.parent:
            node = node.parent
            path_x.append(node.grid_pos[0])
            path_y.append(node.grid_pos[1])

        return [path_x[::-1], path_y[::-1]]
    
if __name__ == "__main__":
    planner = JPSPlanner()
    rospy.spin()