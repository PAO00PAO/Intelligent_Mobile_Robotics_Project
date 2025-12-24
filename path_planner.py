"""
In this file, you should implement your own path planning class or function.
Within your implementation, you may call `env.is_collide()` and `env.is_outside()`
to verify whether candidate path points collide with obstacles or exceed the
environment boundaries.

You are required to write the path planning algorithm by yourself. Copying or calling 
any existing path planning algorithms from others is strictly
prohibited. Please avoid using external packages beyond common Python libraries
such as `numpy`, `math`, or `scipy`. If you must use additional packages, you
must clearly explain the reason in your report.
"""
 """
A*路径规划算法实现，用于在3D环境中找到从起点到终点的无碰撞路径
"""
import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, env, step_size=0.5):
        self.env = env
        self.step_size = step_size
        # 3D空间中的26个移动方向（包含所有相邻和对角线方向）
        self.directions = [
            (dx, dy, dz) for dx in (-1, 0, 1) 
            for dy in (-1, 0, 1) 
            for dz in (-1, 0, 1) 
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

    def heuristic(self, current, goal):
        """使用欧氏距离作为启发函数"""
        return np.sqrt(np.sum((np.array(current) - np.array(goal))**2))

    def plan(self, start, goal):
        """规划从起点到终点的路径"""
        # 确保起点和终点在环境内且无碰撞
        if self.env.is_outside(start) or self.env.is_collide(start):
            raise ValueError("起点在环境外或与障碍物碰撞")
        if self.env.is_outside(goal) or self.env.is_collide(goal):
            raise ValueError("终点在环境外或与障碍物碰撞")

        # 初始化开放列表和闭合列表
        open_heap = []
        start_tuple = tuple(start)
        goal_tuple = tuple(goal)
        heapq.heappush(open_heap, (self.heuristic(start_tuple, goal_tuple), 0, start_tuple))
        
        came_from = {}
        g_score = {start_tuple: 0}

        while open_heap:
            _, current_cost, current = heapq.heappop(open_heap)

            # 到达目标点附近
            if np.linalg.norm(np.array(current) - np.array(goal_tuple)) < self.step_size:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_tuple)
                return np.array(path[::-1], dtype=np.float32)  # 反转路径

            # 生成邻居节点
            for dx, dy, dz in self.directions:
                neighbor = (
                    current[0] + dx * self.step_size,
                    current[1] + dy * self.step_size,
                    current[2] + dz * self.step_size
                )

                # 检查是否在环境内且无碰撞
                if self.env.is_outside(neighbor) or self.env.is_collide(neighbor):
                    continue

                # 计算移动代价（欧氏距离）
                step_cost = np.sqrt(dx**2 + dy**2 + dz**2) * self.step_size
                new_cost = current_cost + step_cost

                # 如果是新节点或代价更低的路径
                if neighbor not in g_score or new_cost < g_score[neighbor]:
                    g_score[neighbor] = new_cost
                    came_from[neighbor] = current
                    f_score = new_cost + self.heuristic(neighbor, goal_tuple)
                    heapq.heappush(open_heap, (f_score, new_cost, neighbor))

        raise RuntimeError("无法找到从起点到终点的路径")           











