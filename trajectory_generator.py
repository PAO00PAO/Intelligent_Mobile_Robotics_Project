"""
In this file, you should implement your trajectory generation class or function.
Your method must generate a smooth 3-axis trajectory (x(t), y(t), z(t)) that 
passes through all the previously computed path points. A positional deviation 
up to 0.1 m from each path point is allowed.

You should output the generated trajectory and visualize it. The figure must
contain three subplots showing x, y, and z, respectively, with time t (in seconds)
as the horizontal axis. Additionally, you must plot the original discrete path 
points on the same figure for comparison.

You are expected to write the implementation yourself. Do NOT copy or reuse any 
existing trajectory generation code from others. Avoid using external packages 
beyond general scientific libraries such as numpy, math, or scipy. If you decide 
to use additional packages, you must clearly explain the reason in your report.
"""
"""
轨迹生成器：将离散路径点转换为平滑轨迹，并可视化时间历程
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

class TrajectoryGenerator:
    def __init__(self, total_time=10.0):
        self.total_time = total_time  # 轨迹总时长（秒）

    def generate(self, path_points):
        """
        生成平滑轨迹
        参数:
            path_points: N×3 numpy数组，路径点坐标
        返回:
            t: 时间数组
            trajectory: 平滑轨迹（M×3）
            waypoint_times: 路径点对应的时间点
        """
        path = np.array(path_points)
        n_points = path.shape[0]
        
        # 计算路径点之间的距离，用于分配时间
        distances = np.zeros(n_points)
        for i in range(1, n_points):
            distances[i] = distances[i-1] + np.linalg.norm(path[i] - path[i-1])
        total_distance = distances[-1] if n_points > 1 else 1.0
        waypoint_times = distances / total_distance * self.total_time  # 路径点对应的时间
        
        # 生成密集时间点用于平滑轨迹
        t = np.linspace(0, self.total_time, 1000)
        
        # 使用三次样条插值生成平滑轨迹
        trajectory = np.zeros((len(t), 3))
        for i in range(3):  # x, y, z三个维度
            spline = make_interp_spline(waypoint_times, path[:, i], k=3)
            trajectory[:, i] = spline(t)
        
        return t, trajectory, waypoint_times

    def plot_trajectory(self, t, trajectory, path_points, waypoint_times):
        """
        绘制轨迹时间历程图（三个子图）
        """
        path = np.array(path_points)
        fig, axes = plt.subplots(3, 1, figsize=(10, 12))
        labels = ['X 坐标 (m)', 'Y 坐标 (m)', 'Z 坐标 (m)']
        
        for i in range(3):
            # 绘制平滑轨迹
            axes[i].plot(t, trajectory[:, i], 'b-', linewidth=2, label='平滑轨迹')
            # 绘制原始路径点
            axes[i].scatter(waypoint_times, path[:, i], color='red', s=50, label='路径点')
            axes[i].set_xlabel('时间 (s)')
            axes[i].set_ylabel(labels[i])
            axes[i].legend()
            axes[i].grid(True)
        
        plt.tight_layout()
        plt.show()
