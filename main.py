# main.py
from flight_environment import FlightEnvironment
from path_planner import AStarPlanner
from trajectory_generator import TrajectoryGenerator
import numpy as np
import matplotlib.pyplot as plt  # 确保导入matplotlib

# 确保matplotlib支持交互模式，确保图片能显示
plt.ion()

# 初始化飞行环境（包含50个障碍物）
env = FlightEnvironment(50)
start = (1, 2, 0)    # 起点坐标
goal = (18, 18, 3)   # 终点坐标

# --------------------------------------------------------------------------------------------------- #
# 路径规划：使用A*算法生成无碰撞路径
print("正在进行路径规划...")
planner = AStarPlanner(env, step_size=0.5)  # 步长0.5米
try:
    path = planner.plan(start, goal)
    print(f"路径规划完成，包含 {len(path)} 个点")
except RuntimeError as e:
    print(f"路径规划失败: {e}")
    exit(1)

# 可视化3D路径（包含障碍物）
print("显示3D轨迹与障碍物图...")
env.plot_cylinders(path)

# --------------------------------------------------------------------------------------------------- #
# 轨迹生成：将离散路径点转换为平滑轨迹
print("正在生成平滑轨迹...")
traj_gen = TrajectoryGenerator(total_time=15.0)  # 总飞行时间15秒
t, trajectory, waypoint_times = traj_gen.generate(path)

# 可视化轨迹时间历程（三个子图合并在一张图里）
print("显示时间-坐标历程图...")
traj_gen.plot_trajectory(t, trajectory, path, waypoint_times)

# 保持图片显示，直到用户关闭
plt.ioff()
plt.show()
