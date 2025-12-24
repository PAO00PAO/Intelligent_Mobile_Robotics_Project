from flight_environment import FlightEnvironment
from path_planner import AStarPlanner
from trajectory_generator import TrajectoryGenerator
import numpy as np

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
env.plot_cylinders(path)

# --------------------------------------------------------------------------------------------------- #
# 轨迹生成：将离散路径点转换为平滑轨迹
print("正在生成平滑轨迹...")
traj_gen = TrajectoryGenerator(total_time=15.0)  # 总飞行时间15秒
t, trajectory, waypoint_times = traj_gen.generate(path)

# 可视化轨迹时间历程（三个子图）
traj_gen.plot_trajectory(t, trajectory, path, waypoint_times)

# You must manage this entire project using Git. 
# When submitting your assignment, upload the project to a code-hosting platform 
# such as GitHub or GitLab. The repository must be accessible and directly cloneable. 
#
# After cloning, running `python3 main.py` in the project root directory 
# should successfully execute your program and display:
#   1) the 3D path visualization, and
#   2) the trajectory plot.
#
# You must also include the link to your GitHub/GitLab repository in your written report.
