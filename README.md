# 无人机 A* 路径规划与轨迹生成项目
本项目实现了基于 A* 算法的无人机 3D 环境路径规划，以及三次样条插值的平滑轨迹生成，支持环境障碍物避碰与轨迹可视化。

## 项目结构
```plaintext
Intelligent_Mobile_Robotics_Project/
├── flight_environment.py  # 飞行环境（障碍物生成、碰撞检测）
├── path_planner.py        # A*算法路径规划实现
├── trajectory_generator.py # 三次样条平滑轨迹生成
└── main.py                # 主程序（环境初始化、规划+轨迹生成+可视化）
