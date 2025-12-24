无人机 A * 路径规划与轨迹生成项目
本项目实现了基于 A * 算法的无人机 3D 环境路径规划，以及三次样条插值的平滑轨迹生成，支持环境障碍物避碰与轨迹可视化。
项目结构
plaintext
Intelligent_Mobile_Robotics_Project/
├── flight_environment.py  # 飞行环境（障碍物生成、碰撞检测）
├── path_planner.py        # A*算法路径规划实现
├── trajectory_generator.py # 三次样条平滑轨迹生成
└── main.py                # 主程序（环境初始化、规划+轨迹生成+可视化）
功能说明
环境构建：
随机生成 3D 空间中的圆柱障碍物
提供边界检测与碰撞检测接口
路径规划：
自主实现 3D A * 算法，支持 18 个移动方向
基于欧氏距离的启发函数，保证路径最优性
输出无碰撞的离散路径点
轨迹生成：
三次样条插值将离散路径转换为平滑轨迹
按路径距离分配时间，保证速度均匀
轨迹严格经过所有路径点（偏差 < 0.1m）
可视化：
3D 环境 + 路径可视化（展示障碍物与规划路径）
时间 - 坐标子图（x/y/z 方向的轨迹时间历程）
运行步骤
克隆仓库到本地：
bash
运行
git clone https://github.com/你的用户名/Intelligent_Mobile_Robotics_Project.git
cd Intelligent_Mobile_Robotics_Project
安装依赖：
bash
运行
pip install numpy matplotlib scipy
运行主程序：
bash
运行
python main.py
先显示 3D 环境与路径图，关闭后显示轨迹时间历程图
核心算法
A * 路径规划：通过开放列表 / 闭合列表 + 启发函数，在 3D 空间中搜索无碰撞路径
三次样条插值：将离散路径点转换为连续平滑的轨迹，保证无人机运动的平稳性
