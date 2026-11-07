# SCAN Planner ROS 2 导航模块

面向 ROS 2 Humble 的独立局部规划与速度控制模块。输入 Fast-LIO 连续里程计、滑动窗口点云、全局定位 TF 和 PCT 三维参考路径，输出局部轨迹、规划状态与底盘速度。算法核心与 ROS 通信分离，集成启动不加载外部定位算法或仿真器。

本项目衍生自 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)。核心算法、原始项目设计和研究成果归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin。原 ROS 2 移植署名及第三方声明保留在 [NOTICE](NOTICE)，本次模块化整理不代表原作者的官方发布或认可。

## 代码结构

```text
src/
├── scan_planner/         # 统一功能包；内部保留独立算法库目标
│   ├── include/scan_planner/core/ # 算法公共接口声明
│   ├── src/core/        # 地图、投影 A*、B-spline、优化及跟踪实现
│   ├── src/ros2/        # 规划/控制节点、ROS 适配及私有头文件
│   ├── config/
│   ├── launch/
│   ├── docs/
│   └── test/
├── scan_planner_msgs/    # 状态、内部样条及执行状态消息
└── scan_planner_demo/    # PCD 局部地图、Go2/Gazebo 测试、模型与回放工具
    ├── description/     # Go2 模型与许可证
    ├── maps/            # 本地 PCD，不纳入 Git
    ├── scripts/         # PCD→SDF 离线转换、路径发布
    └── src/             # PCD 裁剪、测试步态控制
legacy/                  # 旧规划器、平面演示和原 simulator；COLCON_IGNORE
report/                  # 集成问题及实施验证记录
```

`scan_planner` 内的纯 C++ 库 `scan_planner_core` 保留投影 A*、B-spline 和 L-BFGS 主要算法实现，不依赖 ROS 通信库；节点通过该库调用算法。窗口地图按完整快照更新，避免把多帧历史点当成当前帧射线；局部参考按三维路径进度裁剪，不再把整段 z 强制替换为起终点线性高度。

## 构建

当前目标为 ROS 2 Humble、C++17。集成模块不依赖其他三个算法工作空间的源码，也不依赖 Go2、PCL 或 cv_bridge。

```bash
cd /home/jinghao/SCAN-Planner
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to scan_planner \
  --build-base build/merged --install-base install/merged \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/merged/setup.bash
```

使用独立构建/安装目录，避免混入此前从其他机器复制的安装产物。测试包还需要 NumPy、PCL；Gazebo 运行需要 Fortress、`ros_gz`、`gz_ros2_control`、`xacro` 和 ROS 控制器。

## 系统接口

| 方向 | 默认接口 | 类型与约定 |
|---|---|---|
| 输入 | `/fast_lio/odom` | Odometry，默认 odom/base_link；默认按位姿差分估速 |
| 输入 | `/fast_lio/local_map` | PointCloud2，odom 中的完整滑动窗口地图快照 |
| 输入 | `/pct/global_path` | Path，map 中的三维参考路径；空路径撤销任务 |
| 输入 | TF | 消费 map→odom→robot_frame，核心 TF 由外部唯一发布者维护 |
| 输出 | `/cmd_vel` | Twist，control_frame 下的平面 vx/vy/wz |
| 输出 | `/scan_planner/status` | PlannerStatus，含状态、原因、任务/轨迹编号及执行许可 |
| 输出 | `/scan_planner/local_path` | Path，odom 中的局部轨迹采样 |

`/fast_lio/local_cloud` 是当前帧配准点云，不能与 `/fast_lio/local_map` 混为一谈。当前 Fast-LIO 的 base_link 实际为 IMU 原点，部署前应通过 TF 配置真实碰撞与控制参考点。

## 启动

```bash
ros2 launch scan_planner scan_planner.launch.py
```

默认 `unknown_space_policy=occupied`。点云快照只提供占据证据，默认不会把所有未出现的体素当成自由空间，因此缺少自由空间证据时保持零速度。若已确认局部覆盖和 PCT 可通行路线，并接受边界内未命中为空闲的障碍图假设，可显式使用：

```bash
ros2 launch scan_planner scan_planner.launch.py \
  unknown_space_policy:=free_inside_bounds
```

话题、namespace、frame、速度来源和路径高度可配置，详见 [集成说明](src/scan_planner/docs/integration.md) 和 [接口协议](src/scan_planner/docs/interfaces.md)。当前 PCT 已有高度偏移，默认保留输入 z，不再隐式增加 0.4 m。

控制节点独立检查里程计、规划心跳、任务编号、轨迹编号和有效期。输入过期、空路径、定位修正或轨迹无效时停车。实际底盘仍需有自己的命令超时停车机制。

## 演示与验证

构建隔离的 PCD/Gazebo 测试包：

```bash
colcon build --base-paths src --packages-up-to scan_planner_demo \
  --build-base build/gazebo_demo --install-base install/gazebo_demo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/gazebo_demo/setup.bash
```

测试速度话题为 `/scan_demo/cmd_vel`，不使用真实 `/cmd_vel`。需要先从同一份 PCD 转换碰撞场景、指定与地图对齐的出生位置和三维路径；完整命令见 [运行说明](RUNNING.md)。

本机 PCD 已集中放入 [测试包 maps 目录](src/scan_planner_demo/maps/README.md)。SCAN 直接读取并裁剪 PCD，不使用模拟激光雷达；Gazebo 从同一份 PCD 转出的静态网格处理碰撞。已用 `map.pcd` 平坦区域验证 Gazebo 站立、三维里程计、局部轨迹和非零速度发布。隔离测试中 Go2 能按正向指令前进和转向；SCAN 闭环仍有轨迹起始方向反复变化的问题，尚未验证目标到达或爬楼。

```bash
colcon test --base-paths src --build-base build/gazebo_demo \
  --install-base install/gazebo_demo --packages-select scan_planner scan_planner_demo \
  --event-handlers console_direct+
colcon test-result --test-result-base build/gazebo_demo --verbose
```

核心可不加载 ROS 独立测试，命令见集成说明。测试包的静态转换与构建测试不能替代 Gazebo 接触和步态验收；实机外参、路径高度、动态障碍和楼梯运动能力需要另外验证。本模块输出平面速度，不实现足步规划或底层爬楼控制。

## 报告与来源

- [集成问题核查基线](report/README.md)
- [重构实施与验证记录](report/refactor_progress.md)
- [原项目主页](https://wuyi2121.github.io/SCAN-Planner/)
- [原始论文](https://arxiv.org/abs/2606.19555)

算法实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 等项目。完整署名见 NOTICE。仓库保留 [Apache-2.0 许可证](LICENSE)；分发或派生时保留相应署名与许可信息。
