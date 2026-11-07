# 接入当前导航系统

## 构建与环境

```bash
cd /home/jinghao/SCAN-Planner
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to scan_planner \
  --build-base build/merged --install-base install/merged \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/merged/setup.bash
```

集成包依赖 Eigen、ROS 2 标准消息、TF2 与本仓库消息；不依赖 Fast-LIO、KISS、PCT、Go2 仿真、PCL 或 cv_bridge 的源码。使用单独构建/安装目录避免读取旧安装结果。`legacy/` 有 COLCON_IGNORE，仅保存重构前源码对照。

核心也可以不加载 ROS 独立编译和测试：

```bash
cmake -S src/scan_planner -B /tmp/scan_core_build \
  -DSCAN_CORE_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/scan_core_build -j2
ctest --test-dir /tmp/scan_core_build --output-on-failure
```

## 独立启动

```bash
ros2 launch scan_planner scan_planner.launch.py
```

默认话题已对应当前 Fast-LIO 与 PCT 的真实接口。需要外部 TF 链 map→odom→base_link。Fast-LIO 必须使用组合启动配置，原始 launch 的 camera_init/body 及 /Odometry 并非本配置默认接口。

默认 unknown_space_policy=occupied：仅有点云不能证明未命中体素可行，节点保持零速度并报告局部规划失败。在已确认窗口覆盖、全局路线可通行性并接受“边界内未命中为空闲”的障碍图假设后，显式选择：

```bash
ros2 launch scan_planner scan_planner.launch.py \
  unknown_space_policy:=free_inside_bounds
```

该选项并未增加地面支撑、悬崖或未知空间观测能力。核心目前为窗口障碍图，未提供带观测掩码的三态自由空间地图。

更改输入、frame、路径高度或安装参考点：

```bash
ros2 launch scan_planner scan_planner.launch.py \
  odom_topic:=/fast_lio/odom local_map_topic:=/fast_lio/local_map \
  path_topic:=/pct/global_path cmd_vel_topic:=/cmd_vel \
  global_frame:=map planning_frame:=odom \
  robot_frame:=base_link control_frame:=base_link \
  path_height_mode:=reference path_z_offset:=0.0
```

若实际规划参考点与 IMU 原点不同，应先提供真实安装外参 TF，再设置 robot_frame；底盘速度对应另一参考点时设置 control_frame。不能靠改变 frame 名称修正安装误差。

`velocity_source=pose_difference` 适配当前 Fast-LIO 未填写 twist 的实现。外部确认已经正确发布速度后才可选择 odometry_twist。姿态和速度变换包括参考点平移带来的角速度项。

算法参数位于 config/planner.yaml，控制参数位于 config/controller.yaml，公共适配位于 config/integration.yaml。通过 override_config 指定额外 ROS 参数 YAML；显式 launch 参数优先于配置文件。

## 外部模块约定

- PCT 成功或失败时发布全局 Path；空路径会撤销旧任务。路径必须使用 map，并保留必要的三维路线。
- KISS 只需提供已应用 map→odom。其输入未来由 Laser_map 切换为 fast_lio/local_map 不改变 SCAN 接口。
- KISS 的定位置信度没有被 TF 时间戳替代。可选许可适配层由系统集成方提供，需独立处理定位周期、质量和 last_success_stamp。
- 三维路径裁剪保留高度，原有投影搜索与固定 z 优化方式仍有地形适用边界。模块只输出平面 Twist，不实现足步规划或爬楼控制。
- 窗口历史点可能含动态物体残影。应根据地图延迟、计算耗时和底盘停车能力联合选择速度与膨胀尺寸。

## 验证

```bash
colcon test --base-paths src --build-base build/merged --install-base install/merged \
  --packages-select scan_planner --event-handlers console_direct+
colcon test-result --test-result-base build/merged --verbose
```

构建了演示包后，可在 packages-select 中追加 scan_planner_demo，以运行实际速度/里程计闭环测试。

ROS 集成测试只使用 /scan_it 下的模拟话题、非单位 map→odom 和独立 test frame，不启动外部算法或向真实 /cmd_vel 发命令。测试需要运行 DDS 的网络权限；执行环境阻止网络时，核心测试仍可单独运行，但不能据此宣称 ROS 集成通过。

现场接入前另需验证：真实地图同源性、IMU/机身外参、路径高度、实际消息频率与延迟、动态障碍残影、底盘制动和楼梯运动能力。代码级模拟测试不等于实机导航验收。
