# ROS 2 Humble 运行指令

先构建新模块，再使用 `install/merged/setup.bash`；根目录旧 `install/setup.bash` 可能包含其他机器的历史路径。旧 `scan_planner run.launch.py` 已由新集成入口取代，源码对照位于 `legacy/`。

## 构建与接入外部模块

```bash
cd /home/jinghao/SCAN-Planner
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to scan_planner \
  --build-base build/merged --install-base install/merged \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/merged/setup.bash
ros2 launch scan_planner scan_planner.launch.py
```

该命令只启动 SCAN 的规划与控制节点。需要外部 `/fast_lio/odom`、`/fast_lio/local_map`、`/pct/global_path` 和 `map → odom → base_link` TF。尚未接入完整数据时发布零速度与等待状态。

默认未知空间按占据处理。确认地图覆盖、PCT 可通行性及机器人参考点后，若采用局部障碍图假设，显式加入 `unknown_space_policy:=free_inside_bounds`。真实机器人接入条件和参数覆盖方式见 [集成说明](src/scan_planner/docs/integration.md)。

## PCD + Go2 + Gazebo 独立测试

测试包使用 Gazebo Fortress。运行前需在 ROS 2 Humble 环境安装 `ros_gz`、
`gz_ros2_control`、`ros2_controllers`、`xacro` 与 RViz。当前机器尚未检测到
Gazebo、`ros_gz` 或 `xacro`，因此以下步骤目前只完成了源码构建与静态转换验证。

```bash
colcon build --base-paths src --packages-up-to scan_planner_demo \
  --build-base build/gazebo_demo --install-base install/gazebo_demo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/gazebo_demo/setup.bash
```

将一份 PCD 的目标区域转换成 Gazebo 静态碰撞场景。转换器只把已有点
体素化，不程序生成地形、不补地面，也不模拟激光雷达：

```bash
python3 src/scan_planner_demo/scripts/pcd_to_sdf.py \
  src/scan_planner_demo/maps/building2_9.pcd \
  --output build/gazebo_demo/demo_world/building_patch.sdf \
  --bounds -1 3 -1 1 -0.5 0.8 --resolution 0.12
```

这是转换链的示例区域；并未证明该区域可供 Go2 行走。先检查生成的
OBJ/SDF 地面和障碍，再配置出生位置与对应的三维路线。转换时只处理
指定边界内的点，PCD 与 OBJ/SDF 保留相同世界坐标；局部地图节点也使用
同一边界裁剪 PCD。同时写出同名 JSON 元数据，启动时核对 PCD、碰撞网格
和 SDF 的来源。

Gazebo 安装完成、场景与路径对齐后启动：

```bash
ros2 launch scan_planner_demo simulation.launch.py \
  pcd_map_file:="$PWD/src/scan_planner_demo/maps/building2_9.pcd" \
  world_file:="$PWD/build/gazebo_demo/demo_world/building_patch.sdf" \
  scenario_config:="$PWD/src/scan_planner_demo/config/scenario.example.yaml" \
  spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.45 rviz:=true
```

`scenario.example.yaml` 只演示参数格式，不能直接作为任意地图的成功
导航路线。应在 RViz 中确认地面、楼梯与目标后改写 `waypoints`、出生
位置和 SCAN 路径高度模式。Gazebo 根据实际物理位姿发布 3D 里程计与 TF；
PCD 节点依据里程计裁剪 `/scan_demo/local_map`，SCAN 发布
`/scan_demo/scan_planner/local_path` 和 `/scan_demo/cmd_vel`。

五份本机 PCD 位于 [测试包 maps 目录](src/scan_planner_demo/maps/README.md)，
不随 Git 分发或安装。当前测试步态节点是初步足端轨迹/逆运动学适配，
物理接触、稳定行走及楼梯攀爬尚须安装 Gazebo 后逐项验收。

调试工具迁移至 `scan_planner_demo`：

```bash
ros2 run scan_planner_demo keypoint_recorder.py --odom /fast_lio/odom --output keypoints.yaml
ros2 run scan_planner_demo publish_test_path.py --topic /pct/global_path --frame-id map
```

记录器沿用旧路径点参数文件格式，需转换成上游 Path 或演示 waypoints 后使用；集成节点不自动加载旧 fsm.waypoints。测试发布器的五个点仍来自旧场景，必须先核对地图和高度，不应直接用于未知场景。

## 查询接口与运行测试

```bash
ros2 topic echo /scan_planner/status
ros2 topic echo /scan_planner/execution_state
ros2 topic info /fast_lio/local_map --verbose
ros2 run tf2_ros tf2_echo map base_link
colcon test --base-paths src --build-base build/merged \
  --install-base install/merged --packages-select scan_planner
colcon test-result --test-result-base build/merged --verbose
```

ROS 模拟测试只使用 `/scan_it` 命名空间，不使用真实底盘话题。实际执行过的验证及剩余限制记录在 [实施报告](report/refactor_progress.md)。

测试包的 PCD→SDF 与局部点云 ROS 测试不依赖 Gazebo；在当前沙箱运行时
把 ROS 日志写入工作空间：

```bash
mkdir -p build/gazebo_demo/ros_logs
ROS_LOG_DIR="$PWD/build/gazebo_demo/ros_logs" ROS_LOCALHOST_ONLY=1 \
  colcon test --base-paths src --build-base build/gazebo_demo \
  --install-base install/gazebo_demo --packages-select scan_planner_demo
colcon test-result --test-result-base build/gazebo_demo --verbose
```
