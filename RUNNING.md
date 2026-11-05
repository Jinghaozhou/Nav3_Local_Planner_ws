# SCAN-Planner 运行指令

当前配置：Ubuntu 24.04、ROS 2 Jazzy、`5.pcd` 地图、Mode 2 三维路径点、`open_loop` 轨迹回放。

Mode 3 的外部路径接口：规划器订阅 `nav_msgs/msg/Path`。默认话题为 `/initial_path`，也可以通过
`path_topic:=/你的路径话题` 指定任意外部话题。

## 启动规划器与仿真

在第一个终端执行：

```bash
cd /home/pc/SCAN-Planner
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=false \
  navi_mode:=2 \
  keypoints_file:=/home/pc/SCAN-Planner/src/planner/plan_manage/config/keypoints.example.yaml \
  sensor_type:=lidar \
  controller_mode:=open_loop \
  use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:=/home/pc/SCAN-Planner/5.pcd
```

## 启动 RViz2

在第二个终端执行：

```bash
cd /home/pc/SCAN-Planner
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch scan_planner rviz.launch.py
```

Mode 2 会自动读取 `keypoints.example.yaml` 中的三维路径点，并使用 `open_loop` 沿 B-spline 三维轨迹回放。无需在 RViz2 中再次点击 2D Goal Pose。

## 当前路径点

```text
(3.37,  0.0, 0.3)
(2.87, -5.0, 0.3)
(8.38, -5.0, 1.8)
(2.87, -5.0, 3.3)
(8.38, -5.0, 4.8)
```

## 重新构建

如果修改了源码、launch 文件或参数文件，先执行：

```bash
cd /home/pc/SCAN-Planner
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 使用外部 Path 话题（Mode 3）

先启动规划器（下面以仿真、PCD 地图和闭环控制为例）：

```bash
cd /home/pc/SCAN-Planner
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=false \
  navi_mode:=3 \
  path_topic:=/pct_path \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:=/home/pc/SCAN-Planner/building2_9.pcd
```

你的上游节点应发布：

```text
类型：nav_msgs/msg/Path
话题：/global_path（或 path_topic 指定的名称）
```

仓库内置测试发布器会把上面的 5 个路径点发布成 `nav_msgs/msg/Path`。规划器启动并收到里程计后，在另一个终端执行：

```bash
cd /home/pc/SCAN-Planner
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run scan_planner publish_test_path.py --topic /global_path
```

发布器默认使用 `world` 坐标系，并抵消 Mode 3 内部增加的 `0.4 m` 机身高度，因此规划器最终采用的 z 仍是上面列出的 `0.3/0.3/1.8/3.3/4.8 m`。如果修改了 `grid_map.body_height`，同步指定例如 `--planner-z-offset 0.5`。

注意事项：

- 路径点必须使用规划器的 `world` 坐标；当前 Mode 3 不根据 `Path.header.frame_id` 做 TF 转换。
- 回调会对每个输入点执行 `z_out = z_in + grid_map.body_height`，默认增加 `0.4 m`。如果你的路径 z 已经是机身中心高度，请在参数中把 `grid_map.body_height` 调整为 `0.0`，或在上游路径中提供相应的地面高度。
- 路径消息应包含至少一个点；通常建议发布完整的全局路径折线，而不是只发布终点。收到新路径后会重新生成全局轨迹，并继续进行局部避障和重规划。
- 规划器需要先收到 `body_pose`/里程计；启动后再发布路径最稳妥。可用 `ros2 topic echo /quad_0/body_pose`（仿真）或实际机器人对应的里程计话题确认。

接口检查：

```bash
ros2 topic info /global_path
ros2 topic echo /global_path --once
ros2 topic echo /planning/bspline
```
