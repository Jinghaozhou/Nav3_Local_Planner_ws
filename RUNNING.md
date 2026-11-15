# ROS 2 Humble 运行指令

当前工作空间目录为 `/home/jinghao/Nav3_Local_Planner_ws`；PCT 工作空间为
`/home/jinghao/Nav3_Global_planner_ws`。目录重命名后，旧 `build/merged`
和 `build/gazebo_demo` 中的 CMake 缓存仍含原绝对路径。首次按本文构建前，
在本工作空间执行一次：

```bash
cd /home/jinghao/Nav3_Local_Planner_ws
rm -rf build/merged install/merged build/gazebo_demo install/gazebo_demo
```

这四个目录是生成的构建和安装产物；随后按下面命令重新构建。PCT 工作空间
的安装环境若仍指向旧目录，也需在其工作空间重新构建后再启动 PCT 节点。

先构建新模块，再使用 `install/merged/setup.bash`；根目录旧 `install/setup.bash` 可能包含其他机器的历史路径。旧 `scan_planner run.launch.py` 已由新集成入口取代，源码对照位于 `legacy/`。

## 构建与接入外部模块

```bash
cd /home/jinghao/Nav3_Local_Planner_ws
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
`gz_ros2_control`、`ros2_controllers`、`xacro` 与 RViz。本机现已安装这些
组件，并在无界面 Gazebo 中验证了以下 `building2_9.pcd` 示例。

```bash
colcon build --base-paths src --packages-up-to scan_planner_demo \
  --build-base build/gazebo_demo --install-base install/gazebo_demo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/gazebo_demo/setup.bash
```

将 `building2_9.pcd` 的**全部有效 XYZ 点**转换成 Gazebo 静态碰撞场景。
`--all` 自动计算覆盖整份点云的边界，避免手填裁剪范围漏点；0.20 m
体素会近似原始表面。转换器不程序生成地形、不补地面，也不模拟激光雷达：

```bash
python3 src/scan_planner_demo/scripts/pcd_to_sdf.py \
  src/scan_planner_demo/maps/pcd/building2_9.pcd \
  --output src/scan_planner_demo/maps/gazebo/building2_9_full.sdf \
  --all --resolution 0.20
```

当前点云有 853,227 个有效点，完整转换得到 62,463 个占据体素、
111,088 个外表面网格面，OBJ 约 24.4 MB。完整网格覆盖
`x∈[-14.4,13.8)、y∈[-6.0,8.8)、z∈[-0.6,15.2)` m，
PCD 与 OBJ/SDF 保留相同世界坐标。Gazebo Fortress 已解析并加载完整
场景。`x∈[2,6]、y∈[0,2]` 区域有较连续的 `z≈0` 地面，旧的局部场景
在出生点 `(3,1)` 验证过站立、局部轨迹和速度发布；使用 CHAMP
步态后，完整场景中的 SCAN 闭环已到达下述 PCT 路线终点并停车。
点云原点附近是地面边缘，不能把
任意位置当作可通行地形。局部地图节点仍按机器人位置裁剪 PCD，只改变
SCAN 当前看到的局部范围，不会裁剪 Gazebo 的碰撞网格。同名 JSON 记录
完整边界及来源，启动时会核对 PCD、碰撞网格和 SDF。

启动独立测试：

```bash
cd /home/jinghao/Nav3_Local_Planner_ws
source /opt/ros/humble/setup.bash
source install/gazebo_demo/setup.bash
ros2 launch scan_planner_demo simulation.launch.py \
  pcd_map_file:="$PWD/src/scan_planner_demo/maps/pcd/building2_9.pcd" \
  world_file:="$PWD/src/scan_planner_demo/maps/gazebo/building2_9_full.sdf" \
  scenario_config:="$PWD/src/scan_planner_demo/config/scenario.building2_9.yaml" \
  spawn_x:=3.0 spawn_y:=1.0 spawn_z:=0.53 rviz:=true
```

### 使用 PCT Planner 实时提供参考路径

演示默认 `path_source:=scenario`，读取上面的 `scenario_config`。
改用 `path_source:=pct` 后，不启动固定路径发布器，也不需要
`scenario_config`；demo 仍启动 Gazebo、Go2、PCD 局部地图、SCAN 和 RViz。
它订阅 PCT 的 `/pct/global_path`，核对输入 `map` frame，再将同一份
PCD 坐标中的路径发布为 `/scan_demo/global_path`。默认对 z 减去
0.10 m，使 PCT 当前约 0.60 m 的参考高度对应此仿真模型约
0.50 m 的机身高度；其他地图或模型需重新标定偏移。PCT 规划失败
发布的空路径会清除 demo 中的旧任务。

终端 1：启动 demo。可把 `rviz:=true` 改成
`rviz:=false headless:=true` 用于无界面测试。

```bash
cd /home/jinghao/Nav3_Local_Planner_ws
source /opt/ros/humble/setup.bash
source install/gazebo_demo/setup.bash
ros2 launch scan_planner_demo simulation.launch.py \
  path_source:=pct \
  pcd_map_file:="$PWD/src/scan_planner_demo/maps/pcd/building2_9.pcd" \
  world_file:="$PWD/src/scan_planner_demo/maps/gazebo/building2_9_full.sdf" \
  spawn_x:=3.0 spawn_y:=1.0 spawn_z:=0.53 rviz:=true
```

终端 2：启动实际 PCT Planner。两工作空间的 `building2_9.pcd`
已核对为同一文件内容；切换地图时应再次核对 PCD 坐标和碰撞场景。

```bash
cd /home/jinghao/Nav3_Global_planner_ws
source src/pct_planner/scripts/activate_runtime.bash
ros2 launch pct_planner pct_planner.launch.py \
  map:=building2_9.pcd map_dir:="$PWD/map"
```

终端 3：调用 PCT 服务，成功后它才发布实时参考路径。请求起点应
靠近当前 Go2 在地图中的位置；下例对应刚出生的 `(3,1)`。首次请求
可能需要计算或读取层析缓存。

```bash
cd /home/jinghao/Nav3_Global_planner_ws
source src/pct_planner/scripts/activate_runtime.bash
ros2 service call /pct/plan pct_planner_interfaces/srv/Plan "{
  start: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0, z: 0.0}}},
  goal:  {header: {frame_id: map}, pose: {position: {x: 9.0, y: -1.0, z: 0.0}}}
}"
```

确认服务返回 `success: true`，再查看 `/scan_demo/global_path` 和
`/scan_demo/scan_planner/status`。PCT 路径、demo 局部点云和 Gazebo
碰撞网格共享 PCD 坐标，适配器只做 frame 名称和已配置的高度偏移，
不会自动进行地图配准。若 PCT 使用其他路径话题，可传
`pct_path_topic:=/实际话题`；高度偏移可用
`pct_path_z_offset:=-0.10` 调整。RViz 的 **Global path** 仍订阅
`/scan_demo/global_path`，在此模式下显示的是经适配的 PCT 路径。

### 高度压缩后的 building2_9 地图

为单独测试较低台阶，已从原始 PCD 生成
`src/scan_planner_demo/maps/pcd/building2_9_z075.pcd`：XY 保持不变，
高度以 `z=0` 为基准乘以 `0.75`。同一份 PCD 已复制到
`/home/jinghao/Nav3_Global_planner_ws/map/building2_9_z075.pcd`，两处 SHA-256
一致。完整 Gazebo 场景为
`src/scan_planner_demo/maps/gazebo/building2_9_z075_xy15_z05.sdf`。
此场景的网格尺寸为 XY `0.15 m`、Z `0.05 m`；整图各方向都用
`0.05 m` 会产生约 69 万个占据体素，因此采用更适合高度验证的
各向异性网格。新场景实际包含 150,132 个占据体素。

以下是该地图的生成命令。PCD 已存在时，缩放脚本会拒绝覆盖；重新生成
时先移走旧副本，然后依次重建 SDF/OBJ/JSON 并重新复制给 PCT：

```bash
cd /home/jinghao/Nav3_Local_Planner_ws
python3 src/scan_planner_demo/scripts/scale_pcd_z.py \
  src/scan_planner_demo/maps/pcd/building2_9.pcd \
  src/scan_planner_demo/maps/pcd/building2_9_z075.pcd \
  --factor 0.75 --datum 0.0
python3 src/scan_planner_demo/scripts/pcd_to_sdf.py \
  src/scan_planner_demo/maps/pcd/building2_9_z075.pcd \
  --output src/scan_planner_demo/maps/gazebo/building2_9_z075_xy15_z05.sdf \
  --all --resolution-xyz 0.15 0.15 0.05 --max-voxels 200000
install -m 644 src/scan_planner_demo/maps/pcd/building2_9_z075.pcd \
  /home/jinghao/Nav3_Global_planner_ws/map/building2_9_z075.pcd
```

在 `y∈[5.25,5.40) m` 的楼梯截面，生成后的碰撞顶面从
`0.05 m` 逐级升至 `0.15、0.25、0.35、0.50 m` 等；该截面相邻
台阶约为 `0.10–0.15 m`，并非每阶都恰好 `0.15 m`。其他截面和
Go2 的爬楼效果仍待验证。

后续从这份副本进行 PCT 全局规划时，在 PCT 终端选择新地图：

```bash
cd /home/jinghao/Nav3_Global_planner_ws
source src/pct_planner/scripts/activate_runtime.bash
ros2 launch pct_planner pct_planner.launch.py \
  map:=building2_9_z075.pcd map_dir:="$PWD/map"
```

准备跨楼层联调时，在另一个终端启动 SCAN demo：

```bash
cd /home/jinghao/Nav3_Local_Planner_ws
source /opt/ros/humble/setup.bash
source install/gazebo_demo/setup.bash
ros2 launch scan_planner_demo simulation.launch.py \
  path_source:=pct \
  pcd_map_file:="$PWD/src/scan_planner_demo/maps/pcd/building2_9_z075.pcd" \
  world_file:="$PWD/src/scan_planner_demo/maps/gazebo/building2_9_z075_xy15_z05.sdf" \
  scan_config_file:="$PWD/src/scan_planner_demo/config/scan_demo_z075.yaml" \
  downsample_resolution:=0.05 \
  pct_path_z_offset:=-0.23 \
  path_z_offset:=0.05 \
  spawn_x:=3.0 spawn_y:=1.0 spawn_z:=0.40 rviz:=true
```

此处保留 5 cm 偏移作为对照；后续 10 cm 偏移的实验命令与结果
仅保存在本地 `report/`，仓库不包含该记录。

缩高地图使用专用的 `scan_demo_z075.yaml`，将 SCAN 占据栅格分辨率
设为 `0.05 m`，同时把输入局部点云降采样设为 `0.05 m`。原始地图示例
仍默认使用 `scan_demo.yaml` 和 `0.10 m` 降采样。此前在机器人约
`(5.00, 0.18, 0.37) m` 停车时，SCAN 报告局部轨迹碰撞；用停点
快照离线重建，该候选轨迹在 `0.10 m` 栅格下被拒绝，在 `0.05 m`
栅格下通过。5 cm 栅格实跑又在约 `(5.67, 0.01, 0.37) m` 停住，
原因仍为局部轨迹碰撞。此处按单独试验要求添加 SCAN 的
`path_z_offset:=0.05`：PCT 适配器继续减去 `0.23 m`，SCAN 在接收
参考路径时再增加 `0.05 m`，净高度偏移为 `-0.18 m`。它不是在
碰撞检查后改动输出轨迹；局部轨迹仍从当前里程计高度起步。RViz
的 **Global path** 显示适配器输出，不包含 SCAN 内部的 5 cm 偏移；
**Local path** 才显示实际规划轨迹。5 cm 偏移的无界面闭环在约
`(6.08,-0.14,0.37) m` 再次因 SCAN 局部轨迹碰撞停车。提高路径
目标不能让 Go2 自动抬高机身，楼梯通行尚未验证成功。

待 PCT 服务就绪，在第三个终端请求缩高地图的跨层路径：

```bash
cd /home/jinghao/Nav3_Global_planner_ws
source src/pct_planner/scripts/activate_runtime.bash
ros2 service call /pct/plan pct_planner_interfaces/srv/Plan "{
  start: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0, z: 0.0}}},
  goal:  {header: {frame_id: map}, pose: {position: {x: 9.0, y: -1.0, z: 3.45}}}
}"
```

这组起终点已在缩高 PCD 上经过**离线 PCT**验证：分别选中
`0.0 m` 和 `3.375 m` 的地面，返回 136 点路径。此前使用 SCAN
`0.10 m` 栅格的联调在约 `(5.00, 0.18, 0.37) m` 停止，状态原因为
`Final whole trajectory is in collision`；不加 SCAN 高度偏移的
`0.05 m` 栅格联调又在约 `(5.67, 0.01, 0.37) m` 遇到同样的
碰撞拒绝。上面加 `path_z_offset:=0.05` 的无界面复现继续行进到约
`(6.08,-0.14,0.37) m`，但仍因碰撞拒绝停车。后续 10 cm 偏移
复现记录仅保存在本地 `report/`。
`spawn_z:=0.40` 和
`pct_path_z_offset:=-0.23` 是由新场景低层地面碰撞顶面约 `0.05 m`、
旧模型机身离地约 `0.32 m` 以及 PCT 路径起点 `0.60 m` 推算的
试验值，需要用新场景里程计与路径高度检查。PCT 会为新文件生成
独立 Tomogram 缓存。完整实验命令与结果字段仅保存在本地 `report/`。

默认 `gait_controller:=champ` 使用 CHAMP 步态核心及
[`config/gait_champ.yaml`](src/scan_planner_demo/config/gait_champ.yaml)，
针对当前 Go2 位置关节控制器使用 50 Hz 目标频率。需要比较原测试步态时，
在启动命令末尾加入 `gait_controller:=simple`；原参数位于
`config/gait.yaml`。两种步态都订阅 `/scan_demo/cmd_vel`，发布到相同的
关节轨迹控制器，不能同时运行。CHAMP 的代码来源和许可见
[`third_party/champ/README.md`](src/scan_planner_demo/third_party/champ/README.md)。
演示的 `scan_demo.yaml` 为 Gazebo `twist` 速度设置了 0.30 秒时间滤波，
使短时步态起伏不会直接成为下一条轨迹的起点速度；持续超速仍会
导致规划器拒绝新轨迹。独立接入 Fast-LIO 时默认关闭此滤波，需按
实际里程计和机身动态另行标定。
无界面复测中，速度尖峰引起的间歇停车已消失；另一轮运行仍曾因
局部轨迹碰撞和偏离参考走廊而停车。遇到停顿时检查
`/scan_demo/scan_planner/status` 与
`/scan_demo/scan_planner/execution_state` 的 `reason`，区分规划拒绝、
先转向和任务完成。

在无图形界面的环境中将最后的 `rviz:=true` 改为 `rviz:=false headless:=true`。
启动此命令前，先关闭单独使用 `ign gazebo ...building2_9_full.sdf` 打开的
Gazebo；两个同名世界同时运行可能干扰模型创建与话题连接。Gazebo 窗口
显示完整静态碰撞网格，RViz 的 **Local map** 显示机器人附近裁剪后的
PCD 点云，不显示整张碰撞网格。RViz 会在 `joint_state_broadcaster`
启动后打开；初次载入网格、里程计与点云仍可能需要数秒。

若 RViz 仍未显示 Go2 或局部点云，可在另一个已 source 相同环境的终端检查：

```bash
ros2 topic echo --once /joint_states --field name
ros2 topic echo --once /scan_demo/odom --field header.frame_id
ros2 topic echo --once /scan_demo/local_map --field width
```

正常情况下分别有 12 个关节名、`scan_demo_odom` 和大于零的点数。

`scenario.building2_9.yaml` 只适用于上述 PCD 内的连续地面区域，不能作为其他地图的
导航路线。现有路线来自 PCT Planner 对同一份 `building2_9.pcd` 的
`/pct/plan` 规划请求：起点 `(3,1,0)`，目标 `(9,-1,0)`，共 46 点、
长约 6.24 m。PCT 原始输出高度约 0.60 m；演示路径减去 0.10 m，
使 Go2 的机身参考高度约为 0.50 m。完整场景的 0.20 m 体素地面上表面
约在 z=0.20 m，故 Go2 出生高度设为 0.53 m；沿这条路线的采样点均
有同高度碰撞地面。两工作空间使用哈希相同的 PCD，
演示把 PCT 的 `map` 坐标直接对应到 `scan_demo_map`；该高度调整仅针对
当前仿真模型。
应在 RViz 中确认地面、楼梯与目标后改写 `waypoints`、出生
位置和 SCAN 路径高度模式。Gazebo 根据实际物理位姿发布 3D 里程计与 TF；
PCD 节点依据里程计裁剪 `/scan_demo/local_map`，SCAN 发布
`/scan_demo/scan_planner/local_path` 和 `/scan_demo/cmd_vel`。

### RViz 视角与参考路径

RViz 的 **Views** 面板默认使用 Orbit，视点对准出生区域 `(3,1)`。
在右侧三维画面上按住鼠标左键拖动可旋转视角，中键拖动或
`Shift`+左键拖动可平移视点，滚轮可缩放；先把 RViz 窗口最大化，并可
拖动 **Displays** 与三维画面之间的分隔线。`Local map` 仅显示机器人
附近的点云，改变视角不会扩展局部点云的裁剪范围。完整碰撞网格在
Gazebo 窗口中查看。

当前演示的参考路径由
[`config/scenario.building2_9.yaml`](src/scan_planner_demo/config/scenario.building2_9.yaml)
提供：`waypoints` 按 `[x1,y1,z1,x2,y2,z2,...]` 填写，至少两个点，
坐标系为 `scan_demo_map`。默认路线从约 `(3.134,0.966,0.50)` 延伸到
`(9.034,-1.034,0.50)`；这里
的 `z` 是 Go2 机身高度。修改 YAML 后重新启动演示，路径发布节点会
发送一次 `/scan_demo/global_path`。RViz 中蓝色的 **Global path** 是
参考路径，绿色的 **Local trajectory** 是 SCAN 规划结果。直接使用
RViz 的“2D Goal Pose”工具不会给当前 SCAN 输入三维参考路径。

默认 `start_scan:=true`，SCAN 接受路径并在输入有效时发布
`/scan_demo/cmd_vel`，Go2 步态节点将速度变成关节轨迹。另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/jinghao/Nav3_Local_Planner_ws/install/gazebo_demo/setup.bash
ros2 topic echo --once /scan_demo/global_path --field poses
ros2 topic echo --once /scan_demo/scan_planner/status
ros2 topic echo --once /scan_demo/scan_planner/execution_state
ros2 topic echo /scan_demo/cmd_vel
```

`status.motion_allowed=true` 且 `cmd_vel` 非零，才表示 SCAN 正在命令
Go2 运动；速度需连续观察几秒，按 `Ctrl+C` 结束。否则先看
`status.reason` 和 `execution_state.reason`。
完整场景的碰撞地面在出生点约为 z=0.20 m，因此命令使用
`spawn_z:=0.53`，路径机身高度约 0.50 m。低速接触抖动阈值、
空间预瞄和起步速度参数写在
[`config/scan_demo.yaml`](src/scan_planner_demo/config/scan_demo.yaml)。
本机无界面闭环复测中，Go2 沿约 6.24 m 的 PCT 路线从
`(3.0,1.0)` 推进至约 `(8.95,-0.98)`，对应目标 `(9.034,-1.034)`；
SCAN 状态为 `Task completed; waiting for a new reference path`，
`motion_allowed=false`，`cmd_vel` 归零。该路线包含转向，仍需在
障碍绕行和楼梯场景单独验收。

五份本机 PCD 位于 [maps/pcd](src/scan_planner_demo/maps/pcd/README.md)，
生成的 SDF、OBJ、JSON 位于 [maps/gazebo](src/scan_planner_demo/maps/gazebo/README.md)。
地图文件不随 Git 分发或安装，也不会被清理 `build/` 时删除。测试步态
通过 CHAMP 足端轨迹/逆运动学适配现有 Go2 模型；
平坦区域的站立和关节控制、完整场景的持续短距离前进已运行；
障碍绕行与爬楼仍需分别验收。仿真步态
不能据此推断真实 Go2 的速度跟踪能力。

若只需快速复现已验证的局部站立场景，可用
`--bounds 2 6 0 2 -0.1 1.2 --resolution 0.10` 转换为
`maps/gazebo/building_flat.sdf`。原先的 `map.pcd` 平地场景也仍可使用：转换边界为
`--bounds -1 3 -1 1 0 1 --resolution 0.10`，场景配置为
`scenario.flat_map.yaml`，出生点为 `(0,0,0.43)`；其源地图是
`maps/pcd/map.pcd`，转换输出是 `maps/gazebo/flat_patch.sdf`。各场景的转换产物
分别命名，启动时会核对 PCD 与碰撞网格元数据。

需要单独检查 Go2 步态时，可在启动命令末尾追加 `start_scan:=false`，
再另开终端向 `/scan_demo/cmd_vel` 发布测试速度。此模式不会运行 SCAN，
用于隔离物理控制问题。

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 20 /scan_demo/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}}"
```

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

ROS 模拟测试只使用 `/scan_it` 命名空间，不使用真实底盘话题。
实际执行过的验证及剩余限制记录在本地 `report/`，仓库不包含该目录。

测试包的 PCD→SDF 与局部点云 ROS 测试不依赖 Gazebo；在当前沙箱运行时
把 ROS 日志写入工作空间：

```bash
mkdir -p build/gazebo_demo/ros_logs
ROS_LOG_DIR="$PWD/build/gazebo_demo/ros_logs" ROS_LOCALHOST_ONLY=1 \
  colcon test --base-paths src --build-base build/gazebo_demo \
  --install-base install/gazebo_demo --packages-select scan_planner_demo
colcon test-result --test-result-base build/gazebo_demo --verbose
```
