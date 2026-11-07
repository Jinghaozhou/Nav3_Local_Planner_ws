# ROS 2 模块化实施记录

初始记录：2026-11-06；测试包调整补记：2026-11-07。目标环境：ROS 2 Humble、C++17。开发分支：`refactor/ros2-integration`；本地初始基线为 `main` 上的 `11224ff`。

本轮只修改 SCAN-Planner 工作空间。Fast-LIO、KISS、PCT 的接口依据源码核查，未修改其工作空间，也未启动真实雷达、定位栈或底盘。根目录 `LICENSE` 保持不变；后续迁入 Go2 模型资源时，`NOTICE` 增补其 BSD-3-Clause 来源。

## 已完成的结构调整

| 目录 | 当前职责 |
|---|---|
| [scan_planner](../src/scan_planner) | 单一功能包：内部纯 C++ 库负责地图、投影 A*、B-spline、L-BFGS、三维路径裁剪、局部规划和跟踪计算；ROS 节点负责输入适配、TF、差分速度、异步规划、独立控制、执行检查及标准输出 |
| [scan_planner_msgs](../src/scan_planner_msgs) | Bspline、PlannerStatus、ExecutionState；保留 DataDisp 类型供旧代码对照 |
| [scan_planner_demo](../src/scan_planner_demo) | PCD 局部地图、PCD→SDF 静态碰撞场景、Go2/Gazebo 测试步态、三维里程计桥接和路径发布 |
| [legacy](../legacy) | 旧规划器、平面演示和原 simulator 对照；由 COLCON_IGNORE 排除，不作为当前构建或运行入口 |

旧代码没有继续作为新模块的运行依赖。保留对照目录用于追溯 [问题报告](README.md) 中的原始依据；报告链接已指向该目录。

本次又将原先的 `core` 与 ROS 适配两个功能包合并为 `scan_planner`：公开算法声明位于 `include/scan_planner/core/`，算法实现位于 `src/core/`，ROS 节点实现和私有头文件位于 `src/ros2/`。CMake 仍将算法编译为不依赖 ROS 的 `scan_planner_core` 库，并从 `scan_planner` 包导出供演示程序使用。消息和演示继续分别由 `scan_planner_msgs`、`scan_planner_demo` 承载。对外话题及消息协议未因包名合并而修改。

## 集成问题处理情况

| 问题 | 本轮实现 | 仍需外部确认或实测 |
|---|---|---|
| INT-01 里程计未填速度 | 默认 pose_difference，按采集时间差分并滤波；显式支持 odometry_twist | 实际速度精度、延迟；是否另行修改 Fast-LIO |
| INT-02 IMU 与机身参考点 | robot_frame/control_frame 分开；通过安装 TF 转换位姿及参考点速度 | 真实安装外参、碰撞模型尺寸、底盘速度参考点 |
| INT-03 累积地图语义 | 使用完整快照替换障碍层，不对历史点执行当前帧射线清空 | 动态物体残影、地图覆盖、地面支撑与悬崖信息 |
| INT-04 数据新鲜度 | ROS 采集时间和接收间隔双重检查；重复地图时间戳不刷新有效期 | 现场频率与超时阈值 |
| INT-05 坐标转换 | map 路径转入 odom 规划；查询权威 TF 链并检查位姿一致性 | 上游 TF 与里程计是否遵守约定 |
| INT-06 高度偏移 | reference/ground 两种模式，独立 path_z_offset；默认不额外加高 | PCT 已有偏移与 IMU/机身实际高度对应关系 |
| INT-07 空路径 | 撤销任务、失效旧轨迹、停车，保留任务版本隔离 | 与上层任务管理策略联合验收 |
| INT-08 定位质量 | 不把 TF 时间戳等同质量；提供标准 Bool 运行许可入口 | KISS 自定义状态到标准许可的适配器未在本轮开发 |
| INT-09 重定位修正 | 变换跳变时停车、作废旧计算结果、重新锚定路径进度 | 大小修正阈值、现场漂移与恢复策略 |
| INT-10 执行超时 | 独立控制节点检查心跳、数据、任务/轨迹编号及绝对有效期 | 底盘驱动的命令超时停车与制动性能 |
| INT-11 QoS | 路径 Reliable + Transient Local；传感器 Best Effort，可接当前 Reliable 发布者 | 其他传输配置需要双方共同核对 |
| INT-12 三维能力 | 保留分段高度参考和三维路径进度；维持投影搜索与固定 z 优化的能力边界 | 足步规划、地形跟随、真实楼梯执行未实现 |
| INT-13 到达判定 | 按原终点的实测 XY、Z 和速度判断；完成状态保持到新路径 | 实际终点容差与底盘停止精度 |
| INT-14 全局地图同源 | 明确记录外部约定，未改造地图服务 | KISS 与 PCT 实际地图坐标、版本和楼层基准 |
| INT-15 密度和实时性 | 配置地图范围/分辨率，完整轨迹复检，新地图可撤销活动轨迹 | 薄障碍、动态障碍、全负载延迟和停车距离 |
| INT-16 包独立性 | 必需模块与 demo 分开；独立安装目录；内部样条携带 frame 和有效期 | 实机部署环境与完整可选仿真后端验收 |

## 关键实现说明

- 规划耗时计算使用独立工作线程；控制器在独立节点运行。新地图使用独立验证地图检查活动轨迹，避免与规划工作线程共享可变地图。
- 较新的快照到达时，不直接丢弃所有未发布规划结果，而是根据最新快照复检整条结果，避免高频地图使规划饥饿。
- 样条起终点使用明确边界控制点，保留实测起始速度；不再只依靠最小二乘拟合近似端点。
- 发现初始段动态约束不满足时，重新选择时间间隔、拟合与优化，避免旧版仅拉伸中间结点造成闭环停滞；向量速度/加速度通过导数控制点的凸包上界检查。
- 占据膨胀考虑障碍和查询点在各自体素内的位置不确定性，避免仅按体素中心距离漏掉对角方向障碍。
- 内部 Bspline 增加字段，不能混用旧安装目录生成的消息与新节点；所有构建、测试命令均指定 `install/merged`。

## 2026-11-06 模块合并验证记录（历史基线）

当时验证分为无 ROS 核心、ROS 适配与接口、旧平面演示闭环三层。以下旧平面演示已经迁至 `legacy/demo`，不再是当前默认测试入口。

| 验证 | 结果 | 已实际覆盖的内容 |
|---|---|---|
| Humble 模块构建 | 通过 | 合并后 `scan_planner`、`scan_planner_msgs`、`scan_planner_demo` 和 PCD 发布器共 4 个包；未依赖其他三个算法工作空间源码 |
| 无 ROS 独立构建与核心测试 | 通过 | 10 个核心场景：快照替换、未知策略、膨胀方向/体素边界、分层进度与重新锚定、绕障、全轨迹碰撞、分段高度、起始速度、时间重新拟合及跟踪停止 |
| ROS 适配单元测试 | 通过 | 速度差分、重复/回退时间、异常位置跳变、采集时间过期与未来时间 |
| ROS 接口模拟测试 | 通过 | 非单位全局 TF、IMU 安装偏移、平移/姿态一致性、新障碍撤销轨迹、空路径、错误 frame、重复旧地图、里程计中断、定位修正、不同楼层及完成状态保持 |
| 演示实际闭环测试 | 通过，约 14 秒 | 速度指令推动模拟位姿到达 `(2, 0, 0.6)` 的配置容差内；随后持续零速度；高度保持不变 |
| 测试路径发布工具 | 通过 | 独立验证 Reliable + Transient Local 发布、frame 与默认零高度偏移；仅使用 /scan_publisher_check/path |
| 文档与静态检查 | 通过 | Python 语法、package.xml、67 个本地文档链接、Git 空白检查；LICENSE/NOTICE 校验值与基线相同 |

合并后重新使用 `build/merged`、`install/merged` 从源码构建；独立 CMake 模式通过全部 10 个核心场景，完整 `colcon test-result` 为 8 个测试、0 错误、0 失败、0 跳过。ROS 集成测试曾出现断言通过但测试启动器未退出的问题；在测试描述中增加一次性事件循环唤醒后，节点正常退出，完整测试集重新运行通过。隔离环境限制 UDP 套接字，但测试通过本地 DDS 通道完成了消息交互。

曾出现的 `/home/pc/...` 测试环境错误来自未指定测试安装目录；当前命令显式使用 `--install-base install/merged`。没有修改旧安装产物或其他工作空间来掩盖该问题。

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to scan_planner scan_planner_demo \
  --build-base build/merged --install-base install/merged \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/merged/setup.bash
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 colcon test --base-paths src \
  --build-base build/merged --install-base install/merged \
  --packages-select scan_planner scan_planner_demo \
  --event-handlers console_direct+ --ctest-args --output-on-failure
colcon test-result --test-result-base build/merged --verbose
```

测试话题为 `/scan_it` 和 `/scan_demo` 下的模拟输入输出，不向真实 `/cmd_vel` 发布。

## 2026-11-07 测试包调整与待验收项

原 `src/simulator` 已归档至 `legacy/simulator`；当前 `colcon list --base-paths src` 仅发现 `scan_planner`、`scan_planner_msgs`、`scan_planner_demo`。五份本机 PCD 移至 `src/scan_planner_demo/maps`，继续由 `*.pcd` 忽略，不随 Git 或安装包分发。

测试包新增 PCD 裁剪节点、静态 PCD 体素网格转 SDF 工具、Go2 `cmd_vel` 到关节目标的初步步态节点、Gazebo 三维里程计桥接与统一启动文件。转换器对实际 `building2_9.pcd` 的示例区域生成 1019 个占据体素，二进制/ASCII PCD 测试和 PCD 节点 ROS 裁剪/发布测试通过；`build/gazebo_demo` 与 `install/gazebo_demo` 的全新构建通过，完整 `colcon test-result` 为 7 项、0 错误、0 失败。Go2 模型资源已迁入测试包并保留 BSD-3-Clause 许可。

启动时还把转换元数据中的区域边界传给局部地图节点，使 SCAN 接收的点只来自 Gazebo 静态碰撞场景对应的 PCD 区域；对应 ROS 裁剪测试已通过。

**未验收：**本机尚未安装 Gazebo Fortress、`ros_gz`、`gz_ros2_control`、`xacro`，因此未运行物理仿真、Xacro 展开、Go2 站立/行走/爬楼及完整 SCAN 闭环。`apt-get -s install` 显示当前系统的 `krb5-multidev` 与候选 `libkrb5-3`/`libk5crypto3` 版本冲突，未对系统包执行安装或升级。现有测试步态只是足端轨迹和逆运动学适配，不能据此声称稳定行走；PCD 体素碰撞网格也需要逐图检查地面连续性、孔洞和与原始点云的偏差。ROS 集成测试需要把 `ROS_LOG_DIR` 指向工作空间可写目录，避免沙箱中的 `~/.ros/log` 只读错误。

## 当前运行边界

1. 默认 `unknown_space_policy=occupied`，因为窗口点云没有自由空间观测掩码。启用 `free_inside_bounds` 是显式接受局部障碍图假设，不会增加地面支撑或悬崖检测能力。
2. 本轮未接真实 Fast-LIO、KISS、PCT 进程或 rosbag，接口模拟通过也不能等同于系统或实机导航验收。
3. 默认碰撞模型是随航向变化的双圆柱，仍需以实际机器人尺寸验证窄通道与原地转向的扫掠空间。
4. KISS 的标准定位许可适配器、Navigation Manager 和统一 Map Server 不在本轮范围。其最终滑动窗口输入调整不改变 SCAN 的基本 TF 接口。
5. 旧 OpenGL、程序地图生成和平面运动学后端已归档至 `legacy`；新 Gazebo 后端仍缺运行依赖和物理验收。

当前启动和参数说明见 [RUNNING.md](../RUNNING.md)、[集成说明](../src/scan_planner/docs/integration.md) 和 [接口协议](../src/scan_planner/docs/interfaces.md)。
