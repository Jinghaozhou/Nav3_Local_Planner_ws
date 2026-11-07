# SCAN Planner 系统集成问题记录

记录日期：2026-11-05。目标环境：ROS 2 Humble。

本文记录 SCAN Planner 与 Fast-LIO、KISS 定位、PCT Planner 集成时已经从源码确认的接口差异，以及由这些差异推导出的风险、建议方案和验证方法。当前结论是：这些模块可以通过 ROS 2 接口集成，但仅统一话题名称还不够，必须明确速度、机器人参考点、地图语义、路径高度与失效停车约定。

**本文保留重构前的核查基线，不表示所有问题已经解决，也不授权修改其他工作空间。** 重构前的 SCAN 源码现位于 `legacy/planner/`，已修正对应链接；当前实现和验证结果另见 [重构实施记录](refactor_progress.md)。

## 核查范围与结论边界

核查的工作空间：

- `/home/jinghao/SCAN-Planner`
- `/home/jinghao/Fast_lio_ws`，注意目录名称的大小写。
- `/home/jinghao/kiss_localization_ws`
- `/home/jinghao/PCT_planner`

依据为源码、消息/服务定义、launch、配置及对应说明文档。以下源文件链接定位到本机核查版本，行号可能随后续修改变化。本文没有证明当前运行环境已满足这些接口：现场 ROS 图查询因执行环境的 DDS 网络权限限制未完成，未进行联合运行、rosbag 回放或实机测试。

用户已确认：

- 集成目标使用 ROS 2 Humble。
- SCAN 使用多帧累积局部地图；Fast-LIO 已实现滑动窗口输出。
- KISS 当前定位输入尚未使用滑动窗口点云，最终将切换到滑动窗口点云。
- PCT 路径 z 的物理参考点仍需明确，SCAN 应支持显式配置。

## 已核查的接口基线

Fast-LIO 下表接口以 `mid360_bringup/mid360_mapping.launch.py` 组合启动为前提；原始 FAST-LIO 启动入口的默认话题和 frame 不同。

| 模块 | 接口 | 消息类型或形式 | 已确认的含义 |
|---|---|---|---|
| Fast-LIO | `/fast_lio/odom` | `nav_msgs/msg/Odometry` | 位姿为 `odom → base_link`；当前实现没有填写 twist |
| Fast-LIO | `/fast_lio/local_cloud` | `sensor_msgs/msg/PointCloud2` | `odom` 中的当前帧配准点云，不是滑动窗口地图 |
| Fast-LIO | `/fast_lio/local_map` | `sensor_msgs/msg/PointCloud2` | `odom` 中的滑动时间窗口完整快照；默认 5 秒、最多 100 帧、2 Hz、0.15 m 体素；输出 Reliable |
| Fast-LIO | `/Laser_map` | `sensor_msgs/msg/PointCloud2` | 本次运行以来累计的历史地图，不是滑动窗口地图 |
| Fast-LIO / bringup | TF | `odom → base_link`、静态外参 | `base_link` 当前定义为 IMU 本体系；bringup 发布至 `imu_link`、`lidar` 的静态 TF |
| KISS | `map → odom` | TF | 首次成功应用定位后以 10 Hz 广播；失败后仍保留上一有效变换 |
| KISS | `/localization/status` | `kiss_matcher_localization/msg/LocalizationResult` | 包含 matched、applied、last_success_stamp 等；Reliable + Transient Local |
| KISS | `/localization/relocalize` | `kiss_matcher_localization/srv/Relocalize` | 使用最新输入地图触发定位；请求包含 mode、apply_if_safe |
| KISS | `/localization/accept_candidate` | `kiss_matcher_localization/srv/AcceptCandidate` | 应用已有候选定位 |
| PCT | `/pct/global_path` | `nav_msgs/msg/Path` | `map` 中的三维参考路径；Reliable + Transient Local，深度 1；规划失败发布空路径 |
| PCT | `/pct/plan` | `pct_planner_interfaces/srv/Plan` | 请求 start/goal；返回 success/path/message |

接口依据：[Fast-LIO 组合启动](/home/jinghao/Fast_lio_ws/src/mid360_bringup/launch/mid360_mapping.launch.py:60)、[窗口地图实现](/home/jinghao/Fast_lio_ws/src/mid360_bringup/mid360_bringup/local_map_window.py:64)、[KISS 导航接口](/home/jinghao/kiss_localization_ws/src/localization/NAVIGATION_INTERFACE.md)、[PCT ROS 节点](/home/jinghao/PCT_planner/src/pct_planner/pct_planner/ros/planner_node.py:73)。

## 问题与待讨论方案

优先级表示建议处理顺序，不代表这些问题已在实机复现。P0 为启用运动输出前必须明确或处理；P1 为持续联调前应完成；P2 为交付和维护时整理。

### INT-01 里程计没有填写速度 · P0

**当前事实：** Fast-LIO 的发布函数填写位姿和 TF，没有设置 `odomAftMapped.twist`。SCAN 的里程计回调直接读取 `msg->twist.twist.linear`，因此按当前代码连接后会读取默认零值。

**潜在影响：** 移动中的机器人可能以零速度初始化规划，影响轨迹连续性、重规划以及停止状态判断。不能通过“twist 恰好为零”自动判断速度字段无效，因为静止时零速度本身合法。

**建议方案：** 显式配置速度来源。SCAN 可提供按消息时间戳进行的位姿差分及滤波，处理初始化、异常时间间隔和位姿跳变。上游发布真实滤波速度作为另一条独立讨论的改动；若使用消息速度，还需明确其参考坐标系并转换到规划坐标系。

**验证方法：** 录制静止、匀速、转弯和停车数据，对比位移与速度估计，检查单位、方向、延迟和重规划连续性。

依据：[Fast-LIO 里程计发布函数](/home/jinghao/Fast_lio_ws/src/FAST_LIO/src/laserMapping.cpp:632)、[SCAN 速度读取](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_replan_fsm.cpp:383)。

### INT-02 base_link 实际是 IMU 本体系 · P0

**当前事实：** Fast-LIO bringup 明确将 `base_link` 定义为 IMU 本体系，并向 `imu_link` 发布恒等静态变换。它不一定等于 Go2 机身中心、碰撞模型中心或底盘控制原点。

**潜在影响：** 即使 TF 名称完全匹配，仍可能出现机器人轮廓偏移、路径高度错误、速度方向错误或参考点速度不一致。存在安装旋转或平移时，仅改 frame 名称无法解决问题。

**建议方案：** 区分定位参考点、规划碰撞模型参考点与底盘控制参考点，记录真实安装外参。SCAN 在适配层完成必要转换；如果将来统一 TF 命名，需同时协调 Fast-LIO、KISS 和机器人模型，避免重复发布或形成 TF 环。

**验证方法：** 在 RViz 中核对 IMU、雷达、机身中心和碰撞轮廓；转动机器人验证外参和参考点运动是否符合实际。

依据：[Fast-LIO frame 定义](/home/jinghao/Fast_lio_ws/src/mid360_bringup/README.md:115)、[静态 TF 启动](/home/jinghao/Fast_lio_ws/src/mid360_bringup/launch/mid360_mapping.launch.py:71)。

### INT-03 当前帧点云与滑动窗口地图不能混用 · P0

**当前事实：** `/fast_lio/local_cloud` 是当前帧；真正的滑动窗口地图为 `/fast_lio/local_map`。窗口节点每条消息重新输出当前窗口全部点，并未执行动态物体剔除。SCAN 当前地图实现使用雷达原点到点云终点的射线更新。

**潜在影响：** 将多帧地图当作单次雷达观测，可能沿不真实的射线清除障碍；持续累加窗口快照可能保留已经离开窗口的旧障碍。动态物体也可能在窗口内留下历史痕迹。

**建议方案：** 集成模式使用 `/fast_lio/local_map`，按完整快照更新障碍层，在 SCAN 内按机器人位置裁剪空间范围，明确未知空间策略。单帧 raycast 与地图快照两种模式分开，不从累积点云推出当前雷达的自由射线。

**验证方法：** 移动障碍进入和离开视野，检查障碍保留时间；机器人持续移动后检查旧区域清理、地图点数和内存。完整窗口是完整数据快照，不等于窗口包围范围内所有未出现的体素都已被观测为空闲。

依据：[窗口聚合与发布](/home/jinghao/Fast_lio_ws/src/mid360_bringup/mid360_bringup/local_map_window.py:81)、[SCAN 点云回调](/home/jinghao/SCAN-Planner/legacy/planner/plan_env/src/grid_map.cpp:872)、[SCAN 射线处理](/home/jinghao/SCAN-Planner/legacy/planner/plan_env/src/grid_map.cpp:542)。

### INT-04 重复收到消息不代表数据仍然新鲜 · P0

**当前事实：** Fast-LIO 窗口节点在输入停止后仍可定时发布最后窗口，但保留最后一帧采集时间。SCAN 当前控制器主要用 `have_odom_` 等接收标志判断是否具备输入，没有完整的数据时效门控。

**潜在影响：** 雷达或定位输入已经中断，下游却因持续收到旧地图或曾经收到过里程计而继续运动。

**建议方案：** 分别检查采集时间、接收时间和轨迹有效期。超时、未来时间、时钟回跳或输入时间线重置时停车并清理相关缓存。超时应结合地图默认 2 Hz 和实际抖动确定，不能照搬高频里程计阈值；rosbag 的 `/clock` 与墙钟也要区分。

**验证方法：** 暂停雷达、停止里程计、暂停与回退 rosbag，确认在规定时限内停车并报告具体原因。记录中保留重复时间戳，防止仅按到达频率误判。

依据：[窗口时间戳与重复发布](/home/jinghao/Fast_lio_ws/src/mid360_bringup/mid360_bringup/local_map_window.py:100)、[SCAN 控制循环](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/closed_loop_controller.cpp:120)。

### INT-05 map 路径尚未经过 TF 转换进入局部规划 · P0

**当前事实：** PCT 输出 `map` 路径；Fast-LIO 输出 `odom` 点云和里程计。SCAN 默认地图 frame 为 `world`，路径回调直接使用输入 XYZ，未根据 `Path.header.frame_id` 做坐标转换，也没有在该回调中检查是否已有里程计。

**潜在影响：** 接口能连通但数据不在同一坐标系，导致路径、机器人和障碍错位；路径先到达时还可能使用尚未就绪的机器人状态。

**建议方案：** 保留 `map` 全局路径，在连续的 `odom` 中维护局部地图和局部轨迹。通过 TF2 转换所需路径段，按消息时间处理点云。没有有效状态或必要 TF 时缓存/拒绝输入并保持停车，不隐式假设 `map=odom=world`。

**验证方法：** 使用含非零平移和旋转的 `map → odom`，并覆盖路径先到、TF 后到和 frame 错误情况。仅用单位变换测试不足以证明适配正确。

依据：[SCAN 路径接收](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_replan_fsm.cpp:339)、[SCAN 默认 frame](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/config/planner.yaml:27)。

### INT-06 路径高度已有偏移，可能重复增加机身高度 · P0

**当前事实：** SCAN 对 Mode 3 每个路径点执行 `z + body_height`，默认增加 0.4 m。PCT 默认五次轨迹分支在高度平滑前加入默认 0.1 m 参考高度，坐标输出转换还固定增加 0.5 m。高度平滑会进一步影响最终结果，因此不能将所有输出点简单等同于“地面 + 0.6 m”。

**潜在影响：** 对 PCT 路径再无条件增加 0.4 m，可能使规划参考点脱离真实机身高度，影响地面、楼层和低矮障碍判定。PCT 当前参考点也尚不能直接等同于 Fast-LIO 的 IMU 原点。

**建议方案：** 显式定义输入路径参考点、SCAN 碰撞参考点和二者的转换。接当前 PCT 时禁止默认重复抬高；最终偏移通过地图、路径和机器人安装外参核对后确定，不能依赖猜测或运行时自动推断。PCT 自身固定偏移是否参数化应单独讨论。

**验证方法：** 在已知地面高度的平地、坡道和不同楼层采样路径 z，与实际 IMU/机身高度和碰撞轮廓比较。

依据：[SCAN 高度调整](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_replan_fsm.cpp:353)、[PCT 高度生成](/home/jinghao/PCT_planner/src/pct_planner/native/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc:171)、[PCT 默认参考高度](/home/jinghao/PCT_planner/src/pct_planner/native/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h:66)、[PCT 输出转换](/home/jinghao/PCT_planner/src/pct_planner/pct_planner/core/conversion.py:4)。

### INT-07 PCT 失败发布空路径，SCAN 当前会忽略 · P0

**当前事实：** PCT 在规划失败时发布空 `Path` 清除前次结果；SCAN 对空路径只记录日志并返回，不撤销旧任务。

**潜在影响：** 上游已经宣告无可行路径，下游仍可能沿先前路径运动。

**建议方案：** 将空路径约定为撤销当前参考路径，清理旧轨迹执行许可，输出零速度并报告 `GLOBAL_PATH_INVALID`。与控制节点共享任务版本，拒绝迟到的旧任务规划结果。

**验证方法：** 先成功规划并执行，再让 PCT 返回失败，确认旧路径、旧样条和迟到结果均不能重新使机器人运动。

依据：[PCT 失败处理](/home/jinghao/PCT_planner/src/pct_planner/pct_planner/ros/planner_node.py:171)、[SCAN 空路径处理](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_replan_fsm.cpp:339)。

### INT-08 TF 持续更新不代表最近全局定位成功 · P0

**当前事实：** KISS 只在结果被应用后更新内部定位变换，但随后以当前时间持续广播该变换。失败后也保留旧变换；状态消息区分 `matched`、`applied` 和 `last_success_stamp`，匹配成功的候选并不一定已应用。

**潜在影响：** 只检查 TF 的最新时间，会把长期没有成功校正的定位误判为健康；直接使用候选结果会与 TF 中已应用定位不一致。

**建议方案：** 区分 TF 链可用性、定位修正变化和定位质量。SCAN 核心消费已应用 TF；需要质量门控时，通过独立适配层读取 `/localization/status`，或由上层输出统一运行许可。有效期须结合 KISS 默认 60 秒周期与实测漂移确定，不能将一次配准失败直接等同于定位完全失效。

**验证方法：** 首次定位成功后制造配准失败、只生成未应用候选、长时间不再成功校正，检查导航状态与恢复策略。

依据：[KISS 状态定义](/home/jinghao/kiss_localization_ws/src/localization/msg/LocalizationResult.msg)、[KISS TF 广播](/home/jinghao/kiss_localization_ws/src/localization/src/navigation_localizer.cpp:512)、[KISS 默认周期参数](/home/jinghao/kiss_localization_ws/src/localization/config/navigation_localization.yaml:11)。

### INT-09 重定位修正可能使旧局部轨迹失效 · P0

**当前事实：** KISS 可应用新的 `map → odom`，并支持候选确认。SCAN 当前没有与全局定位修正相对应的路径版本管理和轨迹作废流程。

**潜在影响：** 即使 `odom` 连续，原先转换出的全局参考路径也可能已不正确；旧计算结果可能在新定位生效后才发布。

**建议方案：** 检测已应用变换的修正，重新转换参考路径。大幅修正时停车、废弃旧轨迹并重新规划；小幅修正的处理阈值待实测确定。SCAN 不重复发布 `map → odom` 或 `odom → base_link`，KISS 导航节点与旧配准演示节点也不能同时维护同一核心 TF。

**验证方法：** 运动中施加平移和旋转校正，检查旧轨迹失效、版本隔离、停车和重新规划的完整过程。

依据：[KISS 候选确认接口](/home/jinghao/kiss_localization_ws/src/localization/srv/AcceptCandidate.srv)、[KISS TF 与失效约定](/home/jinghao/kiss_localization_ws/src/localization/NAVIGATION_INTERFACE.md)。

### INT-10 轨迹超时与停车不能依赖规划循环及时返回 · P0

**当前事实：** SCAN 规划节点使用单线程执行器，规划和回调共享执行线程；闭环控制节点独立运行，但现有控制循环没有完整的输入时效与执行许可检查。

**潜在影响：** 规划失败、卡住或输入失效时，控制器可能继续跟踪此前接收的轨迹。仅增加规划状态日志不足以保证控制输出停止。

**建议方案：** 保持控制与耗时规划分离，由唯一的速度发布节点独立检查里程计时效、轨迹有效期、规划授权及任务版本。无安全可执行轨迹时持续发布零速度；底盘还需提供命令中断超时保护，以覆盖 SCAN 进程退出的情况。

**验证方法：** 人为延迟规划、终止规划进程、停止输入，分别检查速度输出和底盘超时停车。允许继续执行旧轨迹的前提必须是轨迹仍有效且已通过必要安全检查。

依据：[SCAN 执行器](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_planner_node.cpp)、[闭环控制循环](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/closed_loop_controller.cpp:120)。

### INT-11 Topic 与 QoS 需要同时匹配 · P1

**当前事实：** PCT 路径为 Reliable + Transient Local，深度 1；SCAN 当前参考路径订阅使用默认易失性设置，晚启动时无法依靠该订阅获取此前保留的路径。Fast-LIO 窗口输出为 Reliable，KISS 默认输入请求 Best Effort；两者并不矛盾，不能把订阅侧设置误写成发布侧属性。

**潜在影响：** 启动顺序变化导致收不到已有路径，或配置不兼容导致完全无法通信。反过来，持久化路径也可能是旧任务，不能收到后立即无条件执行。

**建议方案：** 在 SCAN 集成配置中明确路径 QoS 与迟启动行为；收到保留路径后仍检查当前状态、路径合法性和启用条件。里程计与点云 QoS 分别记录并验证，所有话题支持重映射。

**验证方法：** 先发路径再启动 SCAN，重启 SCAN，发布空路径后重启；同时检查 `ros2 topic info --verbose` 中实际端点配置。

依据：[PCT QoS](/home/jinghao/PCT_planner/src/pct_planner/pct_planner/ros/planner_node.py:73)、[SCAN 路径订阅](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_replan_fsm.cpp:89)、[窗口端点 QoS](/home/jinghao/Fast_lio_ws/src/mid360_bringup/mid360_bringup/local_map_window.py:73)。

### INT-12 三维路径不等于已经具备楼梯闭环运动能力 · P1

**当前事实：** SCAN 搜索使用由起终点高度确定的投影高度，局部初始化又按 XY 弧长插值 z，优化梯度的 z 行置零。闭环控制只输出平面速度与转向；旧简化运动学仿真保持 z 不变，现已归档至 `legacy/demo`。

**潜在影响：** PCT 的楼梯、平台、上下层路径可能被局部高度参考简化；XY 接近的不同楼层可能选错进度。开环三维回放成功也不能证明底盘能沿楼梯运动。

**建议方案：** 按三维路径进度选择局部段，保留楼层和高度变化，验证现有投影机制的适用范围。将超出能力的路径明确拒绝或报告失败；爬楼、地形跟随和机身姿态控制能力需由实际运动控制模块验证，不能靠给 Twist 增加垂直速度替代。

**验证方法：** 覆盖平地、坡道、楼梯转平台、上下层 XY 重叠和局部遮挡；分别验收轨迹几何、碰撞检查与底盘执行能力。

依据：[投影搜索](/home/jinghao/SCAN-Planner/legacy/planner/path_searching/src/dyn_a_star.cpp:146)、[局部高度初始化](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/planner_manager.cpp:9)、[优化梯度约束](/home/jinghao/SCAN-Planner/legacy/planner/bspline_opt/src/bspline_optimizer.cpp:1169)、[平面控制](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/closed_loop_controller.cpp:147)。

### INT-13 轨迹结束与真实到达终点需要区分 · P1

**当前事实：** SCAN 状态机存在根据轨迹时长结束任务的逻辑；控制器结束误差只使用 XY。当前目标被占据时，还会沿参考轨迹回退终点。

**潜在影响：** 机器人尚未到达、处于错误楼层或只到达被缩短的目标，就被上层误认为任务完成。

**建议方案：** 保留原始任务终点与局部替代子目标的区别，按实际机器人三维位置、速度及必要的朝向条件判定 `GOAL_REACHED`。不可达原终点应报告失败或等待，不悄悄改变任务成功条件。

**验证方法：** 人为限制底盘运动，使轨迹计时结束但机器人未到；测试相同 XY 不同楼层、目标被障碍占据等情况。

依据：[状态机结束判定](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_replan_fsm.cpp:635)、[目标回退逻辑](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/scan_replan_fsm.cpp:297)、[控制器终点误差](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/src/closed_loop_controller.cpp:148)。

### INT-14 全局地图文件不同，坐标基准未必一致 · P1

**当前事实：** PCT 默认地图配置为 `building2_9.pcd`；KISS launch 默认地图为 `3_part.pcd`，其本机接入说明还使用 `scans.pcd`。这些是默认配置或说明示例，不能据此判断现场正在使用哪份地图。

**潜在影响：** 两个模块均标记 `map`，但地图原点、朝向、比例或楼层基准不同，仍会导致全局路径与定位错位。

**建议方案：** 联调时记录实际地图绝对路径、版本/校验值和坐标基准，确认来自同一全局坐标。不同裁剪文件可以共用坐标，但必须验证；未来统一地图服务属于另一个模块的任务，不要求先改造它才能接入 SCAN。

**验证方法：** 在同一 `map` 中叠加两份地图、定位点云和 PCT 路径，检查多个已知位置及楼层高度的一致性。

依据：[PCT 地图配置](/home/jinghao/PCT_planner/src/pct_planner/config/pct_planner.yaml:3)、[KISS 启动默认值](/home/jinghao/kiss_localization_ws/src/localization/launch/navigation_localization.launch.yaml:3)、[KISS 本机接入说明](/home/jinghao/kiss_localization_ws/README.md)。

### INT-15 栅格分辨率、点云密度与实时性需联合验证 · P1

**当前事实：** Fast-LIO 滑动窗口默认以 0.15 m 体素抽稀并以 2 Hz 发布，SCAN 默认栅格为 0.05 m、地图约 10 × 10 × 5 m。窗口按时间维护，不保证以机器人为中心的空间覆盖。

**潜在影响：** 更细栅格不会恢复上游已丢失的点云细节；薄障碍、视野缺失、历史障碍和计算耗时可能影响局部规划与停车距离。

**建议方案：** 联合配置点云密度、地图范围、未知空间策略、机器人膨胀尺寸和速度上限；以测得的输入延迟、规划耗时及控制响应确定可运行速度。动态避障能力不能仅依据静态点云场景宣称完成。

**验证方法：** 使用细杆、桌腿、窄通道和移动障碍，记录输入年龄、规划耗时、CPU/内存与停止距离。

依据：[窗口默认参数](/home/jinghao/Fast_lio_ws/src/mid360_bringup/launch/mid360_mapping.launch.py:105)、[SCAN 地图参数](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/config/planner.yaml:13)。

### INT-16 ROS 包独立性、内部轨迹协议和交付配置仍需整理 · P2

**当前事实：** SCAN 的算法层仍使用 ROS 参数、日志和时间；主包声明了 Go2、感知模拟、地图生成及 RViz 等运行依赖。当前启动中还使用 `/LIO/...`、`/quad_0/...` 等演示/旧接入话题。内部 `Bspline` 消息没有 frame_id。

**潜在影响：** 集成部署被迫携带仿真依赖，核心难以独立测试；不同环境下轨迹坐标含义不明确。工作空间叠加错误还可能实际启动另一份已安装版本。

**建议方案：** 拆分 core、ROS 适配和可选 demo；独立启动只包含 SCAN 规划与控制。内部轨迹明确 frame、起始时间、有效期和任务版本，外部发布标准 Path、Twist 及状态。记录 Humble 构建环境、实际 `ros2 pkg prefix`、参数、话题和 TF 发布责任。

**验证方法：** 不安装 Go2 仿真、不引入其他算法源码时单独构建和启动 SCAN；使用模拟输入验收；在干净终端检查 overlay 加载顺序及包路径。

依据：[SCAN 包依赖](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/package.xml)、[SCAN 启动入口](/home/jinghao/SCAN-Planner/legacy/planner/plan_manage/launch/run.launch.py)、[内部样条消息](/home/jinghao/SCAN-Planner/legacy/planner/scan_planner_msgs/msg/Bspline.msg)。

### INT-17 PCD 点云与 Gazebo 碰撞网格存在量化和缺失面风险 · P1

**当前事实：** 新测试包从同一份 PCD 裁剪 SCAN 局部点云，并将指定区域的已有点体素化成 Gazebo 静态 OBJ/SDF。转换器不补地面；体素尺寸改变实际接触表面的位置和形状。此链路已经过转换测试，但未在 Gazebo 中完成场景验收。

**潜在影响：** 稀疏地面会留下碰撞孔洞，台阶边缘可能与原 PCD 相差一个体素，Go2 会跌落、卡住或踩在与 SCAN 地图不一致的位置。

**建议方案：** 每份地图显式记录转换边界、体素分辨率和机器人出生位置；在 Gazebo/RViz 叠加检查地面、台阶和障碍。缺失的地面需要地图数据修复，不能让转换器悄悄推断为可通行。

**验证方法：** 对坡道、台阶、孔洞及上下层交叠区域比较 PCD 与碰撞表面，并运行站立与低速接触测试。

依据：[PCD 转换工具](../src/scan_planner_demo/scripts/pcd_to_sdf.py)、[局部点云节点](../src/scan_planner_demo/src/pcd_local_map_node.cpp)。

### INT-18 仿真三维里程计不等于已具备稳定步态 · P1

**当前事实：** Go2 模型新增 Gazebo 3D 里程计插件，明确要求 `dimensions=3`；测试包新增 `cmd_vel` 到 12 个关节目标的初步足端轨迹适配。Gazebo 运行依赖尚未安装，本机未完成物理运动验证。

**潜在影响：** 即使 `/scan_demo/odom` 和 `odom→base` 有高度，关节控制仍可能无法稳定前进、爬坡或跨台阶。重复发布 TF 或把仿真真值误作真实定位精度也会误导集成判断。

**建议方案：** 由 Gazebo 里程计桥接独占 `odom→base`，`robot_state_publisher` 负责关节树，`map→odom` 在独立演示中按同源地图发布固定变换。按站立、水平行走、坡道、楼梯逐级验收；失败时调整或替换步态控制，不让 SCAN 路径直接改写模型位姿。

**验证方法：** 核查 TF 树唯一性、里程计实际 z/速度、四足接触、机身姿态、实际轨迹与 SCAN 轨迹误差及停车行为。

依据：[Go2 模型插件](../src/scan_planner_demo/description/xacro/gazebo.xacro)、[测试步态适配器](../src/scan_planner_demo/src/go2_cmd_vel_gait.cpp)。

## 建议的验证顺序

1. 冻结参考点与坐标约定：确认 IMU/机身/控制原点外参，以及 PCT 路径高度语义。
2. 明确速度来源：验证零速度字段问题，确定 SCAN 差分方案或另行修改上游。
3. 固定部署接口：采用实际话题和 QoS，核对启动入口、包路径及 TF 唯一发布者。
4. 用模拟数据或 rosbag 验证坐标转换：使用非单位 TF，并检查窗口快照更新与采集时间。
5. 验证运动许可与停车：覆盖空路径、数据中断、定位修正、计算超时及旧结果迟到。
6. 完成平地与静态障碍闭环，再验证多层路径、坡道、楼梯和动态障碍能力边界。

## 后续记录约定

每项问题处理后记录所选方案、修改涉及的工作空间、验证数据、结果和剩余限制。没有执行过的验证应标记为“未验证”；其他模块的修改需要单独讨论，不能因为本文提出建议而视为已经决定或完成。
