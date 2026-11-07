# SCAN ROS 2 接口

适用环境为 ROS 2 Humble。`planner_node` 与 `controller_node` 分别独立运行；模块不启动 Fast-LIO、KISS、PCT、雷达驱动或机器人模型，不发布核心 TF 链。

## 输入

| 默认系统话题 | 类型 | 坐标与使用约定 | 订阅 QoS |
|---|---|---|---|
| `/fast_lio/odom` | `nav_msgs/msg/Odometry` | pose 属于 header.frame_id，twist 属于 child_frame_id；默认用位姿差分估速 | SensorData，Best Effort、Volatile |
| `/fast_lio/local_map` | `sensor_msgs/msg/PointCloud2` | 默认 odom；每条为完整窗口快照，不是增量；FLOAT32 XYZ，支持带行填充的有组织点云 | SensorData，Best Effort、Volatile |
| `/pct/global_path` | `nav_msgs/msg/Path` | header.frame_id 必须为 global_frame；pose frame 可为空或与之相同 | Reliable、Transient Local、深度 1 |
| TF | `map → odom → robot_frame` | 全局定位与连续定位由外部唯一发布者维护 | TF2 |
| 可选 `/scan_planner/localization_permission` | `std_msgs/msg/Bool` | 外部质量适配器发出的运行许可；启用检查后必须持续为 true，接收间隔不超过 1 秒 | Reliable、Volatile |

传感器订阅的 Best Effort 与当前上游 Reliable 发布者兼容。点云只提供占据证据，不提供自由空间证据。默认拒绝大端序或无合法 FLOAT32 XYZ 字段的消息，并报告原因。

`robot_frame` 是碰撞与跟踪参考点。若与里程计 child_frame_id 不同，必须存在二者间的正确 TF。`control_frame` 是 Twist 对应的底盘参考点；控制器完成安装旋转及平移参考点的速度换算，最终输出平面 vx/vy/wz。

## 输出

| 默认系统话题 | 类型 | 坐标与行为 | QoS |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | control_frame 下的平面速度；控制节点是本模块唯一发布者；100 Hz | Reliable、Volatile、深度 1 |
| `/scan_planner/status` | `scan_planner_msgs/msg/PlannerStatus` | 20 Hz 心跳；goal 为原任务终点，使用 global_frame | Reliable、Transient Local、深度 1 |
| `/scan_planner/local_path` | `nav_msgs/msg/Path` | planning_frame 下的局部轨迹采样；失效时发布空路径 | Reliable、Transient Local、深度 1 |

命名空间内的状态与轨迹话题是相对话题。独立 launch 将外部输入与速度话题映射到上述默认系统名称，可通过 launch 参数改名。多机器人部署须同时修改这些外部绝对话题，避免不同实例发布到同一 `/cmd_vel`。

## 状态消息

`PlannerStatus` 包含 header、path_id、traj_id、state、motion_allowed、reason、goal。状态码：

| 状态 | 含义 |
|---|---|
| WAITING_INPUT | 等待有效传感器数据或速度估计初始化 |
| WAITING_PATH | 尚未接收参考路径 |
| RUNNING | 已有通过验证且未过期的局部轨迹 |
| GOAL_REACHED | 实测三维位置、速度与原终点满足阈值；保持完成状态至新路径到来 |
| LOCAL_PLAN_FAILED | 局部搜索、优化、最终检查失败或轨迹有效期耗尽 |
| GLOBAL_PATH_INVALID | 空路径、frame 错误、非有限坐标等；撤销旧任务 |
| LOCALIZATION_UNAVAILABLE | TF 不可用、状态与 TF 不一致或外部定位许可无效 |
| DATA_STALE | 输入采集时间过期或消息无效 |
| TF_CHANGED | 全局定位发生较大修正；旧局部轨迹作废 |

RUNNING 表示规划侧许可，控制器仍独立检查里程计、心跳、轨迹编号、开始时间和有效期，因此不能单靠该状态断言底盘正在运动。

## 模块内部协议

- `scan_planner/trajectory`：扩展后的 `Bspline`，带 header.frame_id、path_id、traj_id、start_time、valid_until。控制点和结点必须有限，结点严格递增，阶数为 3。控制器拒绝坐标不符、维度不符、过期和倒序的轨迹。
- `scan_planner/execution_state`：`ExecutionState`，提供任务/轨迹编号、实际跟踪计时、转向暂停和控制侧拒绝原因，用于监控。
- 控制器只执行与最新状态中 path_id 和 traj_id 同时一致的轨迹。轨迹许可不能仅因重复收到状态心跳而延长。
- 转向误差较大时暂停轨迹计时，但不延长绝对 valid_until。规划器依据实测状态生成后续轨迹。

## 路径高度与撤销

`path_height_mode=reference` 时 `z_out=z_in+path_z_offset`；`ground` 时再加 `body_height`。当前 PCT 已有高度偏移，默认 reference、path_z_offset=0，不隐式再加 0.4 m。需要依据 IMU/机身安装外参校验具体数值。

空 Path 表示撤销当前任务：作废旧轨迹、发布空局部路径并停车。Path 的历史发布时间本身不是超时条件，因为静态全局路线可以长时间有效；调用方应通过空路径或替换路径管理任务。收到 Transient Local 保留路径后仍须通过全部输入和轨迹验证。

## 时间与失效

同时检查 ROS 采集时间与本地接收间隔。地图重复发布相同采集时间不能刷新有效期。数据超时、时钟回跳、ROS 时钟停滞、规划心跳中断、轨迹过期或没有执行许可时输出零速度。底盘驱动必须配置自身的命令超时停车，以覆盖控制进程被终止或主机掉电的情况。

TF 的新鲜度不等于全局定位质量。KISS 会重新广播上一有效变换；需要质量门控时，应由独立适配器根据其 last_success_stamp、applied 等输出标准 Bool，并开启 require_localization_permission。SCAN 核心不依赖 KISS 的消息或算法包。
