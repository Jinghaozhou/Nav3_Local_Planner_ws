# 重构前源码对照

`planner/` 保留本地初始提交 `11224ff` 中的旧规划器、控制器、配置和演示入口，用于对照算法迁移与既有集成问题。此目录通过 `COLCON_IGNORE` 排除，不能作为当前集成启动入口。

当前可构建实现位于 `src/scan_planner`、`src/scan_planner_msgs`、`src/scan_planner_demo`。旧平面演示位于 `demo/`；原有传感器、地图生成和 Go2 辅助包位于 `simulator/`。当前测试入口是 `ros2 launch scan_planner_demo simulation.launch.py`，需要 Gazebo、PCD 与预先转换的 SDF 场景。

原项目署名与许可证见根目录 `NOTICE`、`LICENSE`。保留此目录不表示旧版本已经通过当前接口验收。
