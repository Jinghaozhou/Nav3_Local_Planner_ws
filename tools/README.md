# 路径点记录工具

构建 scan_planner_demo 并加载新的安装环境后运行：

```bash
source install/integration/setup.bash
ros2 run scan_planner_demo keypoint_recorder.py --odom /fast_lio/odom --output keypoints.yaml
```

记录器保留旧版输出格式：ROS 参数 YAML 中的 `fsm.waypoints: [x0, y0, z0, ...]`。该文件不能直接作为新集成节点的启动参数；需要转换为上游 nav_msgs/Path，或演示节点的 waypoints 参数。记录坐标必须与目标全局 frame 一致，位姿来自 odom 时还需使用相应时刻的 map→odom 转换。

当前集成入口和工具用法见根目录 [RUNNING.md](../RUNNING.md)。旧版 Mode 2 启动代码仅保留在 legacy 对照目录，不再参与默认构建。
