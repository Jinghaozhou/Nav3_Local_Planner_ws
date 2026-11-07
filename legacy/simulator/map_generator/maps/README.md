# 本地点云地图

此目录集中存放 map_generator 发布器使用的本地 PCD 地图。地图已从工作空间根目录迁入，文件内容保持不变。

| 文件 | 大小约值 |
|---|---|
| `map.pcd` | 48.0 MB |
| `5.pcd` | 48.0 MB |
| `5_dense.pcd` | 430.3 MB |
| `building2_9.pcd` | 13.7 MB |
| `scans.pcd` | 1.11 GB |

这些是本机数据文件，继续由根目录 `.gitignore` 中的 `*.pcd` 排除，不随 Git 分发，也不在构建时复制到 install。移动或部署工作空间时，需要单独提供地图，并使用实际的绝对路径。

在工作空间根目录加载构建环境后，可将地图传给演示入口：

```bash
ros2 launch scan_planner_demo simulation.launch.py \
  use_pcd_map:=true \
  pcd_map_file:="$PWD/src/simulator/map_generator/maps/5.pcd" \
  scenario_config:=/absolute/path/to/scenario.yaml
```

`scenario.yaml` 中的起终点和高度必须与所选地图对应。仅将地图放在本目录，不会自动改变演示的默认起点和路线。

只发布地图时，可直接运行 map_generator；frame_id 应与地图实际坐标基准一致：

```bash
ros2 run map_generator map_pub --ros-args \
  -p file_name:="$PWD/src/simulator/map_generator/maps/building2_9.pcd" \
  -p frame_id:=map
```

`map_pub` 将完整地图发布到相对话题 `global_cloud`，使用 Reliable + Transient Local。它提供离线 PCD 发布，不负责生成程序化随机地图或维护真实传感器滑动窗口。
# 归档说明

此目录不再存放 PCD。五份本机地图已迁至
`src/scan_planner_demo/maps/`；下面保留的是旧测试包的历史说明。

