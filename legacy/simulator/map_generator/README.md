# 仿真 PCD 地图发布包

`map_generator` 提供 `map_pub` 节点，从 `file_name` 指定的绝对路径读取 PCD，完成可选体素降采样与坐标平移后发布 `sensor_msgs/msg/PointCloud2`。

```text
map_generator/
├── maps/                # 原工作空间根目录的本地 PCD 地图
├── src/map_publisher.cpp
├── CMakeLists.txt
└── package.xml
```

地图清单和使用命令见 [maps/README.md](maps/README.md)。大型 PCD 不纳入 Git，也不随构建复制到安装目录；通过参数指定实际路径即可加载。

| 参数 | 默认值 | 用途 |
|---|---|---|
| `file_name` | 空，必须设置 | PCD 文件路径 |
| `frame_id` | `world` | 输出点云的实际坐标基准 |
| `publish_rate` | `0.2` | 发布频率，Hz |
| `downsample_res` | `0.0` | 体素降采样尺寸，米；零表示不降采样 |
| `map_offset_x/y/z` | `0.0` | 对地图点坐标施加平移，米 |

输出为相对话题 `global_cloud`，使用 Reliable + Transient Local。`scan_planner_demo` 的 PCD 模式会启动本节点，并将输出映射到 `/scan_demo/map_source`。
