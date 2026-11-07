# 本机 PCD 地图

这里存放从工作空间原地图目录迁入的五份 PCD：`map.pcd`、`5.pcd`、
`5_dense.pcd`、`building2_9.pcd`、`scans.pcd`。它们继续被根目录的
`*.pcd` 规则排除，不进入 Git，也不随 `colcon` 安装。

`simulation.launch.py` 的 `pcd_map_file` 应指向其中一份实际存在的地图；
`scripts/pcd_to_sdf.py` 从同一份 PCD 的指定区域生成 Gazebo 碰撞场景。
它还生成同名 JSON 元数据，启动入口据此拒绝与 PCD 不匹配的场景。
转换器只保留点所在的占据体素，不填补缺失地面。使用前在 Gazebo 中检查
地面连续性、台阶形状、障碍尺寸和起点位置。路径点与出生位置必须与 PCD
坐标一致。
