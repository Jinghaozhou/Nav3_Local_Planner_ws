# 原始 PCD

本机地图：`map.pcd`、`5.pcd`、`5_dense.pcd`、`building2_9.pcd`、
`scans.pcd`。PCD 文件体积较大，继续由根目录 `.gitignore` 排除，不随
Git 或 ROS 功能包安装。测试时通过 `pcd_map_file` 指定实际文件路径。

`building2_9_z075.pcd` 是 `building2_9.pcd` 的独立实验副本：保留全部
853,227 个有效点的 XY 坐标，以 `z=0` 为基准将高度乘以 `0.75`。
它也已复制到 `/home/jinghao/Nav3_Global_planner_ws/map/building2_9_z075.pcd`，
供 PCT 在同一几何坐标下生成全局路径。不要将原地图的 PCT 缓存或路径
用于这个缩高场景。
