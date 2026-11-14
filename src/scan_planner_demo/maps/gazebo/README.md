# Gazebo 碰撞场景

`building2_9_full.sdf/.obj/.json` 对应 `../pcd/building2_9.pcd` 的全部有效
点云，以 0.20 m 体素生成完整碰撞场景。`building_flat.sdf/.obj/.json`
是同一 PCD 的局部地面验证场景；`flat_patch.sdf/.obj/.json` 对应
`../pcd/map.pcd` 的平地区域。
`building2_9_z075_xy15_z05.sdf/.obj/.json` 对应高度压缩后的
`../pcd/building2_9_z075.pcd`，完整覆盖全部点云，体素尺寸为
XY 0.15 m、Z 0.05 m。楼梯截面量测见 [运行说明](../../../../RUNNING.md)。
SDF 定义静态场景，OBJ 是碰撞网格，JSON 记录源 PCD、网格、裁剪边界与
体素分辨率。这些文件均由 `scripts/pcd_to_sdf.py` 生成，并被 Git 忽略。
整图转换使用 `--all`，局部区域转换使用 `--bounds`；两者不能同时使用。

SDF 和 JSON 包含本机绝对路径。移动地图目录、清理或更换 PCD 后，按
[运行说明](../../../../RUNNING.md)重新生成；启动入口会拒绝不匹配的文件。
转换器只体素化已有点，不推断或填补缺失地面。
