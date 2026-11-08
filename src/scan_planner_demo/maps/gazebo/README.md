# Gazebo 碰撞场景

`building_flat.sdf/.obj/.json` 对应 `../pcd/building2_9.pcd` 的已验证
地面区域；`flat_patch.sdf/.obj/.json` 对应 `../pcd/map.pcd` 的平地区域。
SDF 定义静态场景，OBJ 是碰撞网格，JSON 记录源 PCD、网格、裁剪边界与
体素分辨率。三种文件都由 `scripts/pcd_to_sdf.py` 生成，并被 Git 忽略。

SDF 和 JSON 包含本机绝对路径。移动地图目录、清理或更换 PCD 后，按
[运行说明](../../../../RUNNING.md)重新生成；启动入口会拒绝不匹配的文件。
转换器只体素化已有点，不推断或填补缺失地面。
