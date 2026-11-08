# 本机演示地图

- [pcd/](pcd/README.md)：原始 PCD 点云，供 SCAN 裁剪局部地图。
- [gazebo/](gazebo/README.md)：从同一份 PCD 指定区域转换的 SDF 场景、OBJ
  碰撞网格与 JSON 来源校验信息。

两类地图保留在功能包源码目录，均不随 Git 或 `colcon` 安装。转换产物记录
本机绝对路径；移动 PCD 或 OBJ 后，应重新运行转换命令生成 SDF 和 JSON。
示例参数及运行方法见 [RUNNING.md](../../../RUNNING.md)。
