# CHAMP gait core

The header-only gait core in `include/champ` is copied from the ROS 2 branch of
[CHAMP](https://github.com/chvmp/champ/tree/ros2) (BSD-3-Clause, copyright
Juan Miguel Jimeno, 2019–2020). The Go2 ROS 2 integration pattern was checked
against [unitree-go2-ros2](https://github.com/rahgirrafi/unitree-go2-ros2),
`ignition` branch, commit `dfbc039`. The upstream CHAMP `LICENSE` and source
copyright notices are retained. This directory does not contain the ROS node
from the upstream project; `src/go2_champ_gait.cpp` is this project's adapter.
