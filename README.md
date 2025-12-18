# python_path_searching 包使用说明

本说明文档介绍如何在 `ros2_ws` 工作空间中**只编译这个包**，然后 source 环境并启动本包中的 Python launch 文件。

## 1. 环境准备

1. 已正确安装 ROS 2（例如 Humble、Iron 等）。
2. 已创建工作空间，并将本包放在：

```bash
cd ~/ros2_ws
# 只编译 python_path_searching 包
colcon build --packages-select python_path_searching
source install/setup.bash
ros2 launch python_path_searching path_tracking.launch.py
```