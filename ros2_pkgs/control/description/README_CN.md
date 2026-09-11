# isaacsim_description

这个包不是资产仓库，也不保存生成后的 URDF/USD 文件。它只保存 Isaac Sim 专用的
URDF 生成入口，用来从上游权威 OpenFleX xacro 生成当前仿真运行需要的机器人描述。

## 目录

- `scripts/generate_isaac_urdf.py`：生成 Isaac 专用 URDF。它会移除上游 xacro 里的
  MuJoCo/Gazebo/旧 ros2_control 片段，补充 topic_based_ros2_control、Isaac sensor
  标记、基础物理材质、轮胎简化碰撞体、缺失惯量和 mesh 路径处理。
- `urdf/README.md`：说明本包不提交生成物；生成结果应放到运行产物目录或 install
  workspace，而不是写回源码目录。
- `package.xml` / `CMakeLists.txt`：ROS 2 ament 包元数据和安装规则。

## 5.1 主线边界

Isaac Sim 5.1 运行时由 `isaacsim_bringup` 启动；本包只负责生成可导入的
URDF 输入。ROS 2 节点和 Isaac Sim 进程通过 DDS / Isaac ROS2 bridge 通信，不在本包里
绑定 conda、uv 或 OmniGibson 环境。
