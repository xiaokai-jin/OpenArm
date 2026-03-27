# demo.launch.py 代码详细解读

本文档对 `demo.launch.py` 进行逐段解析，涵盖了代码功能、Python 语法以及对应的 ROS 2 Launch API 的作用。

## 1. 模块导入部分

```python
import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
```

**作用语法与API解析**：
* `os` / `xacro`: Python 标准库与外部库，用于路径拼接和渲染 Xacro (宏定义的 URDF) 文件。
* `ament_index_python.packages.get_package_share_directory`: ROS 2 API，用于获取已安装的 ROS 2 package 的 `share` 目录的绝对路径。
* `launch.*`: 核心 Launch API。
  * `LaunchDescription`: Launch 文件的核心返回对象，包含所有要执行的 action。
  * `LaunchContext`: Launch 运行时的上下文，包含参数替换等运行时状态。
  * `DeclareLaunchArgument`: 声明可通过命令行传入的参数（如 `ros2 launch xxx.launch.py arg:=value`）。
  * `TimerAction`: 用于延迟执行某些 action。
  * `OpaqueFunction`: 允许在 Launch 期间执行普通的 Python 函数。这在需要在运行时评估参数以生成具体逻辑时非常有用。
  * `LaunchConfiguration`: 获取通过 `DeclareLaunchArgument` 声明的参数值。
  * `PathJoinSubstitution`: 环境安全的路径拼接 Substitution（替代物）。
* `launch_ros.actions.Node`: 用于在 Launch 中启动 ROS 2 节点的 Action。
* `launch_ros.substitutions.FindPackageShare`: 在 Launch 上下文安全的查找包路径的方法。
* `MoveItConfigsBuilder`: MoveIt2 提供的工具类，用于快速构建和加载 MoveIt 配置参数（代替极其繁琐的手动加载 YAML）。


## 2. 工具函数：generate_robot_description

```python
def generate_robot_description(
    context: LaunchContext,
    description_package,
    description_file,
    ...
):
    """Render Xacro and return XML string."""
    description_package_str = context.perform_substitution(description_package)
    # ... 变量类型从 LaunchConfiguration 转换为 Python 字符串
```
**作用语法与API解析**：
* 这是一个普通 Python 函数，它会被 `OpaqueFunction` 调用。
* `context.perform_substitution()`: 将 Launch 体系中的 `Substitution` (替代物，比如获取到的 `LaunchConfiguration` 参数对象) 解析为真实的 Python 字符串。必须依托运行时 `context`。

```python
    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf",
        "robot",
        description_file_str,
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            ...
        },
    ).toprettyxml(indent="  ")

    return robot_description
```
**作用语法与API解析**：
* 使用 `get_package_share_directory` 直接获取包路径，结合 `os.path.join` 拼接出完整的 `.xacro` 文件路径。
* `xacro.process_file()`: 核心函数。使用提供的参数字典(`mappings`)替换 xacro 文件中的参数，然后渲染生成最终的 URDF (XML 格式字符串)。这是将模块化的 xacro 转换为 ROS 系统可以直接识别的机器人描述的核心步骤。


## 3. 工具函数：robot_nodes_spawner

```python
def robot_nodes_spawner(
    context: LaunchContext,
    ...
):
    # 调用刚刚的函数获取 xml 文本
    robot_description = generate_robot_description(...)

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]
```
**作用语法与API解析**：
* 此函数用于生成和返回需要启动的两个基础节点：`robot_state_publisher` 和 `ros2_control_node`。
* `Node(...) API`: 
  * `package` 和 `executable` 指定要运行的ROS 2程序。
  * `parameters`: 向节点传递参数，这里将解析出的完整 XML 文本作为 `robot_description` 参数传递给节点，同时传递了 controller 的 YAML 配置文件路径。
* 这个函数最终返回一个包含 Node 对象的列表，由 `OpaqueFunction` 送入 Launch 系统执行。


## 4. 工具函数：controller_spawner

```python
def controller_spawner(context: LaunchContext, robot_controller):
    # 判断控制器的类型，生成对应的左/右臂控制器名称
    robot_controller_str = context.perform_substitution(robot_controller)
    if robot_controller_str == "forward_position_controller":
        left = "left_forward_position_controller"
        ...

    return [
        # 左侧控制器 → 左侧 controller_manager
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[left, "-c", "/left/controller_manager"],
            name="spawner_left_" + left,
        ),
        # 右侧控制器 → 右侧 controller_manager
        Node(...)
    ]
```
**作用语法与API解析**：
* 该函数的作用是启动 ROS 2 Control 的加载器（spawner）。
* `arguments=[...]`: 在执行程序时附加命令行参数。这里的逻辑是将控制器绑定到特定的命名空间 controller_manager 下（如 `/left/controller_manager`），这就是所谓的双 Manager 架构的具体体现。


## 5. 主入口函数：generate_launch_description

```python
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package", default_value="openarm_description"),
        ...
    ]
```
**作用语法与API解析**：
* `generate_launch_description` 是 ROS 2 launch 框架约定的必须存在的入口函数。
* `DeclareLaunchArgument`: 声明了一堆启动命令时可接收的参数（如 `arm_type:=v10`）。

```python
    description_package = LaunchConfiguration("description_package")
    ...
    # 路径拼接 Substitution
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "v10_controllers", controllers_file]
    )
```
**作用语法与API解析**：
* `LaunchConfiguration`：在 Launch 构建阶段拿到对之前声明的 Argument 的引用。
* `PathJoinSubstitution` / `FindPackageShare`：用于在 Launch 描述构建期间进行懒加载的路径拼接（而非前面 Python 的直接拼接，因为此时无法获取环境变量）。

```python
    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[...]
    )
```
**作用语法与API解析**：
* `OpaqueFunction`：极其核心！因为常规の Launch API 是声明式的（构建图但不立即执行），但是我们需要用到 `xacro` 渲染这一具体动作，所以要把它包在 `OpaqueFunction` 里。当 Launch Engine 运行到此处时，会调用 Python 传入的 `function` 并把 `LaunchContext` 注入进去。

```python
    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])
    delayed_arm_ctrl = TimerAction(period=1.0, actions=[controller_spawner_func])
    ...
```
**作用语法与API解析**：
* `TimerAction`：ROS 2 中常用的规避节点依赖顺序问题的暴力做法。这里延时 `1.0` 和 `2.0` 秒启动控制器，是为了等待 `ros2_control_node` (前面定义的控制管理节点) 完全启动并就绪。

```python
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    moveit_params = moveit_config.to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )
```
**作用语法与API解析**：
* `MoveItConfigsBuilder`：自动扫描 `openarm_bimanual_moveit_config` 包内的 SRDF、kinematics.yaml、joint_limits.yaml 等 MoveIt 所需的大量配置文件，并将其打包成参数字典 `to_dict()`。
* 然后启动 `move_group` 这个 MoveIt 运动规划的核心服务端节点。

```python
    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_spawner_func,
            delayed_jsb,
            ...
            run_move_group_node,
            rviz_node,
        ]
    )
```
**作用语法与API解析**：
* 汇总一切！把所有的参数声明（`declared_arguments`）、包装好的函数、节点（Node）都塞进 `LaunchDescription` 列表内返回。ROS 2 Launch 引擎将接管并并发拉起这个列表中的所有对象。

---

## 6. 新增：重力补偿前馈接入说明

当前 `demo.launch.py` 已接入重力补偿前馈链路：

1. 声明了 `use_gravity_compensation` 开关参数（默认 true）
2. 声明了 `gravity_compensation_params_file` 参数文件路径
3. 启动 `left_gravity_compensation_controller` 与 `right_gravity_compensation_controller`
4. 启动 `gravity_compensation_node`，发布左右臂 effort 前馈命令

### 运行时验证建议

```bash
ros2 node list | grep gravity_compensation_node
ros2 control list_controllers | grep gravity_compensation
ros2 topic hz /left_gravity_compensation_controller/commands
ros2 topic hz /right_gravity_compensation_controller/commands
```

### 常见问题

- 如果控制器已 active 但 commands 没有频率，优先检查重力节点是否崩溃。
- 如果点击执行时先下坠，通常是前馈比例偏低或执行阶段前馈中断。
- 如果手动拖动时“很硬”，通常是轨迹控制器 + 高 kp/kd 在保持位姿，不是重力补偿本身异常。
