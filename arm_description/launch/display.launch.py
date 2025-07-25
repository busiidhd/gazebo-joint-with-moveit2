from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# 示例：ros2 launch arm_description display.launch.py model:=`ros2 pkg prefix --share arm_description`/urdf/urdf/demo01_helloworld.urdf
# `ros2 pkg prefix --share arm_description`是一个指令 等价于 get_package_share_directory("arm_description")
def generate_launch_description():

    arm_description_dir = get_package_share_directory("arm_description")
    default_model_path = arm_description_dir + '/urdf/arm.urdf'
    default_rviz_path = arm_description_dir + '/rviz/urdf.rviz'
    model = DeclareLaunchArgument(name="model", default_value=default_model_path)

    # 加载机器人模型
    # 1.启动 robot_state_publisher 节点并以参数方式加载 urdf 文件
    robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]))
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # 2.启动 joint_state_publisher 节点发布非固定关节状态
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # rviz2 节点
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz_path]
    )

    return LaunchDescription([
        model,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])