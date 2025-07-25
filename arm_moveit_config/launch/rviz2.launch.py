from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 初始化MoveIt配置
    moveit_config = MoveItConfigsBuilder("arm", package_name="arm_moveit_config").to_moveit_configs()

    ld = LaunchDescription()

    # 声明通用参数
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=True))

    # 从话题获取机器人描述
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    ld.add_action(robot_description_node)

    # 启动move_group节点
    my_generate_move_group_launch(ld, moveit_config)

    # 启动RViz
    my_generate_moveit_rviz_launch(ld, moveit_config)

    return ld


def my_generate_move_group_launch(ld, moveit_config):
    # 声明move_group专用参数
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # 加载MoveIt简单控制器配置，这里应该加载moveit_controllers.yaml
    moveit_controllers_path = os.path.join(
        moveit_config.package_path,
        "config",
        "moveit_controllers.yaml"
    )

    # 配置move_group参数
    move_group_configuration = {
        # 控制器配置
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": ParameterValue(
            moveit_controllers_path,
            value_type=str
        ),

        # 基础配置
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "publish_planning_scene": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_geometry_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_state_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_transforms_updates": LaunchConfiguration("publish_monitored_planning_scene"),

        # 轨迹执行参数
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,

        # 传感器配置
        "monitor_dynamics": False,
        "use_sim_time": LaunchConfiguration("use_sim_time")
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration
    ]

    # 添加move_group节点
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": ":0"},
    )

    return ld


def my_generate_moveit_rviz_launch(ld, moveit_config):
    # 配置RViz参数
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("arm_moveit_config"),
        "config",
        "moveit.rviz"
    ])

    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        {"use_sim_time": LaunchConfiguration("use_sim_time")}
    ]

    # 添加RViz节点
    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=rviz_parameters,
    )

    return ld