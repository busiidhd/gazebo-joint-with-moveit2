import os
import re
from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def replace_package_paths(urdf_content):
    package_pattern = re.compile(r'package://([^/]+)/')
    match = package_pattern.search(urdf_content)
    if match:
        package_name = match.group(1)
        try:
            package_path = get_package_share_directory(package_name)
            return urdf_content.replace(f"package://{package_name}", package_path)
        except Exception as e:
            print(f"\u8b66\u544a\uff1a\u65e0\u6cd5\u66ff\u6362\u5305\u8def\u5f84 {package_name}\uff0c\u4f7f\u7528\u539f\u59cb\u8def\u5f84: {e}")
    return urdf_content


def generate_launch_description():
    robot_name = "arm"
    urdf_package = "arm_description"
    config_package = "arm_moveit_config"
    world_file = "empty.world"

    arm_xacro_path = os.path.join(
        get_package_share_directory(urdf_package),
        "xacro", "arm.urdf.xacro"
    )

    second_robot_name = "exchanger"
    second_urdf_path = "/home/benson/chart/gazebo/src/exchanger/urdf/exchanger.urdf"

    controller_config_path = os.path.join(
        get_package_share_directory(config_package), 
        "config", 
        "ros2_controllers.yaml"
    )
    world_path = os.path.join(
        get_package_share_directory(urdf_package), 
        "world", 
        world_file
    )

    try:
        arm_urdf_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                arm_xacro_path,
                " use_sim_time:=true",
            ]
        ).perform(LaunchContext())
        arm_urdf_content = replace_package_paths(arm_urdf_content)
    except Exception as e:
        print(f"arm\u7684Xacro\u89e3\u6790\u9519\u8bef: {e}")
        arm_urdf_content = f'<robot name="{robot_name}"><link name="base_link"/></robot>'

    try:
        with open(second_urdf_path, 'r') as f:
            second_urdf_content = f.read()
        second_urdf_content = replace_package_paths(second_urdf_content)
    except Exception as e:
        print(f"exchanger\u7684URDF\u89e3\u6790\u9519\u8bef: {e}")
        second_urdf_content = f'<robot name="{second_robot_name}"><link name="base_link"/></robot>'

    start_gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        additional_env={"GAZEBO_MODEL_PATH": os.environ.get("GAZEBO_MODEL_PATH", "")}
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": arm_urdf_content},
            {"use_sim_time": True},
            {"publish_frequency":100.0}
        ]
    )

    second_robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{second_robot_name}_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": second_urdf_content},
            {"use_sim_time": True},
            {"publish_frequency": 50.0}
        ],
        remappings=[
            ("robot_description", f"{second_robot_name}/robot_description")
        ]
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_arm",
        arguments=[
            "-entity", robot_name,
            "-topic", "/robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.0"
        ],
        output="screen"
    )

    spawn_second_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name=f"spawn_{second_robot_name}",
        arguments=[
            "-entity", second_robot_name,
            "-topic", f"{second_robot_name}/robot_description",
            "-x", "1.0", "-y", "-0.4", "-z", "0.3",
            "-R", "0.0", "-P", "0.0", "-Y", "1.70"
        ],
        output="screen"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            controller_config_path
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    load_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["test_group_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    start_rsp_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=start_gazebo,
            on_start=[robot_state_pub, second_robot_state_pub]
        )
    )

    spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_pub,
            on_start=[
                TimerAction(period=2.0, actions=[spawn_robot]),
                TimerAction(period=4.5, actions=[spawn_second_robot])
            ]
        )
    )

    start_cm_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_second_robot,
            on_exit=[TimerAction(period=5.0, actions=[controller_manager])]
        )
    )

    load_controllers_after_cm = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=2.0, actions=[load_joint_broadcaster]),
                TimerAction(period=4.0, actions=[load_arm_controller])
            ]
        )
    )

    return LaunchDescription([
        start_rsp_after_gazebo,
        spawn_after_rsp,
        start_cm_after_spawn,
        load_controllers_after_cm,
        start_gazebo
    ])
