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
    """将package://路径替换为绝对路径，确保摄像头等资源正常加载"""
    package_pattern = re.compile(r'package://([^/]+)/')
    match = package_pattern.search(urdf_content)
    if match:
        package_name = match.group(1)
        try:
            package_path = get_package_share_directory(package_name)
            return urdf_content.replace(f"package://{package_name}", package_path)
        except Exception as e:
            print(f"警告：无法替换包路径 {package_name}，使用原始路径: {e}")
    return urdf_content


def generate_launch_description():
    # 1. 配置核心参数（仅修改arm为Xacro，exchanger保持URDF）
    robot_name = "arm"
    urdf_package = "arm_description"          # arm的Xacro所在包
    config_package = "arm_moveit_config"
    world_file = "empty.world"
    
    # ********** 关键修改：arm使用Xacro文件路径 **********
    arm_xacro_path = os.path.join(
        get_package_share_directory(urdf_package),
        "xacro", "arm.urdf.xacro"  # 替换为你的arm的Xacro实际路径
    )

    # exchanger保持URDF不变
    second_robot_name = "exchanger"
    second_urdf_path = "/home/benson/chart/gazebo/src/exchanger/urdf/exchanger.urdf"

    # 2. 计算其他文件路径（不变）
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

    # 3. 解析文件（仅修改arm的解析逻辑，exchanger不变）
    # 3.1 解析arm的Xacro（包含摄像头插件）
    try:
        # 强制用xacro解析arm的Xacro文件
        arm_urdf_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),  # 调用xacro工具
                " ",
                arm_xacro_path,  # arm的Xacro路径
                " use_sim_time:=true",  # 传递仿真时间参数
                # 可添加Xacro插件所需参数，例如：" camera_frame:=arm_camera_link"
            ]
        ).perform(LaunchContext())  # 执行解析并获取URDF内容

        # 替换路径（确保摄像头模型、插件配置文件能被找到）
        arm_urdf_content = replace_package_paths(arm_urdf_content)
    except Exception as e:
        print(f"arm的Xacro解析错误: {e}")
        arm_urdf_content = f'<robot name="{robot_name}"><link name="base_link"/></robot>'

    # 3.2 exchanger保持URDF解析逻辑（不变）
    try:
        with open(second_urdf_path, 'r') as f:
            second_urdf_content = f.read()
        second_urdf_content = replace_package_paths(second_urdf_content)
    except Exception as e:
        print(f"exchanger的URDF解析错误: {e}")
        second_urdf_content = f'<robot name="{second_robot_name}"><link name="base_link"/></robot>'

    # 4. 定义启动节点（仅arm的状态发布器使用解析后的Xacro内容，其余不变）
    # 4.1 启动Gazebo（不变）
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

    # 4.2 arm的状态发布器（使用Xacro解析后的URDF）
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": arm_urdf_content},  # 传入Xacro解析后的内容
            {"use_sim_time": True},
            {"publish_frequency":100.0}
        ]
    )

    # 4.3 exchanger的状态发布器（不变）
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

    # 4.4 生成arm（包含摄像头）
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_arm",
        arguments=[
            "-entity", robot_name,
            "-topic", "/robot_description",  # 从话题获取Xacro解析后的URDF
            "-x", "0.0", "-y", "0.0", "-z", "0.0"
        ],
        output="screen"
    )

    # 4.5 生成exchanger（不变）
    spawn_second_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name=f"spawn_{second_robot_name}",
        arguments=[
            "-entity", second_robot_name,
            "-topic", f"{second_robot_name}/robot_description",
            "-x", "0.8", "-y", "-0.2", "-z", "0.3",
            "-R", "0.0", "-P", "3.14159", "-Y", "4.75"
        ],
        output="screen"
    )

    # 4.6 控制器管理器及加载逻辑（不变）
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

    load_joint_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen"
    )

    load_arm_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "test_group_controller"],
        output="screen"
    )

    # 5. 启动顺序控制（不变）
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
                TimerAction(period=1.0, actions=[spawn_robot]),
                TimerAction(period=1.5, actions=[spawn_second_robot])
            ]
        )
    )

    start_cm_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_second_robot,
            on_exit=[TimerAction(period=2.0, actions=[controller_manager])]
        )
    )

    load_broadcaster_after_cm = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[load_joint_broadcaster]
        )
    )

    load_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_broadcaster,
            on_exit=[load_arm_controller]
        )
    )

    # 6. 组装启动流程
    return LaunchDescription([
        start_rsp_after_gazebo,
        spawn_after_rsp,
        start_cm_after_spawn,
        load_broadcaster_after_cm,
        load_controller_after_broadcaster,
        start_gazebo
    ])