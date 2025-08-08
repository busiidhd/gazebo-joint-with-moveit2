from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的路径
    gazebo_moeit_launch_dir = get_package_share_directory('arm_moveit_config')
    
    # 定义Gazebo启动文件路径
    gazebo_launch_path = os.path.join(
        gazebo_moeit_launch_dir,
        'launch',
        'gazebo.launch.py'
    )
    
    # 定义RViz和MoveIt启动文件路径
    rviz_moveit_launch_path = os.path.join(
        gazebo_moeit_launch_dir,
        'launch',
        'rviz2.launch.py'
    )
    
    # 包含Gazebo启动文件
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )
    
    # 包含RViz和MoveIt启动文件
    delayed_rviz_moveit = TimerAction(
        period=7.0,  # 延迟时间（秒）
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_moveit_launch_path)
            )
        ]
    )
    
    return LaunchDescription([
        include_gazebo,       
        delayed_rviz_moveit  
    ])
