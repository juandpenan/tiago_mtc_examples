import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # planning_context

    mappings = {
        'arm': 'right-arm',
        'camera_model': 'orbbec-astra',
        'end_effector': 'pal-gripper',
        'ft_sensor': 'schunk-ft',
        'laser_model': 'sick-571',
        'wrist_model': 'wrist-2010'
    }
    
    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'tiago.urdf.xacro'), mappings=mappings)
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'tiago_pal-gripper.srdf'))
        .robot_description_kinematics(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'kinematics_kdl.yaml'))
        .trajectory_execution(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'controllers_pal-gripper.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor({
                                    'publish_planning_scene': True,
                                    'publish_geometry_updates': True,
                                    'publish_state_updates': True,
                                    'publish_transforms_updates': True,
        })
        .pilz_cartesian_limits(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("tiago_mtc_examples") + "/launch/tiago.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {'use_sim_time': False},
         

        ],
    )

    return LaunchDescription(
        [
            rviz_node,
        ]
    )
