import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_pal.robot_utils import (get_arm,
                                    get_camera_model,
                                    get_end_effector,
                                    get_ft_sensor,
                                    get_laser_model,
                                    get_robot_name,
                                    get_wrist_model)

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
            'config', 'tiago_fake.urdf.xacro'), mappings=mappings)
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'tiago_pal-gripper.srdf'))
        .robot_description_kinematics(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'kinematics_kdl.yaml'))
        .trajectory_execution(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'fake_controllers_manager.yaml'))
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

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
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
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_footprint"],
    )

    # Publish TF

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[#params,
                    moveit_config.robot_description
                    ]
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("tiago_mtc_examples"),
        "config",
        "fake_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "gripper_controller",
        "arm_controller",
        "torso_controller",
        "head_controller",
        "joint_state_broadcaster"
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )