from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import TimerEvent
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('care_robot_sim')
    human_pkg_share = get_package_share_directory('human-gazebo')
    robot_description_file = os.path.join(pkg_share, 'urdf', 'cobot_gripper_camera.urdf.xacro')
    human_description_file = os.path.join(human_pkg_share, 'humanSubjectWithMeshes', 'humanSubjectWithMesh.urdf')
    joint_controllers_file = os.path.join(pkg_share, 'config', 'controllers_trajectory.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'patience_env.world')
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    # Robot description
    robot_description = Command(['xacro ', robot_description_file])

    # Argomenti per la posizione
    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')
    human_x_arg = DeclareLaunchArgument('human_x', default_value='2.0', description='X position of the human')
    human_y_arg = DeclareLaunchArgument('human_y', default_value='1.0', description='Y position of the human')
    human_z_arg = DeclareLaunchArgument('human_z', default_value='0.0', description='Z position of the human')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'debug': 'false',
            'gui': 'true',
            'paused': 'true',
            'world': world_file
        }.items()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('care_robot_sim'), 'config', 'config.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output = "screen"
    )


    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cobot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen',
    )

    # Spawn human
    spawn_human = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'human',
            '-file', human_description_file,
            '-x', LaunchConfiguration('human_x'),
            '-y', LaunchConfiguration('human_y'),
            '-z', LaunchConfiguration('human_z')
        ],
        output='screen',
    )

    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, joint_controllers_file],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_trajectory_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[gripper_position_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(human_x_arg)
    ld.add_action(human_y_arg)
    ld.add_action(human_z_arg)
    ld.add_action(gazebo)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_human)


    ld.add_action(robot_state_publisher)


    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_rviz_node)

    return ld
