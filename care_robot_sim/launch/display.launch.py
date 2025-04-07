import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    package_share_dir = get_package_share_directory('care_robot_sim')
    urdf_file = os.path.join(package_share_dir, 'urdf', 'cobot_gripper_camera.urdf.xacro')
    rviz_config_file = os.path.join(package_share_dir, 'config', 'config.rviz')

    # Declare the use_gui argument
    use_gui_arg = DeclareLaunchArgument(
        name='use_gui', 
        default_value='true', 
        description='Flag to enable joint_state_publisher_gui'
    )

    # Define the robot_description parameter using xacro
    robot_description = Command(['xacro ', urdf_file])

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    ld.add_action(use_gui_arg)
    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)

    return ld
