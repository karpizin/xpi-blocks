from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Paths
    project_path = Path(__file__).resolve().parents[1]
    urdf_file = project_path / 'urdf' / 'hexapod.urdf.xacro'

    # 2. Process URDF (using xacro even for plain URDF for future proofing)
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # 3. Nodes
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Gazebo Launch
    # We include standard gazebo_ros launch and pass our world file
    world_file = project_path / 'urdf' / 'hexapod_terrain.world'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('gazebo_ros')) / 'launch' / 'gazebo.launch.py')
        ),
        launch_arguments={'world': str(world_file)}.items()
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hexapod'],
        output='screen'
    )

    # Load Controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_hexapod_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'hexapod_joint_controller'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        # Load controllers after spawning
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_hexapod_controller],
            )
        ),
    ])
