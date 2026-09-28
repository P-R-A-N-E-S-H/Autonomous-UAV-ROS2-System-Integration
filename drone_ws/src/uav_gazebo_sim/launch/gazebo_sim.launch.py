import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo_sim = get_package_share_directory('uav_gazebo_sim') if os.path.exists('uav_gazebo_sim') else '.'
    world_path = os.path.join(pkg_gazebo_sim, 'worlds', 'warehouse_inspection.sdf')
    bridge_config = os.path.join(pkg_gazebo_sim, 'config', 'ros_gz_bridge.yaml')

    return LaunchDescription([
        # Set Gazebo Resource Path
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_gazebo_sim, 'models')
        ),

        # Gazebo Harmonic Simulator Server & GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]) if os.path.exists('/opt/ros') else PythonLaunchDescriptionSource([]),
            launch_arguments={'gz_args': f'-r {world_path}'}.items()
        ),

        # ROS-GZ Bridge Node
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}]
        )
    ])
