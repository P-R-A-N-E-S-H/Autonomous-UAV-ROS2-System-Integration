import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. Central Mission State Machine (Member 3 Flagship)
        Node(
            package='uav_system_orchestrator',
            executable='mission_state_machine',
            name='mission_state_machine',
            output='screen',
            parameters=[{
                'takeoff_altitude': 2.5,
                'acceptance_radius': 0.35,
                'rate_hz': 20.0
            }]
        ),

        # 2. System Integration Bridge (Member 3 Inter-Module Hub)
        Node(
            package='uav_system_orchestrator',
            executable='system_integration_bridge',
            name='system_integration_bridge',
            output='screen',
            parameters=[{
                'max_velocity': 1.8,
                'max_acceleration': 1.2,
                'publish_rate_hz': 50.0
            }]
        ),

        # 3. Dynamic TF Tree Broadcaster (Member 3 Coordinate Transform Manager)
        Node(
            package='uav_system_orchestrator',
            executable='tf_tree_manager',
            name='tf_tree_manager',
            output='screen'
        ),

        # 4. Flight Safety & Fail-Safe Supervisor (Member 3 Safety Guard)
        Node(
            package='uav_system_orchestrator',
            executable='safety_failsafe_node',
            name='safety_failsafe_node',
            output='screen'
        ),

        # 5. QoS, Latency & Health Diagnostics Monitor (Member 3 Monitor)
        Node(
            package='uav_system_orchestrator',
            executable='system_health_monitor',
            name='system_health_monitor',
            output='screen'
        ),

        # 6. MAVROS Flight Stack Commander Bridge
        Node(
            package='uav_mavros_bridge',
            executable='mavros_commander',
            name='mavros_commander',
            output='screen',
            parameters=[{
                'autopilot_type': 'ArduPilot',
                'stream_rate_hz': 50.0
            }]
        ),
    ])
