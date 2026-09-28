import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- MEMBER 3: ROS2 & SYSTEM INTEGRATION CORE ---
        Node(
            package='uav_system_orchestrator',
            executable='mission_state_machine',
            name='mission_state_machine',
            output='screen',
            parameters=[{'takeoff_altitude': 2.5, 'acceptance_radius': 0.35}]
        ),
        Node(
            package='uav_system_orchestrator',
            executable='system_integration_bridge',
            name='system_integration_bridge',
            output='screen'
        ),
        Node(
            package='uav_system_orchestrator',
            executable='tf_tree_manager',
            name='tf_tree_manager',
            output='screen'
        ),
        Node(
            package='uav_system_orchestrator',
            executable='safety_failsafe_node',
            name='safety_failsafe_node',
            output='screen'
        ),
        Node(
            package='uav_system_orchestrator',
            executable='system_health_monitor',
            name='system_health_monitor',
            output='screen'
        ),
        Node(
            package='uav_mavros_bridge',
            executable='mavros_commander',
            name='mavros_commander',
            output='screen'
        ),

        # --- SYSTEM PEERS (MOCKS FOR REVIEW EVALUATION) ---
        # Member 1: Vision-Language Navigation Goals
        Node(
            package='uav_mocks',
            executable='member1_vlm_mock',
            name='member1_vlm_mock',
            output='screen'
        ),
        # Member 2: Gazebo Harmonic SITL Dynamics
        Node(
            package='uav_mocks',
            executable='member2_flight_sim_mock',
            name='member2_flight_sim_mock',
            output='screen'
        ),
        # Member 4: GPS-Denied Visual SLAM (ORB-SLAM3)
        Node(
            package='uav_mocks',
            executable='member4_slam_mock',
            name='member4_slam_mock',
            output='screen'
        ),
        # Member 5: Adaptive Semantic Memory Database
        Node(
            package='uav_mocks',
            executable='member5_memory_mock',
            name='member5_memory_mock',
            output='screen'
        ),
    ])
