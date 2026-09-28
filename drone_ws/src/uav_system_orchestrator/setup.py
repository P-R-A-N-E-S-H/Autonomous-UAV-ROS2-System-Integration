from setuptools import setup
import os
from glob import glob

package_name = 'uav_system_orchestrator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member 3 (System Integration Engineer)',
    maintainer_email='member3.elitebook@drone-ai.org',
    description='Central System Integration & State Machine Orchestrator for Autonomous UAV Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_state_machine = uav_system_orchestrator.mission_state_machine:main',
            'system_integration_bridge = uav_system_orchestrator.system_integration_bridge:main',
            'tf_tree_manager = uav_system_orchestrator.tf_tree_manager:main',
            'safety_failsafe_node = uav_system_orchestrator.safety_failsafe_node:main',
            'system_health_monitor = uav_system_orchestrator.system_health_monitor:main',
        ],
    },
)
