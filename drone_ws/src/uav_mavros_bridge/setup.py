from setuptools import setup
import os

package_name = 'uav_mavros_bridge'

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
    description='MAVROS and MAVLink Flight Control Bridge for Autonomous Drone Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavros_commander = uav_mavros_bridge.mavros_commander:main',
            'mavlink_telemetry_hub = uav_mavros_bridge.mavlink_telemetry_hub:main',
        ],
    },
)
