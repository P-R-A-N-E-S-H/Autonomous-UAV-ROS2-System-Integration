from setuptools import setup
import os
from glob import glob

package_name = 'uav_flight_stack'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params'), glob('params/*.*')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member 2 (Simulation & Flight Stack Engineer)',
    maintainer_email='member2.sim@drone-ai.org',
    description='ArduPilot SITL Flight Stack Configurations and Mission Executor for Autonomous UAV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_stack_mission_executor = uav_flight_stack.flight_stack_mission_executor:main',
        ],
    },
)
