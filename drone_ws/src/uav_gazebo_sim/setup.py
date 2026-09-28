from setuptools import setup
import os
from glob import glob

package_name = 'uav_gazebo_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member 2 (Simulation & Flight Stack Engineer)',
    maintainer_email='member2.sim@drone-ai.org',
    description='Gazebo Harmonic Simulation Environment and Worlds for Autonomous UAV',
    license='Apache-2.0',
    tests_require=['pytest'],
)
