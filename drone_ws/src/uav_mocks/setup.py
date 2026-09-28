from setuptools import setup
import os

package_name = 'uav_mocks'

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
    description='High-Fidelity Mocks for UAV System Integration Demonstrations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'member1_vlm_mock = uav_mocks.member1_vlm_mock:main',
            'member2_flight_sim_mock = uav_mocks.member2_flight_sim_mock:main',
            'member4_slam_mock = uav_mocks.member4_slam_mock:main',
            'member5_memory_mock = uav_mocks.member5_memory_mock:main',
        ],
    },
)
