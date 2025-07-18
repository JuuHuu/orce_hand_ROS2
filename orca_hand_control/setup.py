from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'orca_hand_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/orca_core', glob('orca_hand_control/orca_core/*.*')),
        ('share/' + package_name + '/orca_core/api', glob('orca_hand_control/orca_core/api/*.*')),
        ('share/' + package_name + '/orca_core/hardware', glob('orca_hand_control/orca_core/hardware/*.*')),
        ('share/' + package_name + '/orca_core/models/orcahand_v1_right', glob('orca_hand_control/orca_core/models/orcahand_v1_right/*.*')),
        ('share/' + package_name + '/orca_core/utils', glob('orca_hand_control/orca_core/utils/*.*')),
        ('share/' + package_name + '/urdf', glob('orca_hand_control/models/urdf/*.*')),
        ('share/' + package_name + '/assets/urdf/left/collision', glob('orca_hand_control/assets/urdf/left/collision/*.*')),
        ('share/' + package_name + '/assets/urdf/left/visual', glob('orca_hand_control/assets/urdf/left/visual/*.*')),
        ('share/' + package_name + '/assets/urdf/right/collision', glob('orca_hand_control/assets/urdf/right/collision/*.*')),
        ('share/' + package_name + '/assets/urdf/right/visual', glob('orca_hand_control/assets/urdf/right/visual/*.*')),
        ('share/' + package_name + '/rviz', glob('orca_hand_control/rviz/config.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.*')),
        
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juu',
    maintainer_email='hujunem95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_orca_hand = orca_hand_control.test:main",
            'publish_robot_joint_states = orca_hand_control.joint_publisher:main',
            'publish_human_joint_states = orca_hand_control.human_joint:main',
            'publish_human_joint_angle = orca_hand_control.human_joint_anlge:main'
        ],
    },
)
