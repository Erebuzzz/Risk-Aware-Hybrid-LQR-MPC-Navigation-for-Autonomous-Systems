import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hybrid_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') if glob('config/*.yaml') else []),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kshitiz and Agolika',
    maintainer_email='kshitiz23@iiserb.ac.in',
    description='Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems',
    license='MIT',
    entry_points={
        'console_scripts': [
            'test_node = hybrid_nav.nodes.test_node:main',
            'hybrid_controller_node = hybrid_nav.nodes.hybrid_controller_node:main',
            'obstacle_publisher_node = hybrid_nav.nodes.obstacle_publisher_node:main',
            'drive_test_node = hybrid_nav.nodes.drive_test_node:main',
            'lqr_controller_node = hybrid_nav.nodes.lqr_controller_node:main',
            'mpc_controller_node = hybrid_nav.nodes.mpc_controller_node:main',
            'adaptive_mpc_controller_node = hybrid_nav.nodes.adaptive_mpc_controller_node:main',
            'twist_relay_node = hybrid_nav.nodes.twist_relay_node:main',
        ],
    },
)
