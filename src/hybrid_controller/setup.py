from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hybrid_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'scipy>=1.10.0',
        'cvxpy>=1.4.0',
        'matplotlib>=3.7.0',
        'pyyaml>=6.0',
    ],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='user@example.com',
    description='Risk-Aware Hybrid LQR-MPC Navigation Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_node = hybrid_controller.nodes.trajectory_node:main',
            'lqr_node = hybrid_controller.nodes.lqr_node:main',
            'mpc_node = hybrid_controller.nodes.mpc_node:main',
            'state_estimator_node = hybrid_controller.nodes.state_estimator_node:main',
        ],
    },
)
