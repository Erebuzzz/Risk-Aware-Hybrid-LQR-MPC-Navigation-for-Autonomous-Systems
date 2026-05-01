from setuptools import find_packages, setup

package_name = 'hybrid_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='Kshitiz and Agolika',
    maintainer_email='kshitiz23@iiserb.ac.in',
    description='Risk-Aware Hybrid LQR-MPC Controller Library',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
