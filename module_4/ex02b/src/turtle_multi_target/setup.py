from setuptools import setup
import os
from glob import glob

package_name = 'turtle_multi_target'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_target_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='enderser4@gmail.com',
    description='Multi-target turtle follower with TF2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle1_tf2_broadcaster = turtle_multi_target.turtle1_tf2_broadcaster:main',
            'turtle2_tf2_broadcaster = turtle_multi_target.turtle2_tf2_broadcaster:main',
            'turtle3_tf2_broadcaster = turtle_multi_target.turtle3_tf2_broadcaster:main',
            'target_switcher = turtle_multi_target.target_switcher:main',
            'turtle_controller = turtle_multi_target.turtle_controller:main',
        ],
    },
)