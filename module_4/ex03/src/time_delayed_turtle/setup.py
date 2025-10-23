from setuptools import setup
import os
from glob import glob

package_name = 'time_delayed_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/time_delay_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='enderser4@gmail.com',
    description='Time-delayed turtle follower with TF2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle1_tf2_broadcaster = time_delayed_turtle.turtle1_tf2_broadcaster:main',
            'turtle2_tf2_broadcaster = time_delayed_turtle.turtle2_tf2_broadcaster:main',
            'delayed_tf2_broadcaster = time_delayed_turtle.delayed_tf2_broadcaster:main',
            'turtle2_time_follower = time_delayed_turtle.turtle2_time_follower:main',
        ],
    },
)