from setuptools import setup
import os
from glob import glob

package_name = 'action_cleaning_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Egor',
    maintainer_email='enderser4@gmail.com',
    description='Cleaning robot action server and client',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cleaning_action_server = action_cleaning_robot.cleaning_action_server:main',
            'cleaning_action_client = action_cleaning_robot.cleaning_action_client:main',
        ],
    },
)
