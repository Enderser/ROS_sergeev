from setuptools import setup
from glob import glob
import os

package_name = 'cool_movement'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Установка launch-файлов
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='enderser4@gmail.com',
    description='Unique sinusoid movement for cool robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sinusoid_movement = cool_movement.sinusoid_movement:main',
        ],
    },
)
