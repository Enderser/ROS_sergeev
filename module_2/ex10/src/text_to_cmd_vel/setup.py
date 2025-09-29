from setuptools import setup

package_name = 'text_to_cmd_vel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergeev Egor',
    maintainer_email='enderser4@gmail.com',
    description='Convert text commands to geometry_msgs/Twist for turtlesim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_cmd_vel = text_to_cmd_vel.text_to_cmd_vel:main'
        ],
    },
)
