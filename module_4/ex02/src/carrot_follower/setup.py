from setuptools import setup

package_name = 'carrot_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/carrot_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Egor',
    maintainer_email='enderser4@gmail.com',
    description='TF2 carrot follower demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle1_tf2_broadcaster = carrot_follower.turtle1_tf2_broadcaster:main',
            'turtle2_tf2_broadcaster = carrot_follower.turtle2_tf2_broadcaster:main',
            'carrot_tf2_broadcaster = carrot_follower.carrot_tf2_broadcaster:main',
            'turtle2_carrot_follower = carrot_follower.turtle2_carrot_follower:main',
        ],
    },
)
