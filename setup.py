from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools', 
        'rclpy', 
        'std_msgs', 
        'geometry_msgs'
    ],
    zip_safe=True,
    maintainer='hidood',
    maintainer_email='hidood@todo.todo',
    description='A ROS2 package for controlling robot status, commands, and velocity.',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'status_publisher = robot_controller.status_publisher:main',
            'command_subscriber = robot_controller.command_subscriber:main',
            'velocity_publisher = robot_controller.velocity_publisher:main',
        ],
    },
)
