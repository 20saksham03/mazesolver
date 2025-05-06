from setuptools import setup

package_name = 'maze_solver_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Maze solver bot using Lidar, IMU, and Odometry for ROS 2 TurtleBot3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wall_follower = maze_solver_bot.wall_follower:main',
        ],
    },
)
