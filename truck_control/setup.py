from setuptools import setup, find_packages

package_name = 'truck_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='tmo',
    maintainer_email='tmo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_publisher = truck_control.command_publisher:main',
            'lane_following_node = truck_control.lane_following_node:main',
            'lane_detect = truck_control.lane_detect:main',
            'pid_controller = truck_control.pid_controller:main',
            'distance_sensor = truck_control.distance_sensor:main',
            'platooning_manager = truck_control.platooning_manager:main',
            'v2v_comm = truck_control.v2v_comm:main',
            'ui_k = truck_control.ui_k:main',
            'ui_tkinter = truck_control.ui_tkinter:main',
            'carla_spectator_follower = truck_control.carla_spectator_follower:main',
        ],
    },
)
