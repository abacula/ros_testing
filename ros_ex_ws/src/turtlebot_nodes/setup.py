from setuptools import find_packages, setup

package_name = 'turtlebot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexandra.bacula',
    maintainer_email='alexandra.bacula@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'lab2_test_node = turtlebot_nodes.lab2_test:main',
        'pub_pose = turtlebot_nodes.pub_pose_filtered:main',
        'map_pub = turtlebot_nodes.occupancy_map:main',
        ],
    },
)
