from setuptools import find_packages, setup

package_name = 'my_publisher_pkg'

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
    maintainer='mk',
    maintainer_email='mk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_publisher = my_publisher_pkg.my_publisher:main',
            'pose_subscriber = my_publisher_pkg.pose_subscriber:main',
            'move_turtle = my_publisher_pkg.move_turtle:main',
            'turtle_controller = my_publisher_pkg.turtle_controller:main'
        ],
    },
)
