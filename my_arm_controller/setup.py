from setuptools import find_packages, setup

package_name = 'my_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'trajectory_msgs'],
    zip_safe=True,
    maintainer='axmx',
    maintainer_email='adedok495@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'arm_controller = my_arm_controller.arm_controller:main',
        ],
    },
)
