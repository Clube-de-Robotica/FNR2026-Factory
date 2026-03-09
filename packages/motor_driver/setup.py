from setuptools import setup

package_name = 'motor_driver'

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
    maintainer='Interlade Studios',
    maintainer_email='user@example.com',
    description='ROS 2 driver for a 4-motor controller with encoders',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = motor_driver.motor_driver_node:main',
        ],
    },
)
