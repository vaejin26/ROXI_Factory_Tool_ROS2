from setuptools import find_packages, setup

package_name = 'roxi_communication'

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
    maintainer='bae',
    maintainer_email='bae@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_comm_node = roxi_communication.serial_comm_node:main',
            'mcu_comm_node = roxi_communication.mcu_comm_node:main',
            'imu_comm_node = roxi_communication.imu_comm_node:main',
            'tray_comm_node = roxi_communication.tray_comm_node:main',
            'motor_comm_node = roxi_communication.motor_comm_node:main',
            'lidar_comm_node = roxi_communication.lidar_comm_node:main'
        ],
    },
)
