from setuptools import find_packages, setup
from glob import glob

package_name = 'vehicle_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpslab',
    maintainer_email='cpslab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_node = vehicle_sensors.imu_node:main',
            'wheel_odom_node = vehicle_sensors.wheel_odom_node:main',
            'imu_bno055_i2c = vehicle_sensors.imu_bno055_i2c:main',

        
        ],
    },
)
