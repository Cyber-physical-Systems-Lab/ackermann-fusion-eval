from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_hw = LaunchConfiguration('use_hw_imu')

    return LaunchDescription([
        # 是否启用硬件 BNO055（默认关）
        DeclareLaunchArgument(
            'use_hw_imu', default_value='false',
            description='Use hardware BNO055 over I2C if true; else use simulated IMU.'
        ),

        # ---- 模拟 IMU ----
        Node(
            package='vehicle_sensors',
            executable='imu_node',
            name='imu_sim',
            parameters=[{
                'rate_hz': 100.0,
                'frame_id': 'imu_link',
                'pub_orientation': False
            }],
            condition=UnlessCondition(use_hw)
        ),

        # ---- 硬件 IMU（BNO055 I2C）----
        Node(
            package='vehicle_sensors',
            executable='imu_bno055_i2c',
            name='imu_bno055',
            parameters=[{
                'i2c_bus': 1,
                'i2c_addr': 0x28,
                'rate_hz': 50.0,
                'frame_id': 'imu_link',
                'use_ndof': False
            }],
            condition=IfCondition(use_hw)
        ),

        # ---- 轮速里程计 ----
        Node(
            package='vehicle_sensors',
            executable='wheel_odom_node',
            name='wheel_odom',
            parameters=[{
                'rate_hz': 50.0,
                'vx': 0.2,
                'wz': 0.0
            }]
        ),

        # ---- 静态 TF：base_link -> imu_link（先设为同一点）----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
    ])
