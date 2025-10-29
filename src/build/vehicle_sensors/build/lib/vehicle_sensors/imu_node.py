import math, random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

def sq(x): return x * x

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ====== 频率与帧名 ======
        self.rate_hz = self.declare_parameter('rate_hz', 100.0).get_parameter_value().double_value
        self.dt = 1.0 / self.rate_hz
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').get_parameter_value().string_value

        # ====== 陀螺零偏（rad/s），默认接近你统计的数量级 ======
        self.gyro_bias_x = self.declare_parameter('gyro_bias_x', 9.0e-5).get_parameter_value().double_value
        self.gyro_bias_y = self.declare_parameter('gyro_bias_y', -2.2e-5).get_parameter_value().double_value
        self.gyro_bias_z = self.declare_parameter('gyro_bias_z', 5.9e-5).get_parameter_value().double_value

        # ====== 陀螺噪声标准差（rad/s） ======
        self.gyro_sd_x = self.declare_parameter('gyro_sd_x', 0.0015).get_parameter_value().double_value
        self.gyro_sd_y = self.declare_parameter('gyro_sd_y', 0.0021).get_parameter_value().double_value
        self.gyro_sd_z = self.declare_parameter('gyro_sd_z', 0.0026).get_parameter_value().double_value

        # ====== 加计噪声标准差（m/s^2），含重力 ======
        self.acc_sd_x = self.declare_parameter('acc_sd_x', 0.015).get_parameter_value().double_value
        self.acc_sd_y = self.declare_parameter('acc_sd_y', 0.014).get_parameter_value().double_value
        self.acc_sd_z = self.declare_parameter('acc_sd_z', 0.036).get_parameter_value().double_value

        # ====== 是否发布 orientation（若不用可设为 False） ======
        self.pub_orientation = self.declare_parameter('pub_orientation', False).get_parameter_value().bool_value
        # 简单模拟一个“几乎单位四元数”的姿态
        self.yaw = 0.0

        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'IMU sim @ {self.rate_hz:.1f} Hz | gyro_sd=({self.gyro_sd_x},{self.gyro_sd_y},{self.gyro_sd_z}) '
            f'| acc_sd=({self.acc_sd_x},{self.acc_sd_y},{self.acc_sd_z})'
        )

    def tick(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # === 角速度：零偏 + 白噪声（静止假设） ===
        msg.angular_velocity.x = self.gyro_bias_x + random.gauss(0.0, self.gyro_sd_x)
        msg.angular_velocity.y = self.gyro_bias_y + random.gauss(0.0, self.gyro_sd_y)
        msg.angular_velocity.z = self.gyro_bias_z + random.gauss(0.0, self.gyro_sd_z)

        # 协方差（对角）
        msg.angular_velocity_covariance[0] = sq(self.gyro_sd_x)
        msg.angular_velocity_covariance[4] = sq(self.gyro_sd_y)
        msg.angular_velocity_covariance[8] = sq(self.gyro_sd_z)

        # === 线加速度：含重力（z 轴约 +9.81），加上白噪声 ===
        g = 9.80665
        msg.linear_acceleration.x = random.gauss(0.0, self.acc_sd_x)
        msg.linear_acceleration.y = random.gauss(0.0, self.acc_sd_y)
        msg.linear_acceleration.z = g + random.gauss(0.0, self.acc_sd_z)

        msg.linear_acceleration_covariance[0] = sq(self.acc_sd_x)
        msg.linear_acceleration_covariance[4] = sq(self.acc_sd_y)
        msg.linear_acceleration_covariance[8] = sq(self.acc_sd_z)

        # === orientation（可选）===
        if self.pub_orientation:
            # 只做平面小幅度 yaw 漂移的演示
            self.yaw += random.gauss(0.0, 1e-4)
            half = self.yaw * 0.5
            msg.orientation.z = math.sin(half)
            msg.orientation.w = math.cos(half)
            # 给一个很小的协方差
            small = 1e-6
            msg.orientation_covariance[0] = small
            msg.orientation_covariance[4] = small
            msg.orientation_covariance[8] = small
        else:
            msg.orientation_covariance[0] = -1.0  # 未提供姿态

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
