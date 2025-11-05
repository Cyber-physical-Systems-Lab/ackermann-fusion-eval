#!/usr/bin/env python3
# 不再广播 TF；仅发布 Odometry.twist 里的 vx，供 EKF 融合
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

BIG = 1e6   # “不知道”的超大方差
SMALL = 1e-2  # vx 的合理不确定度（按需再调）

class WheelOdomFromWheelVel(Node):
    def __init__(self):
        super().__init__('wheel_odom_from_wheelvel')

        self.topic_in   = self.declare_parameter('topic_in',   '/wheel_vel').get_parameter_value().string_value
        self.topic_out  = self.declare_parameter('topic_out',  '/wheel_odom').get_parameter_value().string_value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.radius     = self.declare_parameter('wheel_radius', 0.018).get_parameter_value().double_value
        self.invert     = self.declare_parameter('invert', False).get_parameter_value().bool_value

        # 输入（wheel_vel）：高频传感器 QoS
        self.create_subscription(Float32, self.topic_in, self.cb, qos_profile_sensor_data)
        # 输出（wheel_odom）：同样用 sensor_data（对 EKF 友好、跨进程抗抖）
        self.pub = self.create_publisher(Odometry, self.topic_out, qos_profile_sensor_data)

        self.get_logger().info(
            f'wheel_odom_from_wheelvel: topic_in={self.topic_in}, out={self.topic_out}, R={self.radius:.3f} m, invert={self.invert}'
        )

    def cb(self, msg: Float32):
        omega = float(msg.data)
        if self.invert:
            omega = -omega
        vx = self.radius * omega  # m/s

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        # 不提供位姿：pose 全 0 + 超大方差，确保 EKF 不会把它当位姿观测
        # 对角线索引：x(0) y(7) z(14) roll(21) pitch(28) yaw(35)
        diag = [0,7,14,21,28,35]
        for i in diag:
            odom.pose.covariance[i] = BIG

        # 只提供线速度 x；其它速度/角速度用超大方差“屏蔽”
        # twist 对角索引：vx(0) vy(7) vz(14) vroll(21) vpitch(28) vyaw(35)
        for i in [0,7,14,21,28,35]:
            odom.twist.covariance[i] = BIG
        odom.twist.twist.linear.x = vx
        odom.twist.covariance[0]  = SMALL  # 仅 vx 有效

        self.pub.publish(odom)

def main():
    rclpy.init()
    n = WheelOdomFromWheelVel()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
