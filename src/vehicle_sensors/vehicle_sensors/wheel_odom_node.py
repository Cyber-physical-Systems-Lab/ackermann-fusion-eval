# ~/ros2_ws/src/vehicle_sensors/vehicle_sensors/wheel_odom_node.py
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw: float) -> Quaternion:
    # 平面运动：绕Z轴
    q = Quaternion()
    q.z = math.sin(yaw/2.0)
    q.w = math.cos(yaw/2.0)
    return q

class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')
        self.pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        hz = self.declare_parameter('rate_hz', 50.0).get_parameter_value().double_value
        self.dt = 1.0 / hz
        # 简单运动学：匀速直线（可改成转弯）
        self.vx = self.declare_parameter('vx', 0.2).get_parameter_value().double_value  # m/s
        self.wz = self.declare_parameter('wz', 0.0).get_parameter_value().double_value  # rad/s
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'Wheel odom started at {hz:.1f} Hz, vx={self.vx:.2f} m/s, wz={self.wz:.2f} rad/s')

    def tick(self):
        # 更新位姿（平面运动）
        self.yaw += self.wz * self.dt
        self.x   += self.vx * math.cos(self.yaw) * self.dt
        self.y   += self.vx * math.sin(self.yaw) * self.dt

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation = yaw_to_quat(self.yaw)
        msg.twist.twist.linear.x = float(self.vx)
        msg.twist.twist.angular.z = float(self.wz)
        # 简单协方差（示意）
        msg.pose.covariance[0] = 1e-3
        msg.pose.covariance[7] = 1e-3
        msg.pose.covariance[35] = 1e-2
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(WheelOdomNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
