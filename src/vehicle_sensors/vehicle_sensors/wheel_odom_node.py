import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

def yaw_to_quat(yaw: float) -> Quaternion:
    # 平面运动：绕Z轴
    q = Quaternion()
    q.z = math.sin(yaw/2.0)
    q.w = math.cos(yaw/2.0)
    return q

class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')
        hz = self.declare_parameter('rate_hz', 50.0).get_parameter_value().double_value
        self.dt = 1.0 / hz
        self.vx = self.declare_parameter('vx', 0.2).get_parameter_value().double_value
        self.wz = self.declare_parameter('wz', 0.0).get_parameter_value().double_value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value

        self.pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.br  = TransformBroadcaster(self)

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'Wheel odom {hz:.1f} Hz, vx={self.vx:.2f}, wz={self.wz:.2f}')

    def tick(self):
        self.yaw += self.wz * self.dt
        self.x   += self.vx * math.cos(self.yaw) * self.dt
        self.y   += self.vx * math.sin(self.yaw) * self.dt
        q = yaw_to_quat(self.yaw)

        # Odometry
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id  = self.base_frame
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x  = float(self.vx)
        msg.twist.twist.angular.z = float(self.wz)
        msg.pose.covariance[0] = 1e-3
        msg.pose.covariance[7] = 1e-3
        msg.pose.covariance[35] = 1e-2
        self.pub.publish(msg)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(WheelOdomNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
