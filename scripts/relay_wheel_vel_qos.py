#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

class Relay(Node):
    def __init__(self):
        super().__init__('relay_wheel_vel_qos')

        self.in_topic  = self.declare_parameter('in_topic',  '/wheel_vel').get_parameter_value().string_value
        self.out_topic = self.declare_parameter('out_topic', '/wheel_vel_forward').get_parameter_value().string_value
        rel            = self.declare_parameter('reliability', 'besteffort').get_parameter_value().string_value
        self.abs_value = self.declare_parameter('abs_value', False).get_parameter_value().bool_value

        self.pub_qos = QoSProfile(depth=10)
        self.pub_qos.reliability = ReliabilityPolicy.RELIABLE if rel.lower().startswith('rel') else ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(Float32, self.in_topic, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32, self.out_topic, self.pub_qos)

        self.get_logger().info(f"in={self.in_topic} → out={self.out_topic} rel={rel} abs_value={self.abs_value}")

    def cb(self, msg: Float32):
        v = abs(msg.data) if self.abs_value else msg.data
        self.pub.publish(Float32(data=v))

def main():
    rclpy.init()
    rclpy.spin(Relay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()