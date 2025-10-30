#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

class Relay(Node):
    def __init__(self):
        super().__init__('relay_wheel_vel_qos')
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Float32, '/wheel_vel_reliable', pub_qos)
        self.create_subscription(Float32, '/wheel_vel', self.cb, sub_qos)
    def cb(self, msg): self.pub.publish(msg)

def main():
    rclpy.init(); n=Relay()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()