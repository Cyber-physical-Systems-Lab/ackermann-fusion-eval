#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

class SimWheel(Node):
    def __init__(self):
        super().__init__('sim_wheel')
        self.pub_w = self.create_publisher(Float32, '/wheel_vel', qos_profile_sensor_data)
        self.pub_a = self.create_publisher(Float32, '/wheel_angle', qos_profile_sensor_data)
        self.dt = 0.005  # 200 Hz
        self.t = 0.0
        self.angle = 0.0
        self.timer = self.create_timer(self.dt, self.tick)

    def tick(self):
        # 0-5s: 0 rad/s；5-10s: +5；10-15s: -5；>15s: 3*sin(0.5Hz)
        if self.t < 5: w = 0.0
        elif self.t < 10: w = 5.0
        elif self.t < 15: w = -5.0
        else: w = 3.0*math.sin(2*math.pi*0.5*(self.t-15))
        self.angle = (self.angle + w*self.dt) % (2*math.pi)

        a, v = Float32(), Float32()
        a.data, v.data = self.angle, w
        self.pub_a.publish(a); self.pub_w.publish(v)
        self.t += self.dt

def main():
    rclpy.init(); n = SimWheel()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
