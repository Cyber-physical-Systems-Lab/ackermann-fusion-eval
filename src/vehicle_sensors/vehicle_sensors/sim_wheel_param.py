#!/usr/bin/env python3
#python3 src/vehicle_sensors/vehicle_sensors/sim_wheel_param.py --ros-args -p rate_hz:=200.0 -p amplitude:=3.0 -p sine_hz:=0.5（以200举例)

import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

class SimWheel(Node):
    def __init__(self):
        super().__init__('sim_wheel')
        p = self.declare_parameter
        self.rate_hz   = p('rate_hz', 200.0).get_parameter_value().double_value  # 50/100/200
        self.amp       = p('amplitude', 3.0).get_parameter_value().double_value  # 正弦幅值(rad/s)
        self.freq_hz   = p('sine_hz', 0.5).get_parameter_value().double_value    # 正弦频率(Hz)
        self.dt = 1.0 / max(1e-3, self.rate_hz)

        self.pub_w = self.create_publisher(Float32, '/wheel_vel', qos_profile_sensor_data)
        self.pub_a = self.create_publisher(Float32, '/wheel_angle', qos_profile_sensor_data)
        self.t = 0.0
        self.angle = 0.0
        self.timer = self.create_timer(self.dt, self.tick)

    def tick(self):
        w = self.amp * math.sin(2*math.pi*self.freq_hz*self.t)
        self.angle = (self.angle + w*self.dt) % (2*math.pi)
        a, v = Float32(), Float32()
        a.data, v.data = self.angle, w
        self.pub_a.publish(a); self.pub_w.publish(v)
        self.t += self.dt

def main():
    rclpy.init(); n = SimWheel()
    try: rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
