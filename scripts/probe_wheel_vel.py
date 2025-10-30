
#!/usr/bin/env python3
#自动统计hz/最大间隔
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

class Probe(Node):
    def __init__(self):
        super().__init__('probe_wheel_vel')
        rel = self.declare_parameter('reliability', 'besteffort').get_parameter_value().string_value
        depth = self.declare_parameter('depth', 10).get_parameter_value().integer_value
        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT if rel=='besteffort' else ReliabilityPolicy.RELIABLE,
            history = HistoryPolicy.KEEP_LAST, depth = depth)
        self.last = None; self.n=0; self.max_dt=0.0; self.t0=time.time()
        self.create_subscription(Float32, '/wheel_vel', self.cb, qos)
        self.create_timer(1.0, self.tick)

    def cb(self, msg):
        t=time.time()
        if self.last is not None:
            dt=t-self.last
            if dt>self.max_dt: self.max_dt=dt
        self.last=t; self.n+=1

    def tick(self):
        el=time.time()-self.t0
        hz=self.n/el if el>0 else 0.0
        self.get_logger().info(f"reliability={self.get_parameter('reliability').value} hz={hz:.2f} max_dt={self.max_dt:.4f}s recv={self.n}")
        self.n=0; self.max_dt=0.0; self.t0=time.time()

def main():
    rclpy.init(); n=Probe()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
