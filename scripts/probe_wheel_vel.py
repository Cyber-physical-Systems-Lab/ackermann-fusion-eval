#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

class Probe(Node):
    def __init__(self):
        super().__init__('probe_wheel_vel2')
        topic = self.declare_parameter('topic', '/wheel_vel').get_parameter_value().string_value
        rel   = self.declare_parameter('reliability', 'besteffort').get_parameter_value().string_value
        self.out = self.declare_parameter('out', '').get_parameter_value().string_value

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE if rel.lower().startswith('rel') else ReliabilityPolicy.BEST_EFFORT

        self.recv = 0
        self.last_t = time.time()      # 窗口起点（每秒重置）
        self.max_dt = 0.0              # 这一秒内的“最大相邻到达间隔”
        self.last_msg_t = None         # 上一条消息到达时间（用于相邻间隔）

        self.create_subscription(Float32, topic, self.cb, qos)
        # 每秒打印一次
        self.create_timer(1.0, self.tick)

        # 立刻告诉你订阅配置
        print(f"[probe] topic={topic} reliability={rel}", flush=True)

    def cb(self, _msg):
        now = time.time()
        if self.last_msg_t is not None:
            gap = now - self.last_msg_t     # 真正的“相邻两条消息间隔”
            if gap > self.max_dt:
                self.max_dt = gap
        self.last_msg_t = now
        self.recv += 1

    def tick(self):
        now = time.time()
        hz = self.recv / max(1e-9, now - self.last_t)
        line = f"[probe] hz={hz:.2f} max_dt={self.max_dt:.4f}s recv={self.recv}"
        print(line, flush=True)
        if self.out:
            with open(self.out, 'a') as f:
                f.write(f"{now:.3f},{hz:.3f},{self.max_dt:.6f},{self.recv}\n")
        # 重置下一秒的统计
        self.recv = 0
        self.last_t = now
        self.max_dt = 0.0

def main():
    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
