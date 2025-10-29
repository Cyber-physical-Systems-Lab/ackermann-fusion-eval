#!/usr/bin/env python3
import time, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus

# ------------- BNO055 常量 ----------------
BNO_ADDR = 0x28  

# 寄存器（Page 0）
REG_CHIP_ID   = 0x00
REG_OPR_MODE  = 0x3D
REG_PWR_MODE  = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_UNIT_SEL  = 0x3B
REG_PAGE_ID   = 0x07

# 数据寄存器
REG_ACC_DATA = 0x08   # 6 bytes: X LSB/MSB, Y, Z
REG_GYR_DATA = 0x14   # 6 bytes
REG_QUAT_W   = 0x20   # 8 bytes: W LSB/MSB, X, Y, Z

# 模式
OPR_MODE_CONFIG = 0x00
OPR_MODE_IMU    = 0x08   # IMU模式：加计+陀螺融合
OPR_MODE_NDOF   = 0x0C   # 全融合（若磁力计可靠可用）

# 电源模式
PWR_MODE_NORMAL = 0x00

def to_int16(lsb, msb):
    val = (msb << 8) | lsb
    if val >= 32768:
        val -= 65536
    return val

class BNO055I2C(Node):
    def __init__(self):
        super().__init__('bno055_i2c_node')

        # -------- 参数 --------
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', BNO_ADDR)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('use_ndof', False)  # True→NDOF，False→IMU
        # 缩放可调
        self.declare_parameter('acc_scale_m_s2', 1.0/100.0)             # raw/100 → m/s^2
        self.declare_parameter('gyr_scale_rad_s', (math.pi/180.0)/16.0)  # raw/16 dps → rad/s
        self.declare_parameter('quat_scale', 1.0/16384.0)

        self.addr = self.get_parameter('i2c_addr').get_parameter_value().integer_value
        bus_id = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.dt = 1.0 / self.rate_hz
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.use_ndof = self.get_parameter('use_ndof').get_parameter_value().bool_value
        self.acc_scale = self.get_parameter('acc_scale_m_s2').get_parameter_value().double_value
        self.gyr_scale = self.get_parameter('gyr_scale_rad_s').get_parameter_value().double_value
        self.quat_scale = self.get_parameter('quat_scale').get_parameter_value().double_value

        self.bus = SMBus(bus_id)

        # 初始化 IMU
        self.init_bno055()

        # 发布者
        self.pub = self.create_publisher(Imu, '/imu', 50)

        # 定时器
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'BNO055 I2C node started @ {self.rate_hz:.1f} Hz, addr=0x{self.addr:02X}')

    # 低级 I2C 读写
    def wr(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def rd(self, reg, n=1):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    def init_bno055(self):
        # 切到 CONFIG 模式
        self.wr(REG_OPR_MODE, OPR_MODE_CONFIG)
        time.sleep(0.02)

        # 选择 Page 0
        self.wr(REG_PAGE_ID, 0x00)
        time.sleep(0.01)

        # 软复位（可选）
        # self.wr(REG_SYS_TRIGGER, 0x20)
        # time.sleep(0.65)

        # 正常电源
        self.wr(REG_PWR_MODE, PWR_MODE_NORMAL)
        time.sleep(0.01)

        # 单位选择（保持默认或设为 SI 单位）
        # 单位寄存器各bit定义较繁琐，这里保持默认，再用缩放系数转换
        # 如需强制SI，可尝试 self.wr(REG_UNIT_SEL, 0x00)

        # 进入融合模式
        mode = OPR_MODE_NDOF if self.use_ndof else OPR_MODE_IMU
        self.wr(REG_OPR_MODE, mode)
        time.sleep(0.02)

        # 简单检测
        chip_id = self.rd(REG_CHIP_ID, 1)[0]
        if chip_id not in (0xA0, ):  # 一些模块返回 0xA0 为正确信号
            self.get_logger().warn(f'Unexpected CHIP ID: 0x{chip_id:02X}')

    def read_accel(self):
        b = self.rd(REG_ACC_DATA, 6)
        x = to_int16(b[0], b[1]) * self.acc_scale
        y = to_int16(b[2], b[3]) * self.acc_scale
        z = to_int16(b[4], b[5]) * self.acc_scale
        return x, y, z

    def read_gyro(self):
        b = self.rd(REG_GYR_DATA, 6)
        x = to_int16(b[0], b[1]) * self.gyr_scale
        y = to_int16(b[2], b[3]) * self.gyr_scale
        z = to_int16(b[4], b[5]) * self.gyr_scale
        return x, y, z

    def read_quat(self):
        b = self.rd(REG_QUAT_W, 8)
        w = to_int16(b[0], b[1]) * self.quat_scale
        x = to_int16(b[2], b[3]) * self.quat_scale
        y = to_int16(b[4], b[5]) * self.quat_scale
        z = to_int16(b[6], b[7]) * self.quat_scale
        # 归一化（有时会有轻微偏差）
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-6:
            w /= norm; x /= norm; y /= norm; z /= norm
        return x, y, z, w

    def tick(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 角速度
        gx, gy, gz = self.read_gyro()
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        # 协方差（先给保守常数，可后续标定）
        msg.angular_velocity_covariance[0] = 1e-4
        msg.angular_velocity_covariance[4] = 1e-4
        msg.angular_velocity_covariance[8] = 1e-4

        # 线加速度
        ax, ay, az = self.read_accel()
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.linear_acceleration_covariance[0] = 5e-3
        msg.linear_acceleration_covariance[4] = 5e-3
        msg.linear_acceleration_covariance[8] = 5e-3

        # 姿态（四元数，NDOF/IMU 模式都可提供）
        try:
            qx, qy, qz, qw = self.read_quat()
            msg.orientation.x = float(qx)
            msg.orientation.y = float(qy)
            msg.orientation.z = float(qz)
            msg.orientation.w = float(qw)
            # 给较小协方差（后续可调）
            msg.orientation_covariance[0] = 1e-3
            msg.orientation_covariance[4] = 1e-3
            msg.orientation_covariance[8] = 1e-3
        except Exception as e:
            # 若读失败，标记未提供姿态
            msg.orientation_covariance[0] = -1.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = BNO055I2C()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
