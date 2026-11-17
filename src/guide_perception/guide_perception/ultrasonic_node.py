"""超声波测距节点（RDK X5 平台）

发布 sensor_msgs/Range 到 /ultrasonic/range，频率 10 Hz。

支持两种数据源（通过参数 simulate 切换）：
  1. simulate=true  : 模拟数据生成（开发初期桩代码，无需硬件）。
  2. simulate=false : 真实硬件读取，预留两种接口：
       - I2C 超声波模块（如 SRF02/国产 I2C 模块，经 RDK X5 的 I2C 总线）
       - GPIO 触发式 HC-SR04（经 RDK X5 的 40pin GPIO）

容错：硬件读取失败时发布 nan 距离并继续运行，不拖垮整个感知系统。
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# 真实硬件驱动库（在 RDK X5 上安装；开发机上允许缺失）
try:
    import smbus2  # pip install smbus2，用于 I2C 超声波模块
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

try:
    import RPi.GPIO as GPIO  # RDK X5 兼容库可用 hobot-gpio 替代
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # ---- 参数声明 ----
        self.declare_parameter('simulate', True)        # 模拟数据开关
        self.declare_parameter('frame_id', 'ultrasonic_link')
        self.declare_parameter('publish_rate', 10.0)    # Hz
        self.declare_parameter('min_range', 0.02)       # 传感器盲区下限 (m)
        self.declare_parameter('max_range', 4.0)        # 传感器量程上限 (m)
        # I2C 硬件参数
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x70)        # SRF02 默认地址
        # GPIO 硬件参数（HC-SR04，BCM 编号）
        self.declare_parameter('gpio_trig', 23)
        self.declare_parameter('gpio_echo', 24)
        # 模拟数据参数
        self.declare_parameter('sim_base_dist', 1.5)    # 模拟基准距离 (m)
        self.declare_parameter('sim_obstacle_prob', 0.05)  # 模拟近距离障碍概率

        self.simulate = self.get_parameter('simulate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        self.pub = self.create_publisher(Range, '/ultrasonic/range', 10)
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        # 真实硬件初始化
        self.hw = None
        if not self.simulate:
            self.hw = self._init_hardware()

        self.sim_t = 0.0
        mode = 'SIMULATION' if self.simulate else 'HARDWARE'
        self.get_logger().info(f'ultrasonic_node started in {mode} mode @ {rate} Hz')

    # ============================================================
    # 真实硬件接口（桩代码：在 RDK X5 上按需实现）
    # ============================================================
    def _init_hardware(self):
        """初始化 I2C 或 GPIO 硬件。返回硬件句柄，失败返回 None。"""
        if HAS_SMBUS:
            try:
                bus = smbus2.SMBus(self.get_parameter('i2c_bus').value)
                self.get_logger().info(
                    f'I2C ultrasonic ready on bus {self.get_parameter("i2c_bus").value}, '
                    f'addr 0x{self.get_parameter("i2c_addr").value:02X}')
                return ('i2c', bus)
            except Exception as e:
                self.get_logger().warn(f'I2C init failed: {e}')

        if HAS_GPIO:
            try:
                trig = self.get_parameter('gpio_trig').value
                echo = self.get_parameter('gpio_echo').value
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(trig, GPIO.OUT)
                GPIO.setup(echo, GPIO.IN)
                self.get_logger().info(f'HC-SR04 ready: TRIG={trig}, ECHO={echo}')
                return ('gpio', trig, echo)
            except Exception as e:
                self.get_logger().warn(f'GPIO init failed: {e}')

        self.get_logger().error('No hardware interface available, publishing NaN.')
        return None

    def _read_i2c(self, bus):
        """读取 I2C 超声波模块（以 SRF02 协议为例，按实际模块数据手册修改）。

        SRF02 流程：写寄存器0=0x51 启动测距(cm) -> 延时约65ms -> 读寄存器2/3。
        注意：10Hz 下每周期100ms，65ms 延时在预算内；更高频率需改用异步状态机。
        """
        addr = self.get_parameter('i2c_addr').value
        bus.write_byte_data(addr, 0, 0x51)          # 启动测距，结果单位 cm
        time.sleep(0.065)
        high = bus.read_byte_data(addr, 2)
        low = bus.read_byte_data(addr, 3)
        return ((high << 8) | low) / 100.0          # cm -> m

    def _read_gpio(self, trig, echo):
        """读取 HC-SR04：Trig 拉高 10us，测量 Echo 高电平时间换算距离。"""
        GPIO.output(trig, False)
        time.sleep(0.002)
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        t_timeout = time.time() + 0.03             # 30ms 超时（约 5m）
        while GPIO.input(echo) == 0:
            pulse_start = time.time()
            if pulse_start > t_timeout:
                return float('nan')
        while GPIO.input(echo) == 1:
            pulse_end = time.time()
            if pulse_end > t_timeout:
                return float('nan')
        # 声速 343 m/s，往返除以 2
        return (pulse_end - pulse_start) * 343.0 / 2.0

    def _read_hardware(self):
        """统一硬件读取入口，返回距离（米），失败返回 nan。"""
        if self.hw is None:
            return float('nan')
        try:
            if self.hw[0] == 'i2c':
                return self._read_i2c(self.hw[1])
            elif self.hw[0] == 'gpio':
                return self._read_gpio(self.hw[1], self.hw[2])
        except Exception as e:
            self.get_logger().warn(f'hardware read failed: {e}')
            return float('nan')
        return float('nan')

    # ============================================================
    # 模拟数据生成
    # ============================================================
    def _read_simulated(self):
        """生成模拟距离：基准值 + 缓慢漂移 + 噪声，小概率出现近距障碍。"""
        self.sim_t += 0.1
        base = self.get_parameter('sim_base_dist').value
        drift = 0.3 * math.sin(self.sim_t * 0.5)     # 模拟缓慢移动
        noise = random.gauss(0.0, 0.02)              # 传感器噪声
        dist = base + drift + noise
        # 小概率注入近距离障碍，用于测试融合急停逻辑
        if random.random() < self.get_parameter('sim_obstacle_prob').value:
            dist = random.uniform(0.1, 0.4)
        return max(self.min_range, min(self.max_range, dist))

    # ============================================================
    # 发布
    # ============================================================
    def on_timer(self):
        dist = self._read_simulated() if self.simulate else self._read_hardware()

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        # 视场角：HC-SR04 约 15°，I2C 模块按手册修改
        msg.field_of_view = math.radians(15.0)
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = dist
        self.pub.publish(msg)

    def destroy_node(self):
        # 释放 GPIO 资源
        if self.hw is not None and self.hw[0] == 'gpio' and HAS_GPIO:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
