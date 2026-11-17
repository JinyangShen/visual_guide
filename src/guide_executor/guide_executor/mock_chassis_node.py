"""开发/测试用的模拟底盘（差速运动学模型）。

没有真机时，用它替代真实底盘形成闭环：
  /chassis/velocity  ->  mock_chassis  ->  /odom  ->  上层定位与停滞检测

注意：真机运行时不要启动本节点，否则会和 rtabmap 的 /odom 抢发布权，
导致定位跳变。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class MockChassisNode(Node):
    """模拟底盘：按差速模型积分位姿。"""

    def __init__(self):
        """初始化参数与订阅。"""
        super().__init__('mock_chassis')

        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('velocity_topic', '/chassis/velocity')
        self.declare_parameter('odom_topic', '/odom')

        self.control_rate = self.get_parameter('control_rate').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.create_subscription(
            Twist, self.get_parameter('velocity_topic').value,
            self.on_velocity, 10)
        self.create_timer(1.0 / self.control_rate, self.update)

    def on_velocity(self, msg):
        """接收底盘速度指令。"""
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        """积分位姿并发布里程计与 TF。"""
        dt = 1.0 / self.control_rate
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = odom.pose.pose.orientation.z
        tf.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = MockChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
