"""底盘指令下发接口。

把执行层算出来的速度 (v, w) 变成真正送到底盘的指令。

输出话题通过参数配置，默认 /chassis/velocity：
  - /chassis/velocity：项目内部总线，便于用 mock 底盘做闭环验证；
  - /cmd_vel          ：真机若底盘驱动是 ros2_control 的 diff_drive_controller，
                       或底盘自身订阅 nav2 的 /cmd_vel，把 output_topic 改成
                       /cmd_vel 即可，不需要改任何代码。

真机串口 / CAN 接入位置见 send() 的注释。
"""

from geometry_msgs.msg import Twist


class ChassisDriver:
    """把速度指令下发到底盘话题。"""

    def __init__(self, node, topic='/chassis/velocity'):
        """创建发布器。

        :param node: 所属 ROS 节点
        :param topic: 底盘速度指令话题
        """
        self.node = node
        self.topic = topic
        self.pub = node.create_publisher(Twist, topic, 10)
        self.last_sent = (0.0, 0.0)

    def send(self, linear_x, angular_z):
        """下发一次速度指令。

        :param linear_x: 线速度，单位 m/s
        :param angular_z: 角速度，单位 rad/s（正值左转）
        """
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.pub.publish(twist)
        self.last_sent = (twist.linear.x, twist.angular.z)
        # 真机接入点：串口 / CAN 驱动在这里把 twist 翻译成硬件报文，例如
        #   ser.write(struct.pack('<ff', twist.linear.x, twist.angular.z))
        #   bus.send(can_id, [int(twist.linear.x * 100), int(twist.angular.z * 100)])

    def stop(self):
        """强制停车（节点退出时调用，避免底盘失控）。"""
        self.send(0.0, 0.0)
