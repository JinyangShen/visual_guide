"""执行层节点：把 /guide_command 转成底盘速度指令。

按"方案 A 契约"实现（详见 guide_interfaces/EXECUTION_CONTRACT.md）：

  1. 只依据 GuideCommand.command 这一个字段动作，command 共 5 个取值：
       MODE_STOP(0) / MODE_FORWARD(1) / MODE_LEFT(2) / MODE_RIGHT(3)
       / MODE_CAUTION(4)
  2. 忽略所有辅助字段：reason / confidence / search_direction /
     regression_distance / ultrasonic_* / lateral_error_m 等一律不读。
     尤其 search_direction 是"给上层的建议"，执行层拿到它也不能驱动车轮。
  3. 不订阅 /search_hint。搜索建议属于上层（人工遥控 / 导航）的通道。
  4. 自带指令看门狗：超过 watchdog_timeout 没收到新指令就自行停车。
     上游崩溃时不会发"我挂了"的通知，只有本地看门狗能保证底盘停下。

输入：
  /guide_command   (guide_interfaces/GuideCommand)  上层下发的运动指令，10 Hz

输出：
  /chassis/velocity (geometry_msgs/Twist)  底盘速度指令
  /executor/status  (std_msgs/String)       执行层状态（监控用，不参与决策）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from guide_interfaces.msg import GuideCommand

from guide_executor.chassis_driver import ChassisDriver


class ExecutorNode(Node):
    """执行层：把上层指令翻译成底盘速度，并保证任何异常都停下来。"""

    def __init__(self):
        """初始化参数、订阅与控制定时器。"""
        super().__init__('executor')

        # ---- 看门狗 ----
        # /guide_command 是 10 Hz 发布，0.5s 超时意味着连续丢 5 帧即停车
        self.declare_parameter('watchdog_timeout', 0.5)
        self.watchdog_timeout = self.get_parameter(
            'watchdog_timeout').value
        self.control_rate = self.declare_parameter(
            'control_rate', 50.0).value

        # ---- 速度参数 ----
        self.max_speed = self.declare_parameter('max_speed', 0.30).value
        self.caution_speed = self.declare_parameter(
            'caution_speed', 0.15).value
        self.turn_speed = self.declare_parameter('turn_speed', 0.40).value
        # CAUTION 时转向角速度的折减比例
        self.caution_ratio = self.declare_parameter(
            'caution_ratio', 0.5).value

        # ---- 加速度限制（真实底盘对突变加速度敏感） ----
        self.max_accel = self.declare_parameter('max_accel', 0.5).value
        self.max_angular_accel = self.declare_parameter(
            'max_angular_accel', 1.0).value

        # ---- 输出话题 ----
        self.output_topic = self.declare_parameter(
            'output_topic', '/chassis/velocity').value
        self.status_topic = self.declare_parameter(
            'status_topic', '/executor/status').value

        # ---- 运行状态 ----
        self.last_cmd_time = None
        self.active_command = GuideCommand.MODE_STOP
        # 上一次的运动模式，MODE_CAUTION 需要"限速继续当前方向"
        self.last_motion = GuideCommand.MODE_STOP
        self.emergency = False
        self.watchdog_fired = False
        self.cmd_count = 0
        self.cur_v = 0.0
        self.cur_w = 0.0

        # ---- 底盘下发 ----
        self.driver = ChassisDriver(self, topic=self.output_topic)
        self.status_pub = self.create_publisher(
            String, self.status_topic, 10)

        # ---- 订阅与定时器 ----
        self.create_subscription(
            GuideCommand, '/guide_command', self.on_guide_command, 10)
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            f'执行层已启动：只依据 command 动作，忽略 reason/search_direction '
            f'等辅助字段；看门狗 {self.watchdog_timeout:.2f}s，'
            f'底盘话题 {self.output_topic}')

    def on_guide_command(self, msg):
        """接收上层指令。

        契约要点：这里只取 command 一个字段。reason / confidence /
        search_direction / regression_distance / ultrasonic_* 等字段
        一律不参与运动决策。
        """
        self.last_cmd_time = self.get_clock().now()
        self.active_command = msg.command
        self.emergency = bool(msg.emergency)
        self.cmd_count += 1

        # 记住上一次的运动方向，供 MODE_CAUTION"限速继续当前方向"使用
        if msg.command in (GuideCommand.MODE_FORWARD,
                           GuideCommand.MODE_LEFT,
                           GuideCommand.MODE_RIGHT):
            self.last_motion = msg.command

        if self.watchdog_fired:
            self.get_logger().info('指令恢复，解除看门狗停车状态')
            self.watchdog_fired = False

    def map_command(self, command):
        """把 command 映射成目标 (线速度, 角速度)。

        只读 command，不读消息里的任何其他字段。未知取值按停车处理
        （fail-safe），宁可停也不要猜。
        """
        if command == GuideCommand.MODE_FORWARD:
            return self.max_speed, 0.0
        if command == GuideCommand.MODE_LEFT:
            return 0.0, self.turn_speed
        if command == GuideCommand.MODE_RIGHT:
            return 0.0, -self.turn_speed
        if command == GuideCommand.MODE_CAUTION:
            # 限速继续当前方向：沿用上一次的运动模式
            if self.last_motion == GuideCommand.MODE_LEFT:
                return 0.0, self.turn_speed * self.caution_ratio
            if self.last_motion == GuideCommand.MODE_RIGHT:
                return 0.0, -self.turn_speed * self.caution_ratio
            return self.caution_speed, 0.0
        # MODE_STOP 以及任何未知取值 -> 立即停车
        return 0.0, 0.0

    def apply_ramp(self, target_v, target_w, dt):
        """加速度限幅，避免速度突变。

        :param target_v: 目标线速度
        :param target_w: 目标角速度
        :param dt: 控制周期，单位秒
        :return: 限幅后的 (线速度, 角速度)
        """
        max_dv = self.max_accel * dt
        max_dw = self.max_angular_accel * dt
        dv = target_v - self.cur_v
        dw = target_w - self.cur_w
        dv = max(-max_dv, min(max_dv, dv))
        dw = max(-max_dw, min(max_dw, dw))
        self.cur_v += dv
        self.cur_w += dw
        return self.cur_v, self.cur_w

    def control_loop(self):
        """控制循环：看门狗检查 -> 速度规划 -> 限幅 -> 下发。"""
        now = self.get_clock().now()

        # ---- 1. 指令看门狗 ----
        # 上游断流（感知进程崩溃、网线松了）时执行层必须自己停下来。
        timed_out = False
        elapsed = 0.0
        if self.last_cmd_time is not None:
            elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9
            if elapsed > self.watchdog_timeout:
                timed_out = True

        if timed_out:
            if not self.watchdog_fired:
                self.get_logger().error(
                    f'指令看门狗超时（{elapsed:.2f}s > '
                    f'{self.watchdog_timeout:.2f}s），执行层自行停车')
                self.watchdog_fired = True
            self.active_command = GuideCommand.MODE_STOP
            # 注意：不清空 last_motion。STOP 只表示"停住"，不代表朝向改变；
            # 保留它，后续的 MODE_CAUTION 才能正确地"限速继续当前方向"。

        # ---- 2. command -> 目标速度 ----
        target_v, target_w = self.map_command(self.active_command)

        # ---- 3. 限幅 ----
        if self.active_command == GuideCommand.MODE_STOP:
            # 契约要求 MODE_STOP 立即停车，不做减速滑行；
            # emergency 时更是硬停，所以不走斜坡。
            self.cur_v = 0.0
            self.cur_w = 0.0
        else:
            target_v, target_w = self.apply_ramp(
                target_v, target_w, 1.0 / self.control_rate)

        # ---- 4. 下发 ----
        self.driver.send(self.cur_v, self.cur_w)
        self.publish_status(self.cur_v, self.cur_w, timed_out)

    def publish_status(self, v, w, timed_out):
        """发布执行层状态，供监控与日志使用。"""
        names = {
            GuideCommand.MODE_STOP: 'STOP',
            GuideCommand.MODE_FORWARD: 'FORWARD',
            GuideCommand.MODE_LEFT: 'LEFT',
            GuideCommand.MODE_RIGHT: 'RIGHT',
            GuideCommand.MODE_CAUTION: 'CAUTION',
        }
        mode_name = names.get(self.active_command, 'UNKNOWN')
        state = 'WATCHDOG_STOP' if timed_out else 'OK'
        self.status_pub.publish(String(data=(
            'mode={} v={:.3f} w={:.3f} emergency={} watchdog={} cmds={}'
        ).format(mode_name, v, w, self.emergency, state, self.cmd_count)))

    def destroy_node(self):
        """退出前强制停车，避免底盘失控。"""
        try:
            self.driver.stop()
        except Exception:  # noqa: BLE001 - 退出路径不应再抛异常
            pass
        return super().destroy_node()


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = ExecutorNode()
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
