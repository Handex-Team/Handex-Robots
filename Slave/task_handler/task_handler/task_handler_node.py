#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import subprocess
import threading

class TaskHandler(Node):
    def __init__(self):
        super().__init__('task_handler_node')

        # 订阅 /task 话题
        self.subscription = self.create_subscription(
            Int8,
            '/task',
            self.task_callback,
            10)

        # 发布 /arm_state 话题
        self.publisher_ = self.create_publisher(Int8, '/arm_state', 10)

        # 当前状态消息：0 = busy，1 = idle
        self.state_msg = Int8()
        self.state_msg.data = 1  # 默认空闲

        # 定时器：每 100ms 发布一次当前状态
        self.timer = self.create_timer(0.1, self.publish_state)

        # 状态锁
        self.is_busy = False

        self.get_logger().info("TaskHandler node started...")

    def publish_state(self):
        self.publisher_.publish(self.state_msg)

    def task_callback(self, msg):
        if msg.data == 15 and not self.is_busy:
            self.get_logger().info("Received task 15. Starting processing...")
            threading.Thread(target=self.handle_task).start()

    def handle_task(self):
        self.is_busy = True

        # 设置状态为 busy（不直接发布，只设置值）
        self.state_msg.data = 0
        self.get_logger().info("Set arm_state to 0 (busy)")

        # 执行耗时操作（运行另一个脚本）
        try:
            subprocess.run(['python3', 'heavy_script.py'], check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Script failed: {e}")
        else:
            self.get_logger().info("Script finished successfully")

        # 设置状态为 idle（等下一个定时器周期自动发布）
        self.state_msg.data = 1
        self.get_logger().info("Set arm_state to 1 (idle)")

        self.is_busy = False

def main(args=None):
    rclpy.init(args=args)
    node = TaskHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
