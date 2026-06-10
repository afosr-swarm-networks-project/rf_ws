#!/usr/bin/env python3
import sys
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rf_msgs.msg import MoveCommand, Odometry


class DummyRobotController(Node):
    def __init__(self) -> None:
        super().__init__("dummy_robot_controller")

        # Declare parameters for initial position
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.current_x = float(self.get_parameter("initial_x").value)
        self.current_y = float(self.get_parameter("initial_y").value)
        self.current_yaw = float(self.get_parameter("initial_yaw").value)

        self.latest_cmd = None
        self.cmd_lock = threading.Lock()

        # Create publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.cmd_sub = self.create_subscription(
            MoveCommand, "move_cmd", self._on_move_cmd, 10
        )

        # Background controller thread
        self.running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

        # Publish initial state
        self.publish_odom()
        self.get_logger().info(
            f"DummyRobotController initialized at x={self.current_x:.2f}, y={self.current_y:.2f}"
        )

    def _on_move_cmd(self, msg: MoveCommand) -> None:
        with self.cmd_lock:
            self.latest_cmd = (msg.target_x, msg.target_y)
            self.get_logger().info(f"Received new target command: ({msg.target_x:.2f}, {msg.target_y:.2f})")

    def publish_odom(self) -> None:
        msg = Odometry()
        msg.x = float(self.current_x)
        msg.y = float(self.current_y)
        msg.yaw = float(self.current_yaw)

        self.odom_pub.publish(msg)

    def _control_loop(self) -> None:
        while rclpy.ok() and self.running:
            # 1. Read command every 1 second
            time.sleep(1.0)

            target = None
            with self.cmd_lock:
                if self.latest_cmd is not None:
                    target = self.latest_cmd
                    self.latest_cmd = None

            if target is not None:
                tx, ty = target
                self.get_logger().info(f"Processing command... delaying 1 second before 'moving'")
                
                # 2. Delay another second
                time.sleep(1.0)

                # 3. Simulate movement (dummy jump to target)
                dx = tx - self.current_x
                dy = ty - self.current_y
                if np.hypot(dx, dy) > 1e-6:
                    self.current_yaw = float(np.arctan2(dy, dx))
                self.current_x = tx
                self.current_y = ty

                self.get_logger().info(f"Arrived at target: ({self.current_x:.2f}, {self.current_y:.2f}). Publishing odom.")
                
                # 4. Publish new odom state
                self.publish_odom()

    def shutdown(self) -> None:
        self.running = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DummyRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
